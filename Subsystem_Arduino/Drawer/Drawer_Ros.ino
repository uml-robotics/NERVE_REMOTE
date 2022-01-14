/*  Drawer_Ros.ino
 *  The arduino code that controls the Drawer subsystem in the CCRI infrastructure. The Drawer performs two main functions, resetting the drawer state after each
 *  test and changing the force required to pull the drawer open before starting a test. To make this work, the Drawer has a two stepper motors, one for setting
 *  the pull force on the drawer, and the other for pulling the drawer back into its initial state. The drawer also has a ToF sensor to detect how far the drawer
 *  has been pulled. To interact with the Drawer, Rostopics are used to trigger the differnet actions and also communicate real-time sensor data. For more information
 *  on getting the code to work with ROS, check out this Rosserial tutorial here: https://docs.google.com/document/d/13xQBL52k7maRxUfmVaUPzTwgVvA2ilaONJmkvh9hZC0/edit?usp=sharing
 */

//necessary libraries
#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h> //TOF sensor library
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <infrastructure_msgs/Drawer.h>

#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"

//reset motor pins
#define pulse_reset 8
#define direction_reset 7
#define enable_reset 6

//friction motor pins
#define pulse_friction 13
#define direction_friction 12
#define enable_friction 11

//FSR402 data pins (from handle)
#define fsr_1 A0
// #define fsr_2 A1
#define fsr_3 A2
#define fsr_4 A3
#define fsr_5 A4
#define fsr_6 A5
#define fsr_7 A6
#define fsr_8 A7
#define fsr_9 A8
#define fsr_10 A9
#define fsr_11 A10
#define fsr_12 A11

#define NUM_OF_FSRS 12

//Determines loop rate if desired
#define LOOP_RATE_HZ 100

//Init the stepper motors
AccelStepper reset_motor( AccelStepper::DRIVER, pulse_reset, direction_reset);
AccelStepper friction_motor( AccelStepper::DRIVER, pulse_friction, direction_friction);

//Init tof sensor
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

int start_pos; //distance of drawer from tof in starting pos, the '0' position.
const int buffer_val = 5; //buffer value for tof sensor for calculating distance
const float fric_steps = .00032; //constant that represents relation between friction setting to motor steps
const float base_friction = .3; //min resistance that drawer has
const int min_steps = 2500; //min steps it takes to get brake to touch drawer fin

const int time_unwind = 4000; //in ms
unsigned long time;
unsigned long time_stop;

//ros variables
ros::NodeHandle n;
infrastructure_msgs::Drawer data;
std_msgs::Bool drawer_status;
ros::Publisher datapub("Drawer/Data", &data);
ros::Publisher statuspub("Drawer/MovementStatus", &drawer_status);

// Timer ISR
void TimerHandler()
{
  friction_motor.runSpeed();
  reset_motor.runSpeed();
}

#define TIMER_INTERVAL_MS        1L


//ros callback functions for start_drawer and reset_drawer services
void reset_drawer_callback(const std_msgs::Empty& msg)
{
  drawer_status.data = true;
  statuspub.publish(&drawer_status);
  reset_drawer();
  drawer_status.data = false;
  statuspub.publish(&drawer_status);
}

void start_drawer_callback(const std_msgs::UInt8& msg)
{
  set_friction(msg.data);
}

ros::Subscriber<std_msgs::Empty> resetsub("Drawer/Reset_Drawer", &reset_drawer_callback);
ros::Subscriber<std_msgs::UInt8> startsub("Drawer/Start_Drawer", &start_drawer_callback);

void setup()
{
  //Setup ros pubs and services
  n.initNode();
  n.advertise(datapub);
  n.advertise(statuspub);
  n.subscribe(resetsub);
  n.subscribe(startsub);

  //Set the baud rate of the serial port
  n.getHardware()->setBaud(250000);
  
  //don't run drawer if TOF is not working.
  uint8_t connection_counter = 0;
  if (!tof.begin()) {
    while (1){
      if(connection_counter >= 3)
        n.logerror("Unable to initialize ToF sensor");
      
      connection_counter++;
      delay(5000);
    }
  }

  //Initialize space in drawer msg
  data.fsr_readings = (unsigned int *)malloc(sizeof(uint8_t) * NUM_OF_FSRS);
  data.fsr_readings_length = NUM_OF_FSRS;
  
  //configure the max speed for the motors
  friction_motor.setMaxSpeed(800);
  reset_motor.setMaxSpeed(800);

  //setup the enable pins on the motors
  reset_motor.setEnablePin(enable_reset);
  friction_motor.setEnablePin(enable_friction);
  
  //setup the used arduino pins
  pinMode(pulse_reset, OUTPUT);
  pinMode(direction_reset, OUTPUT);
  pinMode(pulse_friction, OUTPUT);
  pinMode(direction_friction, OUTPUT);

  pinMode(fsr_1, INPUT);
 // pinMode(fsr_2, INPUT);
  pinMode(fsr_3, INPUT);
  pinMode(fsr_4, INPUT);
  pinMode(fsr_5, INPUT);
  pinMode(fsr_6, INPUT);
  pinMode(fsr_7, INPUT);
  pinMode(fsr_8, INPUT);
  pinMode(fsr_9, INPUT);
  pinMode(fsr_10, INPUT);
  pinMode(fsr_11, INPUT);
  pinMode(fsr_12, INPUT);

  // initialize starting pos of drawer. Assumes drawer is closed.
  start_pos = get_current_drawer_position();

  // Init timer ITimer1
  ITimer1.init();

  ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
}

void loop()
{
  //Publishes sensor data and runs callbacks if needed
  collect_data();
  n.spinOnce();
  
  //Loop rate control
  //uncomment if desired
  /* int loop_end_time = millis() + (1/LOOP_RATE_HZ) * 1000;
  while(millis() < loop_end_time); */

}

void reset_drawer()
{
  n.loginfo("Starting drawer reset");
  reset_friction(); //turn off friction
  reset_motor.setSpeed(-800); //set speed to negative value to change direction
  bool did_move = false;
  n.loginfo("Starting drawer pullback");
  int counter = 0;
  while (true)
  {
    if (get_current_drawer_position() < buffer_val)
    {
      break;
    }
    
    //If loop passes the if statement on the first iteration, the motor will move so set did_move to true
    did_move = true;

    //Run motor from ISR

    //Make sure that main loop keeps updating while in this while loop
    loop();
    
  }
  reset_motor.setSpeed(0); //stop motor
  
  n.loginfo("Starting drawer unwind");
  if (did_move) { //if drawer did not move at all (or did not need to be reeled in), don't unwind
    time = millis();
    time_stop = time + time_unwind;
    reset_motor.setSpeed(800);
    while (time < time_stop) { //unwinds motor so string has slack
      //Run the motor in ISR

      //Make sure that main loop keeps updating while in this while loop
      loop();

      //Update the current time 
      time = millis();
    }
  }
  reset_motor.setSpeed(0); //stop motor
  n.loginfo("Finished drawer reset");
}

void set_friction(float resistance)
{
  float steps = ((resistance - base_friction) / fric_steps) + min_steps;
  friction_motor.setSpeed(800);
  friction_motor.moveTo(steps);
  int counter = 0;
  do {
    //Run the motor in ISR
    
    //Make sure that main loop keeps updating while in this while loop
    loop();
    
  } while (friction_motor.currentPosition() < friction_motor.targetPosition());
  friction_motor.setSpeed(0); //Stop the motor once the motor is in position
}

void reset_friction() {
  n.loginfo("Starting friction reset");
  friction_motor.setSpeed(-800);
  friction_motor.moveTo(0);
  int counter = 0;
  do {
    //Run the motor in ISR
    
    //Make sure that main loop keeps updating while in this while loop
    loop();
    
  } while (friction_motor.currentPosition() > friction_motor.targetPosition());
  friction_motor.setSpeed(0); //Stop the motor once the motor is in position
  n.loginfo("Finished friction reset");
}

void publish_handle_val() {
  data.fsr_readings[0] = analogRead(fsr_1);
//  data.fsr_readings[1] = analogRead(fsr_2);
  data.fsr_readings[2]= analogRead(fsr_3);
  data.fsr_readings[3] = analogRead(fsr_4);
  data.fsr_readings[4] = analogRead(fsr_5);
  data.fsr_readings[5] = analogRead(fsr_6);
  data.fsr_readings[6] = analogRead(fsr_7);
  data.fsr_readings[7] = analogRead(fsr_8);
  data.fsr_readings[8] = analogRead(fsr_9);
  data.fsr_readings[9] = analogRead(fsr_10);
  data.fsr_readings[10] = analogRead(fsr_11);
  data.fsr_readings[11] = analogRead(fsr_12);
}

void publish_TOF_val()
{
  data.drawer_distance = get_current_drawer_position();
  if (data.drawer_distance < 0)
  {
    data.drawer_distance = 0;
  }
}

int get_current_drawer_position()
{
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false); // read the current distance measurement

  if (measure.RangeStatus != 4) // phase failures have incorrect data
    return measure.RangeMilliMeter - start_pos;
  else
    return get_current_drawer_position();
}

void collect_data() {
  publish_handle_val();
  publish_TOF_val();
  data.header.stamp = n.now(); //gets the time of the device roscore is running on
  datapub.publish(&data);
}
