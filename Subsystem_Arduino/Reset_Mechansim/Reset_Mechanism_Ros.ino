/*  Reset_Mechanism_Ros.ino
 *  The arduino code that controls the Reset Mechanism. The Reset Mechanism performs two main functions, resetting an object that a robot arm
 *  can manipulate into a repeatable initial state, and to swap the current manipulation object with a new desired manipulation object. To make this work, the Reset
 *  Mechanism has a robot arm attached to a movable carriage for swapping the objects and a linear actuator, a turntable, and string spool for object resetting. To
 *  interact with the mechanism, Rostopics are used to trigger the swap and reset actions and also communicate real-time sensor data. For more information on getting
 *  the code to work with ROS, check out this Rosserial tutorial here: https://docs.google.com/document/d/13xQBL52k7maRxUfmVaUPzTwgVvA2ilaONJmkvh9hZC0/edit?usp=sharing
 */

 /* Things to do in the code still
  *    1. Determine the object locking mechanism for swapping objects and also the arm grabbing mechanism
  *    2. Figure out the stepper motor positions that correspond to the correct travel location on the reset mechanism
  *    3. Figure out the hardware setup of the arduino and set the correct hardware pins
  *    4. Tune the turntable PID loop
  *    5. Maybe set software limits for the min and max positions each of the stepper motors can travel to
  */

//necessary libraries
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <infrastructure_msgs/ResetMechanism.h>

#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include <TimerInterrupt.h>

// #define LOOP_RATE_HZ 100

// Stepper Motor Pins
#define LINEAR_ACTUATOR_STEPPER_PULSE_PIN 0
#define LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN 0
#define LINEAR_ACTUATOR_STEPPER_ENABLE_PIN 0

#define SPOOL_STEPPER_PULSE_PIN 0
#define SPOOL_STEPPER_DIRECTION_PIN 0
#define SPOOL_STEPPER_ENABLE_PIN 0

#define Z_AXIS_STEPPER_PULSE_PIN 0
#define Z_AXIS_STEPPER_DIRECTION_PIN 0
#define Z_AXIS_STEPPER_ENABLE_PIN 0

#define X_AXIS_STEPPER_PULSE_PIN 0
#define X_AXIS_STEPPER_DIRECTION_PIN 0
#define X_AXIS_STEPPER_ENABLE_PIN 0

#define ARM_STEPPER_PULSE_PIN 0
#define ARM_STEPPER_DIRECTION_PIN 0
#define ARM_STEPPER_ENABLE_PIN 0

// DC Motor Pins
#define TURNTABLE_MOTOR_1_ENABLE_PIN 0
#define TURNTABLE_MOTOR_1_DIRECTION_PIN 0
#define TURNTABLE_MOTOR_2_ENABLE_PIN 0
#define TURNTABLE_MOTOR_2_DIRECTION_PIN 0 

// Limit Switch Pins
#define LINEAR_ACTUATOR_SWITCH_PIN 0
#define Z_AXIS_SWITCH_PIN 0
#define X_AXIS_SWITCH_PIN 0

// Photo Interrupt Pin
#define PHOTO_INTERRUPT_PIN A0

// PID Coefficients
#define Kp 1
#define Kd 0.5

// Turntable Rotation Tolerance
#define TURNTABLE_TOLERANCE 2

// Stepper motor tolerance when using positional movement
#define STEPPER_MOTOR_TOLERANCE 5

// Stepper motor encoder locations for carriage
#define COLUMN_1_X_POS 10000
#define COLUMN_2_X_POS 20000
#define COLUMN_3_X_POS 30000
#define COLUMN_4_X_POS 40000
#define COLUMN_5_X_POS 50000
#define COLUMN_6_X_POS 60000

#define ROW_1_Z_POS 40000
#define ROW_2_Z_POS 20000
#define ROW_3_Z_POS 1000

// This defines how far above an object the carriage will move before picking up an object
#define CARRIAGE_INITIAL_PICKUP_POS_OFFSET 1200

#define CARRIAGE_X_HOME 350000
#define CARRIAGE_Z_HOME 1000

// Init the stepper motors
AccelStepper linear_actuator_motor( AccelStepper::DRIVER, LINEAR_ACTUATOR_STEPPER_PULSE_PIN, LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN);
AccelStepper spool_motor( AccelStepper::DRIVER, SPOOL_STEPPER_PULSE_PIN, SPOOL_STEPPER_DIRECTION_PIN);
AccelStepper z_axis_motor( AccelStepper::DRIVER, Z_AXIS_STEPPER_PULSE_PIN, Z_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper x_axis_motor( AccelStepper::DRIVER, X_AXIS_STEPPER_PULSE_PIN, X_AXIS_STEPPER_DIRECTION_PIN);
AccelStepper arm_motor( AccelStepper::DRIVER, ARM_STEPPER_PULSE_PIN, ARM_STEPPER_DIRECTION_PIN);

// PID Controller Setup
PID_v2 PIDController(Kp, 0, Kd, PID::Direct);

// current encoder reading variable
volatile long encoder_reading = 0;
volatile bool turntable_clockwise = true;

// Current object on the reset mechanism
uint8_t current_object;

// Ros variables
ros::NodeHandle n;
infrastructure_msgs::Reset_Mechansim data;
std_msgs::Bool action_status;

ros::Publisher data_pub("Reset_Mechanism/Data", &data);
ros::Publisher status_pub("Reset_Mechanism/Movement_Status", &action_status);

// Ros callback functions for reset_current and reset_drawer services
void reset_current_object_callback(const std_msgs::Empty& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  reset_current_object();
  action_status.data = false;
  statuspub.publish(&action_status);
}

void swap_current_object_callback(const std_msgs::UInt8& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  swap_current_object(msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}

// ROS Subscribers that are meant for motor testing
void move_linear_actuator_motor_to_position_callback(const std_msgs::Int32& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  move_motor_to_position(linear_actuator_motor, msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}


void move_x_axis_motor_to_position_callback(const std_msgs::Int32& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  move_motor_to_position(x_axis_motor, msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}


void move_z_axis_motor_to_position_callback(const std_msgs::Int32& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  move_motor_to_position(z_axis_motor, msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}


void move_spool_motor_to_position_callback(const std_msgs::Int32& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  move_motor_to_position(spool_motor, msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}

void move_arm_motor_to_position_callback(const std_msgs::Int32& msg)
{
  action_status.data = true;
  statuspub.publish(&action_status);
  move_motor_to_position(arm_motor, msg.data);
  action_status.data = false;
  statuspub.publish(&action_status);
}

ros::Subscriber<std_msgs::Empty> reset_sub("Reset_Mechanism/Reset_Current_Object", &reset_current_object_callback);
ros::Subscriber<std_msgs::UInt8> swap_sub("Reset_Mechanism/Swap_Current_Object", &reset_current_object_callback);

ros::Subscriber<std_msgs::Int32> move_linear_actuator_motor_sub("Reset_Mechanism/Move_Linear_Actuator_Motor_To_Position", &move_linear_actuator_motor_to_position_callback);
ros::Subscriber<std_msgs::Int32> move_x_axis_motor_sub("Reset_Mechanism/Move_X_Axis_Motor_To_Position", &move_x_axis_motor_to_position_callback);
ros::Subscriber<std_msgs::Int32> move_z_axis_motor_sub("Reset_Mechanism/Move_Z_Axis_Motor_To_Position", &move_z_axis_motor_to_position_callback);
ros::Subscriber<std_msgs::Int32> move_spool_motor_sub("Reset_Mechanism/Move_Spool_Motor_To_Position", &move_spool_motor_to_position_callback);
ros::Subscriber<std_msgs::Int32> move_arm_motor_sub("Reset_Mechanism/Move_Arm_Motor_To_Position", &move_arm_motor_to_position_callback);



// setup performs all the one-time configuration code that runs when the reset mechanism is first turned on. 
void setup() 
{
  //Setup ros pubs and subscribers
  n.initNode();
  n.advertise(data_pub);
  n.advertise(status_pub);
  
  n.subscribe(reset_sub);
  n.subscribe(swap_sub);

  n.subscribe(move_linear_actuator_motor_sub);
  n.subscribe(move_x_axis_motor_sub);
  n.subscribe(move_z_axis_motor_sub);
  n.subscribe(move_spool_motor_sub);
  n.subscribe(move_arm_motor_sub);

  //Set the baud rate of the serial port
  n.getHardware()->setBaud(250000);

  //Configure pin directions
  pinMode(LINEAR_ACTUATOR_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(LINEAR_ACTUATOR_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(LINEAR_ACTUATOR_STEPPER_ENABLE_PIN, OUTPUT);
  
  pinMode(SPOOL_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(SPOOL_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(SPOOL_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(Z_AXIS_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(Z_AXIS_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(Z_AXIS_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(X_AXIS_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(X_AXIS_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(X_AXIS_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(ARM_STEPPER_PULSE_PIN, OUTPUT);
  pinMode(ARM_STEPPER_DIRECTION_PIN, OUTPUT);
  pinMode(ARM_STEPPER_ENABLE_PIN, OUTPUT);

  pinMode(TURNTABLE_MOTOR_1_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_1_DIRECTION_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_2_ENABLE_PIN, OUTPUT);
  pinMode(TURNTABLE_MOTOR_2_DIRECTION_PIN, OUTPUT);

  pinMode(LINEAR_ACTUATOR_SWITCH_PIN, INPUT);
  pinMode(Z_AXIS_SWITCH_PIN, INPUT);
  pinMode(X_AXIS_SWITCH_PIN, INPUT);

  pinMode(PHOTO_INTERRUPT_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoder_pin), update_encoder, CHANGE);

  //configure the max speed for the motors
  linear_actuator_motor.setMaxSpeed(800);
  spool_motor.setMaxSpeed(800);
  z_axis_motor.setMaxSpeed(800);
  x_axis_motor.setMaxSpeed(800);
  arm_motor.setMaxSpeed(800);

  //configure the acceleration rates of the stepper motors
  linear_actuator_motor.setAcceleration(200);
  spool_motor.setAcceleration(200);
  z_axis_motor.setAcceleration(200);
  x_axis_motor.setAcceleration(200);
  arm_motor.setAcceleration(200);

  //setup the enable pins on the motors
  linear_actuator_motor.setEnablePin(LINEAR_ACTUATOR_STEPPER_ENABLE_PIN);
  spool_motor.setEnablePin(SPOOL_STEPPER_ENABLE_PIN);
  z_axis_motor.setEnablePin(Z_AXIS_STEPPER_ENABLE_PIN);
  x_axis_motor.setEnablePin(X_AXIS_STEPPER_ENABLE_PIN);
  arm_motor.setEnablePin(ARM_STEPPER_ENABLE_PIN);

  //home carriage & linear actuator
  home_motors();
  
}


// ISRs

//Encoder Pin
void update_encoder()
{
  if(turntable_clockwise)
  {
    encoder_reading++;
  }
  else
  {
    encoder_reading--;
  }
}

//Timer
void TimerHandler()
{
  linear_actuator_motor.run();
  spool_motor.run();
  z_axis_motor.run();
  x_axis_motor.run();
}

#define TIMER_INTERVAL_MS        1L

void loop()
{
  publish_sensor_data();
  n.spinOnce();

  //Loop rate control
  //uncomment if desired
  /* int loop_end_time = millis() + (1/LOOP_RATE_HZ) * 1000;
  while(millis() < loop_end_time); */
}

void home_motors()
{
  //Log that motors are being homed
  n.loginfo("Homing carriage and linear actuator motors");

  //change action status publisher
  action_status.data = true;
  statuspub.publish(&action_status);

  bool x_motor_moving = true;
  bool z_motor_moving = true;
  bool linear_actuator_moving = true;

  //Set the stepper motor speeds for the motors that need to be homed
  linear_actuator_motor.setSpeed(-200);
  z_axis_motor.setSpeed(-200);
  x_axis_motor.setSpeed(-200);

  //Loop while any of the motors that are being homed are moving
  while(linear_actuator_moving || x_motor_moving || z_motor_moving)
  {
    //Check if the motor limit switches have been triggered
    if(linear_actuator_moving)
    {
      if(get_linear_actuator_switch())
      {
        //Stop the motors if the limit switches are triggered
        linear_actuator_motor.setSpeed(0);
        linear_actuator_moving = false;
      }
    }
    
    if(z_motor_moving)
    {
      if(get_z_axis_switch())
      {
        z_axis_motor.setSpeed(0);
        z_motor_moving = false;
      }
    }

    if(x_motor_moving)
    {
      if(get_x_axis_switch())
      {
        x_axis_motor.setSpeed(0);
        x_motor_moving = false;
      }
    }
  }

  // The motors are now in their home positions so reset the current positions on the stepper motors
  linear_actuator_motor.setCurrentPosition(0);
  z_axis_motor.setCurrentPosition(0);
  x_axis_motor.setCurrentPosition(0);

  // Log that motors are finished being homed
  n.loginfo("Finished homing carriage and linear actuator motors");

  // Move the carriage and linear actuator to a good setup location
  // TO DETERMINE
  
}

// move_motor_to_position is a test method that moves a specifed motor to a desired position
void move_motor_to_position(AccelStepper motor, int desired_position)
{
  n.loginfo("Testing motor position movement");

  //Command the desired motor to move to the desired position
  motor.moveTo(desired_position);

  //Wait until the motor has reached its desired position
  while( abs(motor.currentPosition() - motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  n.loginfo("The motor has reached its desired position");
  
}

/* reset_current_object performs the reset function on the Reset Mechanism. This involves pulling the current object back to the center of the turntable and then
   rotating the turntale to reset the objects orientation */
   
void reset_current_object()
{
  n.loginfo("Resetting current object");

  // Pull the object back into its initial position
  spool_motor.move(-30000);

  // Wait until the motor has reached its desired position
  while( abs(spool_motor.currentPosition() - spool_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Rotate the table back to its initial position
  move_turntable_to_position(0);

  // Unwind the spool motor to allow slack in the reset string
  spool_motor.move(30000);

  //Wait until the motor has reached its desired position
  while( abs(spool_motor.currentPosition() - spool_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  n.loginfo("Finished resetting current object");
}


/* swap_current_object performs the swap function on the Reset Mechanism. This involves grabbing the current object with the robot arm on the mechanism, placing it back
 *  on the object shelf, and grabbing and placing the new desired object on the turntable.
 */
void swap_current_object(uint8_t new_object)
{
  n.loginfo("Swapping current object on the reset mechansim");
  
  // Check if the new object is the object on the reset mechanism already
  if( new_object == current_object )
  {
    n.loginfo("The desired object is already on the reset mechansim. Stopping swap algorithm");
    return;
  }

  int z_position_of_new_object;
  int x_position_of_new_object;

  int z_position_of_old_object;
  int x_position_of_old_object;
  
  // Determine where the carraige needs to travel to
  switch(new_object)
  {
    case 0:
      x_position_of_new_object = COLUMN_1_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 1:
      x_position_of_new_object = COLUMN_2_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 2:
      x_position_of_new_object = COLUMN_3_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 3:
      x_position_of_new_object = COLUMN_4_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 4:
      x_position_of_new_object = COLUMN_5_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 5:
      x_position_of_new_object = COLUMN_6_X_POS;
      z_position_of_new_object = ROW_3_Z_POS;
      break;
    case 6:
      x_position_of_new_object = COLUMN_1_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 7:
      x_position_of_new_object = COLUMN_2_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 8:
      x_position_of_new_object = COLUMN_3_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 9:
      x_position_of_new_object = COLUMN_4_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 10:
      x_position_of_new_object = COLUMN_5_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 11:
      x_position_of_new_object = COLUMN_6_X_POS;
      z_position_of_new_object = ROW_2_Z_POS;
      break;
    case 12:
      x_position_of_new_object = COLUMN_1_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    case 13:
      x_position_of_new_object = COLUMN_2_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    case 14:
      x_position_of_new_object = COLUMN_3_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    case 15:
      x_position_of_new_object = COLUMN_4_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    case 16:
      x_position_of_new_object = COLUMN_5_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    case 17:
      x_position_of_new_object = COLUMN_6_X_POS;
      z_position_of_new_object = ROW_1_Z_POS;
      break;
    default:
      n.loginfo("The entered Object ID was invalid");
      return;
  }

  switch(old_object)
  {
    case 0:
      x_position_of_old_object = COLUMN_1_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 1:
      x_position_of_old_object = COLUMN_2_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 2:
      x_position_of_old_object = COLUMN_3_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 3:
      x_position_of_old_object = COLUMN_4_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 4:
      x_position_of_old_object = COLUMN_5_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 5:
      x_position_of_old_object = COLUMN_6_X_POS;
      z_position_of_old_object = ROW_3_Z_POS;
      break;
    case 6:
      x_position_of_old_object = COLUMN_1_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 7:
      x_position_of_old_object = COLUMN_2_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 8:
      x_position_of_old_object = COLUMN_3_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 9:
      x_position_of_old_object = COLUMN_4_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 10:
      x_position_of_old_object = COLUMN_5_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 11:
      x_position_of_old_object = COLUMN_6_X_POS;
      z_position_of_old_object = ROW_2_Z_POS;
      break;
    case 12:
      x_position_of_old_object = COLUMN_1_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
    case 13:
      x_position_of_old_object = COLUMN_2_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
    case 14:
      x_position_of_old_object = COLUMN_3_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
    case 15:
      x_position_of_old_object = COLUMN_4_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
    case 16:
      x_position_of_old_object = COLUMN_5_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
    case 17:
      x_position_of_old_object = COLUMN_6_X_POS;
      z_position_of_old_object = ROW_1_Z_POS;
      break;
  }

  // Time to start swapping. First release the old object
  // TODO
  
  // Move the carraige over the current object
  x_axis_motor.moveTo(CARRIAGE_X_HOME);
  z_axis_motor.moveTo(CARRIAGE_Z_HOME + CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motors have reached their desired position
  while( abs(x_axis_motor.currentPosition() - x_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE || abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE)
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Swing the pickup arm over the object
  arm_motor.moveTo(300);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Now lower the arm down onto the object
  z_axis_motor.move(-CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motor has reached its desired position
  while( abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Pickup the current object
  // TODO FIGURE OUT PICKUP MECHANISM

  // Move the carriage to place the old object back in its storage position
  x_axis_motor.moveTo(x_position_of_old_object);
  z_axis_motor.moveTo(z_position_of_old_object + CARRIAGE_INITIAL_PICKUP_POS_OFFSET);
  
  //Wait until the motors have reached their desired position
  while( abs(x_axis_motor.currentPosition() - x_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE || abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE)
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Swing the pickup arm over the shelf
  arm_motor.move(2000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Now lower the arm down onto the object
  z_axis_motor.move(-CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motor has reached its desired position
  while( abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Release the object
  // TODO FIGURE OUT PICKUP MECHANISM

  // Swing the arm back into a safe traveling position before moving the carriage
  arm_motor.move(1000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }
  
  // Move the carriage to pick up the new object
  x_axis_motor.moveTo(x_position_of_new_object);
  z_axis_motor.moveTo(z_position_of_new_object + CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motors have reached their desired position
  while( abs(x_axis_motor.currentPosition() - x_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE || abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE)
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Swing the pickup arm over the object
  arm_motor.move(-1000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Now lower the arm down onto the object
  z_axis_motor.move(-CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motor has reached its desired position
  while( abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Pickup the new object
  // TODO FIGURE OUT PICKUP MECHANISM

  // Raise the carriage a little before moving the arm
  z_axis_motor.move(CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motor has reached its desired position
  while( abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Swing the arm back into a safe traveling position before moving the carriage
  arm_motor.move(1000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Move the carriage to the base of the reset mechanism
  x_axis_motor.moveTo(CARRIAGE_X_HOME);
  z_axis_motor.moveTo(CARRIAGE_Z_HOME + CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motors have reached their desired position
  while( abs(x_axis_motor.currentPosition() - x_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE || abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE)
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Swing the arm over the reset mechansim
  arm_motor.move(1000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Lower the arm over the reset mechanism
  z_axis_motor.move(-CARRIAGE_INITIAL_PICKUP_POS_OFFSET);

  //Wait until the motor has reached its desired position
  while( abs(z_axis_motor.currentPosition() - z_axis_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  // Release the new object
  // TODO FIGURE OUT PICKUP MECHANISM

  // Secure the new object
  // TODO

  // Rotate the arm into a safe storage space
  arm_motor.move(1000);

  //Wait until the motor has reached its desired position
  while( abs(arm_motor.currentPosition() - arm_motor.targetPosition()) > STEPPER_MOTOR_TOLERANCE )
  {
    //continue running the main loop while the motor is moving
    loop()
  }

  n.loginfo("Finished swapping");
}


// get_linear_actuator_switch returns whether the limit switch on the linear actuator is pressed or not
bool get_linear_actuator_switch()
{
  return !digitalRead(LINEAR_ACTUATOR_SWITCH_PIN);
}

// get_z_axis_switch returns whether the limit switch on the z axis of the carriage is pressed or not
bool get_z_axis_switch()
{
  return !digitalRead(Z_AXIS_SWITCH_PIN);
}

// get_x_axis_switch returns whether the limit switch on the x axis of the carriage is pressed or not
bool get_x_axis_switch()
{
  return !digitalRead(X_AXIS_SWITCH_PIN);
}

// publish_sensor_data updates the reset mechanism ros msg with the most current sensor data and publishes the updated msg
void publish_sensor_data()
{
  data.turntable_position = encoder_reading;
  
  data.linear_actuator_position = linear_actuator_motor.currentPosition();
  data.spool_position = spool_motor.currentPosition();
  data.x_axis_position = z_axis_motor.currentPosition();
  data.z_axis_position = x_axis_motor.currentPosition();
  data.arm_position = arm_motor.currentPosition();

  data.linear_actuator_switch = get_linear_actuator_switch();
  data.z_axis_switch = get_z_axis_switch();
  data.x_axis_actuator_switch = get_x_axis_switch();

  data.current_object = current_object;
  
  data.header.stamp = n.now(); //gets the time of the device roscore is running on
  datapub.publish(&data);
}

// move_turntable_to_position uses a PID controller to rotate the turntable to a desired angle
void move_turntable_to_position(int desired_position)
{
  //PID Initialization
  PIDController.Start(encoder_reading,        // input
                      0,                      // current output
                      desired_position);      // setpoint

  // Run the motors while the position of the turntable is not close to the desired position
  while( abs(encoder_reading - desired_position) < TURNTABLE_TOLERANCE) )
  {
    // Determine which direction the turntable must spin to reach its target
    int output = PIDController.Run(encoder_reading);
    if( desired_position < encoder_reading )
    {
      set_turntable_motors(false, output);
    }
    else
    {
      set_turntable_motors(true, output);
    }
  }

  //Once the turntable has rotated to the desired position, ensure that the motors have stopped moving
  set_turntable_motors(true, 0);
}

// set_turntable_motors sets the speed and the direction of the turntable motors and thus rotates the turntable
void set_turntable_motors(bool clockwise, uint8_t motor_speed)
{
  //Store the current direction the motor is turning so that the encoder can count in the right direction
  turntable_clockwise = clockwise;

  //Set the voltages on the motor1 output pins to set the desired motor output
  digitalWrite(TURNTABLE_MOTOR_1_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_1_ENABLE_PIN, motor_speed);

  //Set the voltages on the motor2 output pins to set the desired motor output
  digitalWrite(TURNTABLE_MOTOR_2_DIRECTION_PIN, clockwise);
  analogWrite(TURNTABLE_MOTOR_2_ENABLE_PIN, motor_speed);
}
