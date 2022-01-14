/*
   Photoresistor_Ros.ino
   Arduino code that manages the photoresistor version of the shelf readers. The shelf utilizes photoresistors 
   to indicate whether or not an object is covering one of the fiducial markers. External lighting may need to 
   be added to cabinet to ensure proper operation (LED strips, etc.) Publishes data over ROS Serial and data can
   be stored locally on the PC. For more information on working with rosserial, check out this Rosserial tutorial
   here: https://docs.google.com/document/d/13xQBL52k7maRxUfmVaUPzTwgVvA2ilaONJmkvh9hZC0/edit?usp=sharing
   Tested on Teensy 3.6 and 3.2 with teensyduino, results may vary on other boards.Tested on Ubuntu 16.04 with
   ROS Kinetic. Note when setting up all the teensys, to correctly set up each teensy, the number of sensors and the
   pins that they are located on, as well as the lights topic name, needs to be updated.
*/
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#define NUM_SENSORS 5

//Set Analog Pin Definitions for your board here
//used for the target teensy 3.6 device
const int adc_pins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

//Set ADC Precision here
//8 default, 12 recommended to avoid noise, 16 max in hardware but will pick up noise.
const int anlg_rd_precision = 12;

ros::NodeHandle nh;

std_msgs::UInt32MultiArray light_msg;
ros::Publisher lights("lights2", &light_msg);


void setup()
{
  nh.initNode();
  light_msg.data = (unsigned long*)malloc(sizeof(unsigned long) * NUM_SENSORS);
  light_msg.data_length = NUM_SENSORS;


  analogReadResolution(anlg_rd_precision);
  for (int i = 0; i < NUM_SENSORS; i++) {
    light_msg.data[i] = 0;
  }

  nh.advertise(lights);
}



void update_msg(std_msgs::UInt32MultiArray& light_msg) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    light_msg.data[i] = analogRead(adc_pins[i]);
  }
}

void loop()
{
  lights.publish(&light_msg);

  update_msg(light_msg);
  nh.spinOnce();
  delay(20);
}
