 /* photoresistor_reader_v2.cpp is a ROS node that parses the ROS data from the 4 shelf Teensys.
  * This code works with the photoresistor version of the Shelf. Every time an object is place or removed 
  * from a fiducial marker, a message will be printed to the console that gives details about the event 
  * that occured. This version of the reader will measure the initial lighting conditions of the shelf 
  * and then determine whether an object is present or not based on that initial reading. This makes the
  * photoresistors more sensitive, but it also requires the assumption that there are no objects present
  * on the shelf when turning on the substation
  */

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <stdlib.h>

//Determines how sensitive the photoresistors are
#define LIGHT_TOLERANCE 600

bool prevObjectStatus1[10];
bool prevObjectStatus2[10];
bool prevObjectStatus3[10];
bool prevObjectStatus4[10];

bool initialized[4] = {false, false, false, false};

unsigned int initialObjectState1[10];
unsigned int initialObjectState2[10];
unsigned int initialObjectState3[10];
unsigned int initialObjectState4[10];

void readPhotoresistorMsg(std_msgs::UInt32MultiArray msg, int board)
{
    if (initialized[board - 1])
    {
        //Loop through all of the photoresistor data
        for (uint8_t currentSensor = 0; currentSensor < msg.data.size(); currentSensor++)
        {
            //Figure out whether an object is currently present or not
            unsigned int initialReadingForCurrentSensor;
            switch (board)
            {
                case 1:
                    initialReadingForCurrentSensor = initialObjectState1[currentSensor];
                    break;
                case 2:
                    initialReadingForCurrentSensor = initialObjectState2[currentSensor];
                    break;
                case 3:
                    initialReadingForCurrentSensor = initialObjectState3[currentSensor];
                    break;
                case 4:
                    initialReadingForCurrentSensor = initialObjectState4[currentSensor];
                    break;
            }

            bool currentStatus = abs((int)(msg.data[currentSensor] - initialReadingForCurrentSensor)) > LIGHT_TOLERANCE;

            //Get the previous status of the current sensor based on which board is currently being processed
            bool prevStatus;
            switch (board)
            {
                case 1:
                    prevStatus = prevObjectStatus1[currentSensor];
                    break;
                case 2:
                    prevStatus = prevObjectStatus2[currentSensor];
                    break;
                case 3:
                    prevStatus = prevObjectStatus3[currentSensor];
                    break;
                case 4:
                    prevStatus = prevObjectStatus4[currentSensor];
                    break;
            }

            //Determine if the state of the sensor has changed and log the change if it does
            if (currentStatus != prevStatus)
            {
                //Log the state change
                if (currentStatus)
                {
                    ROS_INFO("An object has been placed on position %d on board %d", currentSensor + 1, board);
                }
                else
                {
                    ROS_INFO("An object has been removed from position %d on board %d", currentSensor + 1, board);
                }
            }

            //Update the previous states
            switch (board)
            {
                case 1:
                    prevObjectStatus1[currentSensor] = currentStatus;
                    break;
                case 2:
                    prevObjectStatus2[currentSensor] = currentStatus;
                    break;
                case 3:
                    prevObjectStatus3[currentSensor] = currentStatus;
                    break;
                case 4:
                    prevObjectStatus4[currentSensor] = currentStatus;
                    break;
            }
        }
    }
    else
    {
        //Get all of the initial photoresistor readings and then compare all of the future readings to the initial values
        for (uint8_t currentSensor = 0; currentSensor < msg.data.size(); currentSensor++)
        {
            switch (board)
            {
                case 1:
                    initialObjectState1[currentSensor] = msg.data[currentSensor];
                    break;
                case 2:
                    initialObjectState2[currentSensor] = msg.data[currentSensor];
                    break;
                case 3:
                    initialObjectState3[currentSensor] = msg.data[currentSensor];
                    break;
                case 4:
                    initialObjectState4[currentSensor] = msg.data[currentSensor];
                    break;
            }
        }

        //set initialized to true so main sensor loop can run
        initialized[board - 1] = true;
    }
}

void lightsCallback1(std_msgs::UInt32MultiArray::ConstPtr msg)
{
    readPhotoresistorMsg(*msg, 1);
}

void lightsCallback2(std_msgs::UInt32MultiArray::ConstPtr msg)
{
    readPhotoresistorMsg(*msg, 2);
}

void lightsCallback3(std_msgs::UInt32MultiArray::ConstPtr msg)
{
    readPhotoresistorMsg(*msg, 3);
}

void lightsCallback4(std_msgs::UInt32MultiArray::ConstPtr msg)
{
    readPhotoresistorMsg(*msg, 4);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "photoresistor_reader");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("lights1", 10, lightsCallback1);
    ros::Subscriber sub2 = nh.subscribe("lights2", 10, lightsCallback2);
    ros::Subscriber sub3 = nh.subscribe("lights3", 10, lightsCallback3);
    ros::Subscriber sub4 = nh.subscribe("lights4", 10, lightsCallback4);

    //the board will start off with no obstacles on it, so initialize all object state variables to false
    for (int i = 0; i < 10; i++)
    {
        prevObjectStatus1[i] = false;
        prevObjectStatus2[i] = false;
        prevObjectStatus3[i] = false;
        prevObjectStatus4[i] = false;
    }

    ros::spin();

    return 0;
}