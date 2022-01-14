/* photoresistor_reader.cpp is a ROS node that parses the ROS data from the 4 shelf Teensys. This code 
 * works with the photoresistor version of the Shelf. Every time an object is place or removed from a
 * fiducial marker, a message will be printed to the console that gives details about the event that
 * occured. This version defines a photoresistor light threshold value that when crossed, updates whether
 * an object is plaecd or not. This makes the photoresistors not sensitive and sometimes causes false positives,
 * but the initial state of the shelf doesn't need to be known. 
 */

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>

#define LIGHT_THRESHOLD 1400

bool prevObjectStatus1[10];
bool prevObjectStatus2[10];
bool prevObjectStatus3[10];
bool prevObjectStatus4[10];

void readPhotoresistorMsg(std_msgs::UInt32MultiArray msg, int board)
{
    //Loop through all of the photoresistor data
    for(int currentSensor = 0; currentSensor < msg.data.size(); currentSensor++)
    {
        //Figure out whether an object is currently present or not
        bool currentStatus = msg.data[currentSensor] > LIGHT_THRESHOLD;

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
        if(currentStatus != prevStatus)
        {
            //Log the state change
            if(currentStatus)
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

    ros::spin();

    return 0;
}