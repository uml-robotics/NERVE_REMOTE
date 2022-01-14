/* rfid_reader.cpp is a ROS node that parses the ROS data from the 4 shelf
 * Teensys. This code works with the RFID verison of the shelf. Every time
 * an object is place or removed from a fiducial marker, a message will be printed
 * to the console that gives details about the event that occured.
 */

#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <stdlib.h>
#include <string.h>

//Determines how sensitive the photoresistors are
#define LIGHT_TOLERANCE 200

bool prevObjectStatus1[10];
bool prevObjectStatus2[10];
bool prevObjectStatus3[10];
bool prevObjectStatus4[10];

void readRfidMsg(std_msgs::ByteMultiArray msg, int board)
{
    //Loop through all of the photoresistor data
    for(int currentSensor = 0; currentSensor < msg.data.size() / 10; currentSensor++)
    {
        //Figure out if a tag has been detected
        bool nonZeroTag = false;
        for(int currentByte = 0; currentByte < 10; currentByte++)
        {
            //stop checking if a non zero number was been found
            if(!nonZeroTag)
            {
                nonZeroTag = msg.data[currentSensor * 10 + currentByte] != 0;
            }
        }

        //Determine which tag was found
        std::string tagID = "";
        if(nonZeroTag)
        {
            for(int currentByte = 0; currentByte < 10; currentByte++)
            {
                tagID += (std::to_string(msg.data[currentSensor * 10 + currentByte]) + " ");
            }
        }

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
        if(nonZeroTag != prevStatus)
        {
            //Log the state change
            if(nonZeroTag)
            {
                std::string msg = "An object with an ID of " + tagID + "has been placed on position " + std::to_string(currentSensor + 1) + " on board " + std::to_string(board);
                ROS_INFO("%s", msg.c_str());
            }
            else
            {
                std::string msg = "An object with an ID of " + tagID + "has been removed from position " + std::to_string(currentSensor + 1) + " on board " + std::to_string(board);
                ROS_INFO("%s", msg.c_str());
            }
        }

        //Update the previous states
        switch (board)
        {
            case 1:
                prevObjectStatus1[currentSensor] = nonZeroTag;
                break;
            case 2:
                prevObjectStatus2[currentSensor] = nonZeroTag;
                break;
            case 3:
                prevObjectStatus3[currentSensor] = nonZeroTag;
                break;
            case 4:
                prevObjectStatus4[currentSensor] = nonZeroTag;
                break;
        }
    }
}

void rfidCallback1(std_msgs::ByteMultiArray::ConstPtr msg)
{
    readRfidMsg(*msg, 1);
}

void rfidCallback2(std_msgs::ByteMultiArray::ConstPtr msg)
{
    readRfidMsg(*msg, 2);
}

void rfidCallback3(std_msgs::ByteMultiArray::ConstPtr msg)
{
    readRfidMsg(*msg, 3);
}

void rfidCallback4(std_msgs::ByteMultiArray::ConstPtr msg)
{
    readRfidMsg(*msg, 4);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rfid_reader");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("rfid1", 10, rfidCallback1);
    ros::Subscriber sub2 = nh.subscribe("rfid2", 10, rfidCallback2);
    ros::Subscriber sub3 = nh.subscribe("rfid3", 10, rfidCallback3);
    ros::Subscriber sub4 = nh.subscribe("rfid4", 10, rfidCallback4);

    ros::spin();

    return 0;
}