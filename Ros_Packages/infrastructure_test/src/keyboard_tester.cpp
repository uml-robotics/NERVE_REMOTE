 /* keyboard_tester.cpp is a ROS node that allows the different substations to be interacted with
  * using a keyboard. This allows quick testing on the substations and allows for actions to be
  * triggered with a key press instead of a long ROS topic command. So far, q will reset the door, 
  * w will activate the electromagnets in the door, e will reset the Drawer, and r will engage the
  * brake in the drawer.
  */

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>


int main(int argc, char **argv)
{
    //Initialize this node
    ros::init(argc, argv, "substation_node_manager");
    ros::NodeHandle nh;

    ros::Publisher door_reset_pub = nh.advertise<std_msgs::Empty>("door_reset_keyboard_pub", 10);
    ros::Publisher door_start_pub = nh.advertise<std_msgs::UInt8>("door_start_keyboard_pub", 10);
    ros::Publisher drawer_reset_pub = nh.advertise<std_msgs::Empty>("drawer_reset_keyboard_pub", 10);
    ros::Publisher drawer_start_pub = nh.advertise<std_msgs::UInt8>("drawer_start_keyboard_pub", 10);

    std_msgs::Empty empty_msg;
    std_msgs::UInt8 uint8_msg;
    uint8_msg.data = 100;

    HANDLE hIn;
    INPUT_RECORD InRec;
    DWORD NumRead;

    hIn = GetStdHandle(STD_INPUT_HANDLE);

    while(ros::ok())
    {
        //check if a key has been pressed
        ReadConsoleInput(hIn, &InRec, 1, &NumRead);
                      
        if (InRec.EventType == KEY_EVENT && InRec.Event.KeyEvent.uChar.AsciiChar == 'q')
        {
            door_reset_pub.publish(empty_msg);
        }

        if (InRec.EventType == KEY_EVENT && InRec.Event.KeyEvent.uChar.AsciiChar == 'w')
        {
            door_start_pub.publish(uint8_msg);
        }

        if (InRec.EventType == KEY_EVENT && InRec.Event.KeyEvent.uChar.AsciiChar == 'e')
        {
            drawer_reset_pub.publish(empty_msg);
        }

        if (InRec.EventType == KEY_EVENT && InRec.Event.KeyEvent.uChar.AsciiChar == 'r')
        {
            drawer_start_pub.publish(uint8_msg);
        }

        ros::spinOnce();
    }

    return 0;
}