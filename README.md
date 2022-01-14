# NERVE_REMOTE

## About:
  The Nerve Remote package contains all the code used in the UML verison of the CCRI project. Examples of
  projects contained in this package are substation arduino middleware for the door, drawer, shelf, and 
  reset mechanism substation, and ROS test utilities such as the shelf reader.

## Files:
  - Ros_Packages/ - Contains all the ROS Packages developed in the CCRI project
      - infrastructure_msgs/ - ROS package that contains all the custom messages used by the substation arduino code such as the Door custom message.
      - infrastructure_test/ - ROS package that contains some utility tools for testing the substations. The tools include a data parser for both the photoresistor and the RFID versions of the shelf substation and a keyboard launcher that 
  - Subsystem_Arduino/ - Contains all the substation middleware code that is written for the arduino hardware. All the substations use ROS Serial to communicate with higher-level ROS code.
      - Door/ - Contains the arduino code for the Door substation
      - Drawer - Contains the arduino code for the Drawer substation
      - Reset_Mechanism  - Contains the arduino code for the Reset Mechanism substation
      - Shelf - Contains the arduino code for both the photoresistor and the RFID verison of the Shelf substation