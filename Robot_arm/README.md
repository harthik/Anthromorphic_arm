# Anthromorphic_arm
The motor_pub.cpp file contains the publisher node which publishes the desired incremental joint angles for the robot to reach a desired position. This node publishes on the joint_angles topic which is recived by motor_sub code on the esp32 which recives the joint angles and moves the stepper motor as desired.

# Steps to set up the communication.
# Publisher

A cpp package should be created using ros2 pkg create and the code in motor_pub.cpp can be uploaded into the src folder.

# Steps to upload the subscriber to the esp32 board.

We need to have the espidf setup and idf.py available.

Firstly we need to create an empty project through the espidf on vscode.

In the main folder replace the main.c file with the main.c file in motor_sub folder.

Add a components filder in the same level as the main folder.

Clone the [micro-ros ](https://github.com/micro-ROS/micro_ros_espidf_component)library into the components directory.

We can go ahead and follow the steps provided in the micro-ros page to run the subscriber node on the esp32.
