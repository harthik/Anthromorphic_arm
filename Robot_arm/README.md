# Anthromorphic_arm
The motor_pub.cpp file contains the publisher node. This node publishes the desired incremental joint angles for the robot to reach a desired position. It publishes on the joint_angles topic which is recived by motor_sub code on the esp32. The esp32 moves the stepper motor as desired.

# Steps to set up the communication.
# Publisher

A cpp package should be created using ros2 pkg create and the motor_pub.cpp code can be uploaded to the src folder.

# Steps to upload the subscriber to the esp32 board.

We need the espidf setup and idf.py available.

We first need to create an empty project through the espidf on vscode.

In the main folder, replace the main.c file with the main.c file in motor_sub folder of this repository.

Add a components folder in the same level as the main folder.

Clone the [micro-ros ](https://github.com/micro-ROS/micro_ros_espidf_component)library into the components directory.

Follow the steps provided in the micro-ros page to run the subscriber node on the esp32.
