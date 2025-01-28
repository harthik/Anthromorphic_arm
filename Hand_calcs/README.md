# Hand calculation files to calculate the Torque reuired and also the forward kinematics
The files my_robot.py contain the required code to calculate the forward kinematics and lagrange dynamics for any robot arm.

The usage example is available in the anth_arm_dyne.ipynb and anth_arm_kine.ipynb files.

# Parameters to define for dynamics

The robot has to be described using the DH convention. Additionally the parameter l, which is the center of mass of the link should be specified which is used for dynamics calcultions.

The position of the motor on the link along the x axis of the link.

The mass of the links and the mass of the motors.

The inertia tensor of each of the links with respect to their CG.

The angular accleration and angular velocity of the motors.

# Parameters to define for forward kinematics

The robot has to be described using the DH convention. Additionally the parameter l, which is the center of mass of the link should be specified which is used for dynamics calcultions (not relavent for forward kinematics).
