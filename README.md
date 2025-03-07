Robot arm consists of the publisher and subscriber code for the robot arm.
v1_print consists of the initial model of the robot arm.


### Changes to publisher
The joint piblisher currently can take in a desired position and move to that point before switching off or can directly enter a time dependent path into the code and the desired q dot to follow that path are given.
Write another publisher that takes in a path and publishes the positions to the joint publisher or given a start and end position the joint publisher is incrementally given the positions to move to every 0.5 seconds as that is the rate which works with the stepper motors.
