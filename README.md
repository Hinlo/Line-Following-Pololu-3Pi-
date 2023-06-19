# Line-Following-Pololu-3Pi-
This project programs the Pololu 3Pi+ wheeled robot for line following of dark lines on a light background, utilising the IR sensors. 

A PID controller was developed and tuned to enable smoother motion. Odometry allows the robots position relative to the starting point to be tracked, enabling the robot to return to start upon reaching the track end.

The program is robust to short breaks in the track as well as sharp corners.
File details are given below.

## Final Code.ino
This is the primary looping file which initiates the robot set up. This is the run file to upload to the robot. Pin choices are made based on the Pololu 3Pi+ pin layout. The user guide for this robot can be found [here](https://www.pololu.com/docs/0J83). 

## encoders.h
The encoders enable the counting of wheel rotations and therefore are used to track robot position on a 2D plane. This file simply instantiates the encoders, and is imported into **kinematics.h** for application to the odometry calculation.

## fsm.h
This is the Finite State Machine. It imports the other files and includes all the state functions as well as a function to select which state is appropriate based on linesensor and kinematics data, the state choice ultimately affects the instruction sent to the motors.

## kinematics.h
Imports the **encoders.h** and **motors.h** files to perform calculations of robot position on a 2D plane (x-y coordinates) and angle relative to starting angle (theta).

## linesensor.h
Instantiates the IR sensors, sets the rate of sensing and saves the latest readings of each sensor to an array.

## motors.h
Instantiates the robot wheel motors and sets the maximum allowed wheel rotation speed.

## pid.h
Calculations and tuning for each of the P, I and D terms to return a feedback value to moderate the wheel speeds. The feedback value is the sum of the P, I and D terms.
