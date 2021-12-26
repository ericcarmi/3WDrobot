# 3WD omnidirectional robot

This repo has code for Arduino (control) and ROS (network, GUI).
Also contains CAD files: camera mount, stepper motor/linear actuator to move a microphone up and down. 

The motiviation for this EEE Master's project was from a physics Senior project (an undergrad course) where I designed, 3D printed, and measured an acoustic lens in the anechoic chamber at CSUS. Measuring was tedious and not very precise, so making a robot to do the measuring seemed like a good next step. I only had one semester to do the EEE Master's project, so the solution is not "pretty". I brought the chassis for the robot but I did everything else myself: programming the controls (kinematics for 3-wheel drive), designing and 3D printer attachments (camera holders, linear actuator for mic movement), and programming the UI to control and monitor the robot (in ROS).  


![Robot in the anechoic chamber](robot.png?raw=true "3WD robot")
