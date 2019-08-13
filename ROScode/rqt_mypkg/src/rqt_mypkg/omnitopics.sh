#!/bin/sh
# Shell script for publishing messages to omnirobot from rqt gui
# Codes: 0 = stop, 1 = time&direction, 2 = velocity, 3 = servo

function=$1

if [function == 0]
then
  rostopic pub arduinoSub omnirobotmessages/Arduinostate '{vel_x: 0.0, vel_y: 0.0, vel_theta: 0.0, toggleVelocity: True}' -r 10
fi
if [function == 1]
then
  rostopic pub arduinoSub omnirobotmessages/Arduinostate '{toggleFunction: True, functionSelect: 1, functionIntParam: %d, functionFloatParam: %d}' --once %(direction,time)
fi
if [function == 2]
then
    rostopic pub arduinoSub omnirobotmessages/Arduinostate '{vel_x: 0.0, vel_y: 0.0, vel_theta: 0.0, toggleVelocity: True}' --once
fi
if [function == 3]
then
    rostopic pub arduinoSub omnirobotmessages/Arduinostate '{vel_x: 0.0, vel_y: 0.0, vel_theta: 0.0, toggleVelocity: True}' --once
fi
