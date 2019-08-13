#!/usr/bin/python
'''
Mapping class for use inside rqt
'''
from numpy import *


class Map():
    def __init__(self):

        # Local robot state variables
        self.x = 0
        self.y = 0
        self.t = 0   # Orientation
        self.z = 0   # Mic height
        self.linCalibParam = 1
        self.rotCalibParam = 1
        self.micCalibParam = 1

    def initialize(self):
        x = 0
        y = 0
        t = 0
        z = 0


    # Calibrate direction and time for forward, back, rotations
    def updatePosition(self,direction,time):
        speed = 1 # Constant, but could be changed by passing in the PWM values
        print(direction)
        if(direction == 0):
            dist = self.linCalibParam * speed * time
            self.x += dist * cos(self.t)
            self.y += dist * sin(self.t)
        if(direction == 1):
            dist = -1*self.linCalibParam * speed * time
            self.x += dist * cos(self.t)
            self.y += dist * sin(self.t)
        if(direction == 8):
            rot = self.rotCalibParam * time
            self.t += rot
            self.t = mod(self.t,2*pi)
        if(direction == 9):
            rot = -self.rotCalibParam * time
            self.t += rot
            self.t = mod(self.t,2*pi)

    def updateMicHeight(self,direction,time):
        if(direction == 0):
            self.z += self.micCalibParam * time
        else:
            self.z += -self.micCalibParam * time
