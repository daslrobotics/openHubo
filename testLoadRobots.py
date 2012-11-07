#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
import sys
import time
from copy import copy
import openhubo
#TODO: Work with the concept of activeDOF?


""" Simple test script to run some of the functions above. """
if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    huboplus_file='huboplus/huboplus.robot.xml'
    hubo2_file='hubo2/hubo2.robot.xml'

    #-- Set the robot controller and start the simulation
    with env:
        env.Load(huboplus_file)
        huboplus = env.GetRobot('huboplus')
        huboplus.SetTransform([1,0,0,0,1,0,0])

        env.Load(hubo2_file)
        hubo2 = env.GetRobot('hubo2')
        hubo2.SetTransform([1,0,0,0,0,0,0])

        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        hubo2 = env.GetRobot('hubo2')
        #Create a "shortcut" function to translate joint names to indices
        hubo2_ind = openhubo.makeNameToIndexConverter(hubo2)
        hubo2_ind = openhubo.makeNameToIndexConverter(huboplus)



