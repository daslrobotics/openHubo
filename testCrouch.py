#!/usr/bin/env python
# 08-16-12 OpenRave Tutorial Test Code 
# Hubo

from openravepy import *
import time
import scipy
import tab
from numpy import *
from numpy.linalg import *
import sys
import datetime
from servo import *

if __name__=='__main__':
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(5)
    env.Load(file_env)

    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)
        robot.SetController(RaveCreateController(env,'servocontroller'))
        env.StopSimulation()
        env.StartSimulation(timestep=0.0001)

    time.sleep(3)

    sendServoCommand(robot,array(zeros(60)))
    #Run this in interactive mode to preserve the state

