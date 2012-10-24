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
from servo import *

if __name__=='__main__':
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)
        robot.SetController(RaveCreateController(env,'servocontroller'))
        env.StartSimulation(timestep=0.001)

    time.sleep(3)
    robot.GetController().SendCommand('set degrees')

    sendSparseServoCommand(robot,{'LHP':-20,'LKP':40,'LAP':-20,'RHP':-20,'RKP':40,'RAP':-20})

    #Run this in interactive mode to preserve the state

