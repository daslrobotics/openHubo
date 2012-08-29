#!/usr/bin/env python

from openravepy import *
from numpy import *
import time
import sys


def run():

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()

    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    env.Load(file_env)

    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        #define ODE physics engine and set gravity

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    mypos1={'TY':45,'LHR':20,'LHY':20,'LHP':0}
    setACHReference(robot,mypos1)
    time.sleep(2)
    mypos2={'TY':0,'LHR':20,'LHY':20,'LHP':0}
    setACHReference(robot,mypos2)

    raw_input('Press any key to quit...')

def setACHReference(robot,refPos):
    for x in refPos.keys():
        print "setting joint ",x,"to angle ",refPos[x]
        ind=robot.GetJointIndex(x)
        cmd="setpos1 {0} {1}".format(ind,refPos[x])
        print cmd
        robot.GetController().SendCommand(cmd)


if __name__=='__main__':
    run()
