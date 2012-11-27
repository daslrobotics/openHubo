#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
#TODO: Work with the concept of activeDOF?

def createTrajectory(robot):
    """ Create a trajectory based on a robot's config spec"""
    traj=RaveCreateTrajectory(robot.GetEnvironment,'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    return traj

""" Simple test script to run some of the functions above. """
if __name__=='__main__':
    file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.0005

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'trajectorycontroller')
        robot.SetController(controller)

        #Set an initial pose before the simulation starts
        controller.SendCommand('set gains 50 0 8')

        #Use the new SetDesired command to set a whole pose at once.
    pose0=array(zeros(robot.GetDOF()))

    #Manually align the goal pose and the initial pose so the thumbs clear
    pose0[ind('RSR')]=-pi/8
    pose0[ind('LSR')]=pi/8

    controller.SetDesired(pose0)
    robot.SetDOFValues(pose0)

    env.StartSimulation(timestep=timestep)

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)

    pose1=pose0.copy()
    print pose1

    pose1[ind('LAP')]=-pi/8
    pose1[ind('RAP')]=-pi/8

    pose1[ind('LKP')]=pi/4
    pose1[ind('RKP')]=pi/4

    pose1[ind('LHP')]=-pi/8
    pose1[ind('RHP')]=-pi/8

    traj=RaveCreateTrajectory(env,'')

    #Set up basic parameters
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()

    traj.Init(config)

    t0=0
    t1=2

    waypt0=list(pose0)
    waypt1=list(pose1)

    waypt0.extend(zeros(7))
    waypt1.extend(zeros(7))

    waypt0.append(t0)
    waypt1.append(t1)
    waypt2=copy(waypt0)
    waypt2[-1]=t1;

    traj.Insert(0,waypt0)
    traj.Insert(1,waypt1)
    traj.Insert(2,waypt2)

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    #Prove that the retiming actually works
    for k in range(40):
        data=traj.Sample(float(k)/10)
        print data[ind('LKP')]
    
    controller.SetPath(traj)
    controller.SendCommand('start')
    while not(controller.IsDone()):
        time.sleep(.1)
