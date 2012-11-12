#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
from recorder import viewerrecorder
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
    try:
        file_env = sys.argv[1]
    except IndexError:
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
        robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
        controller.SendCommand('set gains 50 0 8')
        time.sleep(1)

        #Use the new SetDesired command to set a whole pose at once.
        pose=array(zeros(60))

        #Manually align the goal pose and the initial pose so the thumbs clear
        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        controller.SetDesired(pose)

    env.StartSimulation(timestep=timestep)

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)

    pose0=robot.GetDOFValues()
    pose1=pose0.copy()

    pose1[ind('LAP')]=-20
    pose1[ind('RAP')]=-20

    pose1[ind('LKP')]=40
    pose1[ind('RKP')]=40

    pose1[ind('LHP')]=-20
    pose1[ind('RHP')]=-20
    pose1=pose1*pi/180*2

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

    vidrec=viewerrecorder(env)
    controller.SetPath(traj)
    vidrec.start()
    controller.SendCommand('start')
    while not(controller.IsDone()):
        time.sleep(.1)
    vidrec.stop()
