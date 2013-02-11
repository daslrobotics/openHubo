#!/usr/bin/env python

import openhubo
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
from trajectory import *
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

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.0005

    [robot,controller,ind,ref,recorder]=openhubo.load_simplefloor(env)

    pose0=array(zeros(robot.GetDOF()))

    controller.SetDesired(pose0)
    robot.SetDOFValues(pose0)

    env.StartSimulation(timestep=timestep)

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
