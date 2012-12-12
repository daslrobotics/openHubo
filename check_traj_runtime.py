#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
from trajectory import *
#TODO: Work with the concept of activeDOF?

""" Simple test script to run some of the functions above. """
if __name__=='__main__':
    
    perf_t0=time.time()
    env = Environment()
    #env.SetViewer('qtcoin')
    env.SetDebugLevel(3)

    timestep=0.0005

    [robot,controller,ind]=openhubo.load_simplefloor(env)

    pose0=array(zeros(robot.GetDOF()))

    controller.SetDesired(pose0)
    robot.SetDOFValues(pose0)

    env.StartSimulation(timestep=timestep)

    pose1=pose0.copy()

    pose1[ind('LAP')]=-pi/6
    pose1[ind('RAP')]=-pi/6

    pose1[ind('LKP')]=pi/3
    pose1[ind('RKP')]=pi/3

    pose1[ind('LHP')]=-pi/6
    pose1[ind('RHP')]=-pi/6

    pose1[ind('LSR')]=pi/4
    pose1[ind('RSR')]=-pi/4

    
    pose1[ind('leftIndexKnuckle1')]=pi/8
    pose1[ind('leftMiddleKnuckle1')]=pi/8
    pose1[ind('leftRingKnuckle1')]=pi/8
    pose1[ind('leftPinkyKnuckle1')]=pi/8
    pose1[ind('leftThumbKnuckle1')]=pi/8

    pose1[ind('rightIndexKnuckle1')]=pi/8
    pose1[ind('rightMiddleKnuckle1')]=pi/8
    pose1[ind('rightRingKnuckle1')]=pi/8
    pose1[ind('rightPinkyKnuckle1')]=pi/8
    pose1[ind('rightThumbKnuckle1')]=pi/8

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
    #for k in range(40):
        #data=traj.Sample(float(k)/10)
        #print data[ind('LKP')]
    perf_t1=time.time() 
    controller.SetPath(traj)
    controller.SendCommand('start')
    while not(controller.IsDone()):
        #print "Real time {}, sim time {}".format(t,controller.GetTime())
        #Only approximate time here
        time.sleep(.05)

    perf_t2=time.time()

    print "Sim run time is {}, load time was {}, real run time was {}".format(controller.GetTime(),perf_t1-perf_t0,perf_t2-perf_t1)
    env.Destroy()

