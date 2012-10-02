#!/usr/bin/env python

""" OpenHubo servo functions """

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time

#TODO: Work with the concept of activeDOF?

def sendServoCommandByLimb(robot,trunk=zeros(4),la=zeros(7),ra=zeros(7),ll=zeros(6),rl=zeros(6),lf=zeros(15),rf=zeros(15)):
    #Ugly method of joining up manipulator arrays
    #TODO: Error checking in array length?
    joints=trunk.copy()
    joints=append(append(append(append(append(append(trunk,la),ra),ll),rl),lf),rf)
    #build command string to pass to the servo controller.
    strtmp = 'setpos '+' '.join(str(f) for f in joints)
    robot.GetController().SendCommand(strtmp)
    #Debug desired joint commands to the terminal (Slow...)
    #for f in range(0,60):
    #    print "{}: {}".format(robot.GetJointFromDOFIndex(f).GetName(),joints[f])

""" Send an array of servo positions directly to the robot """
def sendServoCommand(robot,raw):
    #build command string to pass to the servo controller.
    positions=array(zeros(len(robot.GetDOF())))
    for k in range(len(raw)):
        positions[k]=raw[k]

    strtmp = 'setpos '+' '.join(str(f) for f in positions)
    robot.GetController().SendCommand(strtmp)

""" Update only joints that are specified in the dictionary (not implemented
yet) """
def sendSparseServoReference(robot,posDict):
    positions=zeros(len(robot.GetDOF()))
    print "Not yet implented!"

def sendSingleJointTrajectory(robot,trajectory,jointID,timestep=.1):
    """ Send a trajectory that will be played back for a single joint """
    #TODO: time by sim timesteps (i.e. manually step simulation)
    for k in trajectory:
        strtmp = 'setpos1 {} {}'.format(jointID,k)
        robot.GetController().SendCommand(strtmp)
        time.sleep(timestep)
        

if __name__=='__main__':
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
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    time.sleep(1)
    # Avoid setting servo Kp higher than about 1/20 of the update frequency
    #robot.GetController().SendCommand('setgains 50 .5 2  .1 .1')
    #robot.GetController().SendCommand('setpos 0 0 30 -30 0 50')

    qW = array([0,0,0,0])
    offset_LSR = 15;
    offset_RSR = -15;		
    qLA = array([0,0 + offset_LSR,0,0,0,0,0])
    qRA = array([0,-0 + offset_RSR,0,0,0,0,0])
    qLA[3] = -30
    qRA[3] = -30
    qLL = array([0,0,-30,60,-30,0])/20
    qRL = array([0,0,-30,60,-30,0])/20

    robot.GetController().SendCommand('setgains 50 0 3 .1 .1')

    sendServoCommandByLimb(robot,qW,qLA,qRA,qLL,qRL)

    time.sleep(1)

    t=array([k/20.0 for k in range(100)])
    A=20
    traj=cos(2.0*pi*t)*A-A

    sendSingleJointTrajectory(robot,traj,robot.GetJoint('LEP').GetDOFIndex(),.05)
    #Run this in interactive mode to preserve the state

