#!/usr/bin/env python

""" OpenHubo servo functions """

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
import openhubo
import collections
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class ServoTest:

    def __init__(self,filename):
        self.jointdata={}
        self.import_servo_data(filename)

    def import_servo_data(self,filename):

        with open(filename,'r') as f:
            gainstring=f.readline().rstrip()
            names=f.readline().rstrip().split(' ')
            servostrings=f.readlines()

        for l in servostrings:
            data=l.rstrip().split(' ')
            #Store a dictionary of lists?
            self.jointdata.setdefault(data[0],[float(x) for x in data[1:]])

    def plot(self,servolist=['LEP']):
        for s in servolist:
            REF='{}_REF'.format(s)
            plt.plot(self.jointdata[REF],'+',hold=True)
            plt.plot(self.jointdata[s],hold=True)

#TODO: Work with the concept of activeDOF?

shortcuts=collections.namedtuple('Shortcuts',['env','robot','controller','pose'])

def setupEnv():
    env = Environment()
    #env.SetViewer('qtcoin')
    #time.sleep(.5)
    env.SetDebugLevel(DebugLevel.Info)
    timestep=0.0005

    with env:
        env.StopSimulation()
        env.Load('simpleFloor.env.xml')
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'servocontroller')
        robot.SetController(controller)

        #Set an initial pose before the simulation starts
        controller.SendCommand('setgains 50 0 8')

        pose=array(zeros(robot.GetDOF()))

        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        #Set initial pose to avoid thumb collisions
        robot.SetDOFValues(pose)
        controller.SetDesired(pose)
    return shortcuts(env,robot,controller,pose)

def sendServoCommand(robot,raw=array(zeros(60))):
    """ Send an array of servo positions directly to the robot. """
    robot.GetController().SetDesired(raw)

def sendSparseServoCommand(robot,posDict):
    """ Update only joints that are specified in the dictionary."""
    #TODO: check units and scale 
    positions=robot.GetDOFValues()
    
    #Translate from dictionary of names to DOF indices to make a full servo command
    for k in posDict.keys():
        positions[robot.GetJoint(k).GetDOFIndex()]=posDict[k]

    robot.GetController().SetDesired(positions)

def sendSingleJointTrajectory(robot,trajectory,jointID,timestep=.1):
    """ Send a trajectory that will be played back for a single joint."""
    #TODO: time by sim timesteps (i.e. manually step simulation)
    for k in trajectory:
        strtmp = 'setpos1 {} {}'.format(jointID,k)
        robot.GetController().SendCommand(strtmp)
        time.sleep(timestep)

def sendSingleJointTrajectorySim(robot,trajectory,jointID,dt=.0005,rate=20):
    """ Send a single joint trajectory in simulation time.
    This is the equivalent of real-time control on the actual robot, except
    much easier due to our control of simulation time. """

    #Pull in environment pointer from robot
    env=robot.GetEnv()
    # Convert to microseconds
    skip=1.0/(dt*rate)
    tstep=int(dt*1000000.0)*skip
    starttime=env.GetSimulationTime()
    t=starttime

    for k in trajectory:
        #Wait for the simulation thread to complete its timestep (There must be
        # a better way...)
        
        while env.GetSimulationTime()<(t+tstep):
            time.sleep(dt/10)
            pass
        t=t+tstep

        #Build servo control command
        strtmp = 'setpos1 {} {}'.format(jointID,k)
        robot.GetController().SendCommand(strtmp)

        print "Simulation Time: {}".format((env.GetSimulationTime()-starttime)/1000000.0)

""" Examples to learn how to use the new servocontroller."""
if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    time.sleep(.5)
    # 3 = fatal, error, and warnings, but not debug output
    env.SetDebugLevel(3)

    timestep=0.0005

    # Important! Lock the environment and stop the simulation to load a new
    # environment. If you don't, physics will be running BEFORE the controller
    # is defined. This means that the robot starts to crumple, and when the
    # controller is enabled, it will doa  sort of "hypnic jerk", or partially
    # embed in the floor and explode
    with env:
        env.StopSimulation()
        env.Load('simpleFloor.env.xml')
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'servocontroller')
        robot.SetController(controller)

        #Set an initial pose before the simulation starts
        controller.SendCommand('setgains 50 0 8')

        pose=array(zeros(robot.GetDOF()))

        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        #Set initial pose to avoid thumb collisions
        robot.SetDOFValues(pose)
        controller.SetDesired(pose)
        #You can also re-enable simulation later on if you need to do
        # pre-simulation tweaks
    env.StartSimulation(timestep=timestep)
    time.sleep(1)
    controller.SendCommand('record_on')
    #Use the new SetDesired command to set a whole pose at once.
    #Manually align the goal pose and the initial pose so the thumbs clear
    pose[ind('RSR')]=-22.5*pi/180
    pose[ind('LSR')]=22.5*pi/180

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)
    pose[ind('LAP')]=-20*pi/180
    pose[ind('RAP')]=-20*pi/180

    pose[ind('LKP')]=40*pi/180
    pose[ind('RKP')]=40*pi/180

    pose[ind('LHP')]=-20*pi/180
    pose[ind('RHP')]=-20*pi/180

    controller.SetDesired(pose)
    time.sleep(2)

    print "Testing single joint pose"

    controller.SendCommand('set degrees')

    controller.SendCommand('setpos1 {} {} '.format(ind('LSP'),-60))

    time.sleep(2)
    print controller.SendCommand('getpos1 {} '.format(ind('LSP')))

    print "Testing single joint pose"

    controller.SendCommand('set radians')
    
    controller.SendCommand('setpos1 {} {} '.format(ind('LEP'),-pi/4))
    time.sleep(2)
    print controller.SendCommand('getpos1 {} '.format(ind('LEP')))

    filename='recorded_positions.txt'
    controller.SendCommand('record_off {}'.format(filename))

    test=ServoTest(filename)
    servos=['LEP','LWP'] 
    test.plot(servos)
    plt.show()
