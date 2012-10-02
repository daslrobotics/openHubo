#!/usr/bin/env python

""" OpenHubo servo functions """

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time

#TODO: Work with the concept of activeDOF?

def testMotionRange(robot,jointName,steps=50,timestep=.05):
""" Demonstrate the range of motion of a joint.
        This quick test shows the limit-checking of the servo plugin.
        It attempts to reach +/-180 deg. for each joint.  However, the actual
        reference position will be clipped to within a few degrees of the
        limits."""
    joint=robot.GetJoint(jointName)
    dq=360.0
    q0=-180.0

    for k in [x*dq/steps+q0 for x in range(steps+1)]:
        print k
        robot.GetController().SendCommand('setpos1 {} {}'.format(joint.GetDOFIndex(),k))
        time.sleep(timestep)

def sendServoCommandByLimb(robot,trunk=zeros(4),la=zeros(7),ra=zeros(7),ll=zeros(6),rl=zeros(6),lf=zeros(15),rf=zeros(15)):
""" Build up a full-body pose by limb.
        This function builds a trajectory by limb, allowing you to work with
        arms, legs, and fingers independently. It's not the most efficient
        implementation, but it can be convenient to hack with."""
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

def sendServoCommand(robot,raw):
""" Send an array of servo positions directly to the robot. """
    #build command string to pass to the servo controller.
    positions=array(zeros(len(robot.GetDOF())))
    for k in range(len(raw)):
        positions[k]=raw[k]

    strtmp = 'setpos '+' '.join(str(f) for f in positions)
    robot.GetController().SendCommand(strtmp)

def sendSparseServoReference(robot,posDict):
    """ Update only joints that are specified in the dictionary (not implemented
    yet) """
    positions=zeros(len(robot.GetDOF()))
    print "Not yet implented!"

def sendSingleJointTrajectory(robot,trajectory,jointID,timestep=.1):
    """ Send a trajectory that will be played back for a single joint """
    #TODO: time by sim timesteps (i.e. manually step simulation)
    for k in trajectory:
        strtmp = 'setpos1 {} {}'.format(jointID,k)
        robot.GetController().SendCommand(strtmp)
        time.sleep(timestep)
        
def sendSingleJointTrajectory(robot,trajectory,jointID,timestep=.1):
    """ Send a trajectory that will be played back for a single joint """
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
            pass
        t=t+tstep

        #Build servo control command
        strtmp = 'setpos1 {} {}'.format(jointID,k)
        robot.GetController().SendCommand(strtmp)

        print "Simulation Time: {}".format((env.GetSimulationTime()-starttime)/1000000.0)

""" Simple test script to run some of the functions above. """
if __name__=='__main__':
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    env.Load(file_env)

    timestep=0.0005

    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=timestep)

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


    t=array([k/100.0 for k in range(500)])
    A=10.0
    traj=cos(.5*2.0*pi*t)*A-A

    sendSingleJointTrajectorySim(robot,traj,robot.GetJoint('LEP').GetDOFIndex(),timestep,100)
    #sendSingleJointTrajectory(robot,traj,robot.GetJoint('LEP').GetDOFIndex(),.1)
    #Run this in interactive mode to preserve the state
