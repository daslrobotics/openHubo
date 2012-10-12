#!/usr/bin/env python

""" OpenHubo servo functions """

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time

#TODO: Work with the concept of activeDOF?
def makeNameToIndexConverter(robot):
    def convert(name):
        return robot.GetJoint(name).GetDOFIndex()
    return convert

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

def sendServoCommand(robot,raw=array(zeros(60))):
    """ Send an array of servo positions directly to the robot. """
    robot.GetController.SetDesired(raw)

def sendSparseServoCommand(robot,posDict):
    """ Update only joints that are specified in the dictionary."""
    positions=robot.GetDOFValues()*180.0/pi
    #Translate from dictionary of names to DOF indices to make a full servo command
    for k in posDict.keys():
        positions[robot.GetJoint(k).GetDOFIndex()]=posDict[k]

    strtmp = 'setpos '+' '.join(str(f) for f in positions)
    robot.GetController().SendCommand(strtmp)


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
            time.sleep(dt/10)
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
        file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)

    timestep=0.0005

    #-- Set the robot controller and start the simulation
    with env:
        env.Load(file_env)
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'servocontroller')
        robot.SetController(controller)

        #Set an initial pose before the simulation starts
        robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
        controller.SendCommand('setgains 50 0 8')

        env.StopSimulation()
        env.StartSimulation(timestep=timestep)

    time.sleep(1)

    #Use the new SetDesired command to set a whole pose at once.
    pose=array(zeros(60))

    #Manually align the goal pose and the initial pose so the thumbs clear
    pose[ind('RSR')]=-22.5
    pose[ind('LSR')]=22.5

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)
    pose[ind('LAP')]=-20
    pose[ind('RAP')]=-20

    pose[ind('LKP')]=40
    pose[ind('RKP')]=40

    pose[ind('LHP')]=-20
    pose[ind('RHP')]=-20

    controller.SetDesired(pose)
    time.sleep(1)

    t=array([k/100.0 for k in range(500)])
    A=20.0
    traj=cos(.5*2.0*pi*t)*A-A

    sendSingleJointTrajectorySim(robot,traj,robot.GetJoint('LEP').GetDOFIndex(),timestep,100)
    #sendSingleJointTrajectory(robot,traj,robot.GetJoint('LEP').GetDOFIndex(),.1)
    #Run this in interactive mode to preserve the state

