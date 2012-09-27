# 08-16-12 OpenRave Tutorial Test Code 
# Hubo

from openravepy import *
import time
import scipy
from numpy import *
from numpy.linalg import *
import sys
import datetime

def sendServoCommand(robot,trunk=zeros(4),la=zeros(7),ra=zeros(7),ll=zeros(6),rl=zeros(6),lf=zeros(15),rf=zeros(15)):
    #Ugly method of joining up manipulator arrays
    #TODO: Error checking in array length?
    joints=trunk.copy()
    joints=append(append(append(append(append(append(trunk,la),ra),ll),rl),lf),rf)
    #build command string to pass to the servo controller.
    strtmp = 'setpos '+' '.join(str(f) for f in joints)
    #Note that the servo command does not currently do limit checking...make
    # sure you don't command a position outside the allowable range!
    robot.GetController().SendCommand(strtmp)
    #Debug desired joint commands to the terminal (Slow...)
    for f in range(0,60):
        print "{}: {}".format(robot.GetJointFromDOFIndex(f).GetName(),joints[f])
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

time.sleep(3)
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
qLL = array([0,0,-30,60,-30,0])/2
qRL = array([0,0,-30,60,-30,0])/2

sendServoCommand(robot,qW,qLA,qRA,qLL,qRL)
raw_input("Press any key to quit...")

