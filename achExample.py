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
    time.sleep(2)

    setACHReference(robot,{'LSR':50,'RSR':-50,'REP':-20,'LEP':-20})
    itrs=10000
    starttime=time.time();

    for i in range(itrs):
        setACHReference(robot,readACHPacket())
        time.sleep(.1)

    endtime=time.time()
    print "Elapsed time for {} commands is {}".format(itrs,endtime-starttime)
    print "Average cmd time is {}".format((endtime-starttime)/itrs)


def setACHReference(robot,refPos):
    for x in refPos.keys():
        #print "setting joint ",x,"to angle ",refPos[x]
        ind=robot.GetJointIndex(x)
        #This command has less overhead than a full set of angles, though maybe
        # not for only manipulator
        cmd="setpos1 {0} {1}".format(ind,refPos[x])
        #print cmd
        robot.GetController().SendCommand(cmd)

def readACHPacket():
    #Eventually use this function to access the ACH channel and get the most
    #recent set point. For now, make up random values to send
    setPoint={'LSP':0.0,'LSR':80.0,'LSY':0.0,'LEP':-50,'LWY':0.0,'LWP':0.0}
    t=time.time();
    omega=2*numpy.pi/2
    A=numpy.sin(omega*t)*20.0
    for x in setPoint.keys():
        setPoint[x]+=A
        #TODO: figure out why robot explodes when setpoint is too far past limit
        #(probably have to query limits to fix this).

    return setPoint

if __name__=='__main__':
    run()
