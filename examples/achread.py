#!/usr/bin/env python

import tab
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
import matplotlib.pyplot as plt

def analyzeTime(filename):
    with open(filename,'r') as f:
        instring=f.read()
    inlist=instring.split('\n')
    indata=[float(x) for x in inlist[:-2]]
    plt.plot(diff(indata))
    plt.show()
    return indata

""" Simple test script to run some of the functions above. """
if __name__=='__main__':
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'rlhuboplus.robot.xml'
    # Uncomment below to only load specific plugins (could save memory on
    # embedded systems)
    #
    #RaveInitialize(load_all_plugins=False)
    #success = RaveLoadPlugin('libqtcoinrave.so')
    #success = RaveLoadPlugin('libopenmr.so')
    #success = RaveLoadPlugin('librplanners.so')

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.01

    with env:
        env.StopSimulation()
        env.Load(file_env)
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        #Clear physics for kinematic trajectory playback
        env.SetPhysicsEngine(RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'achcontroller')
        robot.SetController(controller)
        controller.SendCommand("SetCheckCollisions false")
        controller.SendCommand("SetReadOnly 1")

        #Set an initial pose before the simulation starts

        #Use the new SetDesired command to set a whole pose at once.
        pose=array(zeros(robot.GetDOF()))

        robot.SetDOFValues(pose)
        controller.SetDesired(pose)

        env.Load('rlhuboplus.ref.robot.xml')
        ref_robot=env.GetRobot('rlhuboplus_ref')
        ref_robot.Enable(False)
        ref_robot.SetController(RaveCreateController(env,'mimiccontroller'))
        controller.SendCommand("SetVisRobot rlhuboplus_ref")

    for l in ref_robot.GetLinks():
        for g in l.GetGeometries():
            g.SetDiffuseColor([.7,.7,0])
            g.SetTransparency(.7)
    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)

    print "tarting..."
    env.StartSimulation(timestep,True)
