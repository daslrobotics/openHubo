#!/usr/bin/env python

import openhubo
from openravepy import RaveCreateController
import matplotlib.pyplot as plt
from numpy import diff

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

    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(1)

    timestep=0.01

    [robot,ctrl,ind,__,recorder]=openhubo.load_scene(env,options)

    #Override default controller with ach-read controller
    ctrl=RaveCreateController(env,'achreadcontroller')
    robot.SetController(ctrl)
    ref_ghost=openhubo.load_ghost(env,options.robotfile,"ref_",[.7,1,.7])
    ctrl.SendCommand('SetRefRobot '+ref_ghost.GetName())

    #Uncomment this section if you want to visualize the commands separately
    #cmd_ghost=openhubo.load_ghost(env,options.robotfile,"cmd_",[.9,.7,.9])
    #ctrl.SendCommand('SetCmdRobot '+cmd_ghost.GetName())

    print "Starting Simulation..."
    env.StartSimulation(timestep,True)
