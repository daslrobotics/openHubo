#!/usr/bin/env python

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

    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.01

    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,None,True,False)
    ctrl=RaveCreateController(env,'achcontroller')
    robot.SetController(ctrl)

    print "Starting Simulation..."
    env.StartSimulation(timestep,True)
