#!/usr/bin/env python

from openhubo import *
from openravepy import *
from numpy import *
from openhubo.trajectory import *

""" Simple test script to run some of the functions above. """

(env,options)=setup('qtcoin')
env.SetDebugLevel(4)

options.physicsfile='physics.xml'
options.ghost=True

[robot,ctrl,ind,ref,recorder]=load_scene(env,options)
env.StartSimulation(TIMESTEP)

#Initialize pose object and trajectory for robot
pose=Pose(robot,ctrl)
[traj,config]=create_trajectory(robot)

traj_append(traj,pose.to_waypt(0.01))

pose['LAP']=-pi/8
pose['RAP']=-pi/8

pose['LKP']=pi/4
pose['RKP']=pi/4

pose['LHP']=-pi/8
pose['RHP']=-pi/8

pose['LSP']=-pi/8
pose['LEP']=-pi/4

traj_append(traj,pose.to_waypt(1.0))

pose[:]=0.0

traj_append(traj,pose.to_waypt(1.0))

planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

print "Showing samples of knee pose at given times:"
for k in range(40):
    data=traj.Sample(float(k)/10)
    print data[ind('LKP')]

ctrl.SetPath(traj)
ctrl.SendCommand('start')
while not(ctrl.IsDone()):
    sleep(.1)
