#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

from openravepy import *
from numpy import *
from numpy.linalg import inv
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import datetime
import sys
import os
import openhubo
from generalik import *
from cbirrt import *
import openhubo
from openhubo import pause
#dump of planning functions
from planning import *

if __name__=='__main__':

    env = Environment()
    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(3)

    env.Load('physics.xml')
    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,options.scenefile,True)

    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,robot.GetName())

    first_pose=Cbirrt(probs_cbirrt)
    setInitialPose(robot)
   
    ## Create an example goal pose (The result of all of these steps should be a
    # goal pose vector)
    pose=robot.GetDOFValues()
    start_position=robot.GetTransform()

    pose[ind('RSP')]=-pi/4
    pose[ind('LSP')]=-pi/4

    #Choose degrees of freedom that are allowed to be moved to explore towards
    #the goal.
    activedofs=first_pose.ActivateManipsByIndex(robot,[0,1])
    activedofs.append(ind('LHP'))
    activedofs.append(ind('RHP'))

    #Set the goal pose as 
    for x in activedofs:
        first_pose.jointgoals.append(pose[x])
    #Add in hip pitches
    robot.SetActiveDOFs(activedofs) 

    first_pose.filename='firstpose.traj'
    print first_pose.Serialize()
   
    first_pose.run()
    openhubo.pause()
    RunTrajectoryFromFile(robot,first_pose,False)

    ## Now, reset to initial conditions and activate whole body
    env.StopSimulation()
    second_pose=Cbirrt(probs_cbirrt)
    setInitialPose(robot)
    robot.SetTransform(start_position) 

    ## This time, use the whole body (or at least, all 4 manipulators)
    activedofs=second_pose.ActivateManipsByIndex(robot,[0,1,2,3])

    # Use 
    TSR_left=TSR(robot.GetLink('leftFoot').GetTransform())
    TSR_left.manipindex=2

    TSR_right=TSR(robot.GetLink('rightFoot').GetTransform())
    TSR_right.manipindex=3

    chain_left=TSRChain(0,1,1)
    chain_left.insertTSR(TSR_left)
    chain_right=TSRChain(0,1,1)
    chain_right.insertTSR(TSR_right)

    second_pose.insertTSRChain(chain_left)
    second_pose.insertTSRChain(chain_right)
    second_pose.supportlinks=['leftFoot','rightFoot']
    second_pose.exactsupport=1
    second_pose.filename='secondpose.traj'
    second_pose.psample=.05

    print second_pose.Serialize()
    second_pose.run()
    RunTrajectoryFromFile(robot,second_pose,False)
