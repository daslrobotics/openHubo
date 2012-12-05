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
import tab
from generalik import *
from cbirrt import *
import openhubo
from openhubo import pause
#dump of planning functions
from planning import *

if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)

    [robot,controller,ind]=openhubo.load_simplefloor(env)
    
    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,'hubo')

    first_pose=Cbirrt(probs_cbirrt)
    setInitialPose(robot)
   
    ## Create an example goal pose (The result of all of these steps should be a
    # goal pose vector)
    pose=robot.GetDOFValues()

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
    RunTrajectoryFromFile(robot,first_pose,False,True,False)

    pose1=robot.GetDOFValues()

    second_pose=Cbirrt(probs_cbirrt)
    
    ## Define a new goal pose
    pose1[ind('RSP')]=0

    ## This time, use the whole body (or at least, all 4 manipulators)
    activedofs=first_pose.ActivateManipsByIndex(robot,[0,1,2,3])

    # Use 
    TSR_left=TSR(robot.GetLink('leftFoot').GetTransform())
    TSR_left.manipindex=2

    TSR_right=TSR(robot.GetLink('rightFoot').GetTransform())
    TSR_right.manipindex=3

    chain_left=TSRChain(0,0,1).InsertTSR(TSR_left)
    chain_right=TSRChain(0,0,1).InsertTSR(TSR_right)

    second_pose.insertTSRChain(chain_left)
    second_pose.insertTSRChain(chain_right)

    chain.run()



