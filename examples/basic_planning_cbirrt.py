#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openhubo
import openhubo.planning as planning
import openhubo.comps as comps

from numpy import pi
from openhubo import pause
from openhubo.comps.TSR import TSR,TSRChain
from openravepy import RaveCreateProblem

(env,options)=openhubo.setup('qtcoin')
env.SetDebugLevel(3)

options.physicsfile='physics.xml'

[robot,ctrl,ind,ref,recorder]=openhubo.load_scene(env,options)

probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
env.LoadProblem(probs_cbirrt,robot.GetName())

first_pose=comps.Cbirrt(probs_cbirrt)
planning.setInitialPose(robot)

## Create an example goal pose (The result of all of these steps should be a
# goal pose vector)
pose=openhubo.Pose(robot,ctrl)
start_position=robot.GetTransform()

pose['RSP']=-pi/4
pose['LSP']=-pi/4

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
pause()
env.StartSimulation(openhubo.TIMESTEP)
planning.RunTrajectoryFromFile(robot,first_pose,False)

## Now, reset to initial conditions and activate whole body
env.StopSimulation()
second_pose=comps.Cbirrt(probs_cbirrt)
planning.setInitialPose(robot)
robot.SetTransform(start_position)

## This time, use the whole body (or at least, all 4 manipulators)
activedofs=second_pose.ActivateManipsByIndex(robot,[0,1,2,3])

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
env.StartSimulation(openhubo.TIMESTEP)
planning.RunTrajectoryFromFile(robot,second_pose,False)
