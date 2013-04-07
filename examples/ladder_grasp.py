#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openhubo as oh
from openhubo import comps
from openhubo.comps.TSR import TSR,TSRChain
from openhubo import planning
import re
from openravepy import RaveCreateProblem
from numpy import mat,array
from openhubo import pause
from openhubo.deprecated import CloseLeftHand,CloseRightHand

def makeGripTransforms(links):
    """ Make pre-defined grip locations based on a pre-inspection of the ladder"""
    grips = []
    for k,link in enumerate(links):
        T = comps.Transform(link.GetTransform())
        print T

        if re.search('post',link.GetName()):
            for j in range(8):
                #create one grip above each rung
                h=0.07+.3*j
                grips.append(T * comps.Transform(None,[0,0,h]))
        elif re.search('rung',link.GetName()):
            grips.append(T * comps.Transform(None,[0,.12,0]))
            grips.append(T * comps.Transform(None,[0,-.12,0]))
    #backwards compatible
    return grips

if __name__=='__main__':

    (env,options)=oh.setup('qtcoin')
    env.SetDebugLevel(3)

    # Load the environment
    [robot, ctrl, ind,ref,recorder]=oh.load_scene(env,options.robotfile,'ladderclimb.env.xml',True)
    pose=oh.Pose(robot,ctrl)

    print "Position the robot"
    pause()
    stairs=env.GetKinBody('ladder')
    links=stairs.GetLinks()

    #Make any adjustments to initial pose here
    handles=[]
    for k in links:
        handles.append(oh.plot_body_com(k))

    grips = makeGripTransforms(links)
    griphandles=planning.plotTransforms(env,grips,array([0,0,1]))

    # make a list of Link transformations

    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')

    env.LoadProblem(probs_cbirrt,robot.GetName())

    planning.setInitialPose(robot)
    oh.sleep(1)

    #Define manips used and goals
    z1=.05
    theta=0.5
    LH=0
    RH=8
    POST=8
    RUNG0=16
    RUNG=2
    LF=0
    RF=1

    #Post grips at shoulder height
    rgrip1=TSR(grips[3+RH],comps.Transform(None,[.0,-.015,0]).tm,mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),1)
    lgrip1=TSR(grips[3],comps.Transform(None,[.0,.015,0]).tm,mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),0)

    # Define keyframe poses in terms of manips
    pose1={'rightArm':rgrip1,'leftArm':lgrip1}

    print "Place the robot in the desired starting position"
    #TODO: Sample the Right foot floating base pose, passed in as a separate TSR chain?

    #solveWholeBodyPose(robot,probs_cbirrt,pose1)

    first_pose=comps.Cbirrt(probs_cbirrt)

    chains=[TSRChain(0,1,0) for x in range(3)]
    chains[0].insertTSR(lgrip1)
    chains[1].insertTSR(rgrip1)
    chains[2]=planning.MakeInPlaceConstraint(robot,'leftFoot')

    for c in chains:
        first_pose.insertTSRChain(c)

    #Activate useful DOFs
    activedofs=first_pose.ActivateManipsByIndex(robot,[0,1])
    #Add in hip pitches
    activedofs.append(ind('LHP'))
    activedofs.append(ind('RHP'))
    robot.SetActiveDOFs(activedofs)

    first_pose.supportlinks=['leftFoot','rightFoot']
    first_pose.filename='firstpose.traj'
    print first_pose.Serialize()
    success=first_pose.run()
    pause()

    env.StartSimulation(oh.TIMESTEP)
    planning.RunTrajectoryFromFile(robot,first_pose)

    CloseLeftHand(robot,1.05)
    CloseRightHand(robot,1.05)

