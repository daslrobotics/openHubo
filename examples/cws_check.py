#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openhubo as oh
from openhubo import comps
from openhubo.comps import TSR,TSRChain
from openhubo import planning
import re
from openravepy import RaveCreateProblem
from numpy import mat,array
from openhubo import pause,mapping, cws

def close_left_hand(robot, angle):
    fingers = mapping.get_left_fingers(robot)
    for f in fingers:
        robot.SetDOFValues([angle],[f.GetDOFIndex()])

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

def setLeanPose(robot):
    #Define a manual initial pose for IK solution
    robot.SetActiveDOFs(robot.GetManipulator('leftFoot').GetArmIndices())
    robot.SetActiveDOFValues([0,0,0,.6,-.6,0])
    robot.SetActiveDOFs(robot.GetManipulator('rightFoot').GetArmIndices())
    robot.SetActiveDOFValues([0,0,0,.6,-.6,0])
    robot.SetActiveDOFs(robot.GetManipulator('leftArm').GetArmIndices())
    robot.SetActiveDOFValues([0,0,0,-pi/2,0,0,0])
    robot.SetActiveDOFs(robot.GetManipulator('rightArm').GetArmIndices())
    robot.SetActiveDOFValues([0,0,0,-pi/2,0,0,0])
    #Hack for DRCHubo
    fingers = mapping.get_fingers(robot);
    for f in fingers:
        robot.SetDOFValues([-1],[f.GetDOFIndex()])

def doIK(robot):
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

    #Define manips used and goals
    z1=.1
    theta=0.5
    LH=0
    RH=8
    POST=8
    RUNG0=16
    RUNG=2
    LF=0
    RF=1

    #Post grips at shoulder height
    rgrip1=TSR(grips[3 + RH],comps.Transform(None,[.0,-.02,0]).tm,mat([0,0, 0, 0,0, z1, 0,0 ,0,0, -theta,theta]),1)
    lgrip1=TSR(grips[3],comps.Transform(None,[.0,.02,0]).tm,mat([0,0, 0,0, 0, z1, 0,0 ,0,0, -theta,theta]),0)

    # Define keyframe poses in terms of manips
    pose1={'rightArm':rgrip1,'leftArm':lgrip1}

    print "Place the robot in the desired starting position"
    #TODO: Sample the Right foot floating base pose, passed in as a separate TSR chain?

    planning.solveWholeBodyPose(robot,probs_cbirrt,pose1,10)

(env,options)=oh.setup('qtcoin')
env.SetDebugLevel(3)

# Load the environment
[robot, ctrl, ind,ref,recorder]=oh.load_scene(env,options.robotfile,'ladderclimb.env.xml',True)
pose=oh.Pose(robot,ctrl)

print "Position the robot"
pause()

setLeanPose(robot)
oh.sleep(1)


T=robot.GetTransform()
T[2,3]-=.0015
robot.SetTransform(T)
oh.plot_contacts(robot)
pause()

stable, hull, CWS, report = cws.perform_cws(robot,['leftFoot','rightFoot','leftPalm','rightPalm'])

