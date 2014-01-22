#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openhubo as oh
from openhubo import comps
from openhubo.comps import TSR
from openhubo import planning
import re
from openravepy import RaveCreateProblem
from numpy import mat, array, pi, cos, sin
from openhubo import mapping
from openhubo import cws
import numpy as np

def set_finger_torque_limits(robot,limit):
    fingers = mapping.get_fingers(robot)
    for f in fingers:
        f.SetTorqueLimits([limit])

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
                h=0.10 + .3 * (j+1)
                grips.append(T * comps.Transform(None,[0,0,h]))
        elif re.search('rung',link.GetName()):
            grips.append(T * comps.Transform(None,[0,.12,0]))
            grips.append(T * comps.Transform(None,[0,-.12,0]))
    #backwards compatible
    return grips


(env,options)=oh.setup('qtcoin')
env.SetDebugLevel(4)

# Load the environment

options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'
options.scenefile='../scenes/ladderclimb.env.xml'
[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
pose=oh.Pose(robot,ctrl)

col = robot.GetEnv().GetCollisionChecker()
colopts = col.GetCollisionOptions()
#Use simple contact
col.SetCollisionOptions(0)

T=robot.GetTransform()

T[0,3]+=(np.random.rand(1) -.5) * .10
T[2,3]+=(np.random.rand(1) -.5) * .05
robot.SetTransform(T)

print "Adjusted start pose is:"
print T[0:3,3]

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
z1=.01
theta=pi/16
LH=0
RH=8
POST=8
RUNG0=16
RUNG=2
LF=0
RF=1

#Post grips at shoulder height
rphi = -pi/3
lphi = -pi/3

r_radial_vec = array([sin(rphi), -cos(rphi),0])*.026
l_radial_vec = array([sin(lphi), cos(lphi),0])*.026
rgrip1=TSR(grips[3+RH],comps.Transform(None,r_radial_vec).tm,mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, rphi,rphi]),1)
lgrip1=TSR(grips[3+LH],comps.Transform(None,l_radial_vec).tm,mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -lphi,-lphi]),0)

#Post grips at shoulder height
phi = -pi/6

radial_vec = array([sin(phi), 0, cos(phi)])*.024

rrung=TSR(grips[RUNG0+RF],comps.Transform(None,radial_vec).tm,mat([-.00,-.00, 0,0, 0,0, 0,0 , phi-theta,phi+theta,0,0]),3)
lrung=TSR(grips[RUNG0+LF],comps.Transform(None,radial_vec).tm,mat([-.00,-.00, 0,0, 0,0, 0,0 , phi-theta,phi+theta,0,0]),2)

# Define keyframe poses in terms of manips
#pose1={'rightArm':rgrip1,'leftArm':lgrip1,'leftFoot':lrung,'rightFoot':rrung}
#pose1={'rightArm':rgrip1,'leftArm':lgrip1, 'rightFoot':rrung}
#pose1={'rightFoot':rrung}
pose1={'rightArm':rgrip1,'leftArm':lgrip1,'leftFoot':lrung,'rightFoot':rrung}

res,ik = planning.solveWholeBodyPose(robot,probs_cbirrt,pose1,10)

planning.close_fingers(robot)
LAP=robot.GetJoint('LAP')
RAP=robot.GetJoint('RAP')
planning.bisect_close(robot,LAP,LAP.GetValue(0)+.05)
planning.bisect_close(robot,RAP,RAP.GetValue(0)+.05)

pose = oh.Pose(robot)
pose.update()
pose.send()

env.StartSimulation(oh.TIMESTEP)

#Strong joints = stable posture

#1) run CWS check with good contacts using infinite strengths, and bounded strengths
#cws.
#2) relax finger strength and show failure / success
oh.pause(2)
col.SetCollisionOptions(colopts)

m = oh.find_mass(robot)
g = 9.8

leg_force_scale = 2
arm_force_scale = .5

finger_force_nominal = 1 * g
print "Scaling finger force to test"
env.StopSimulation()
pose.useregex = True
pose['LF*']-=.1
pose['RF*']-=.1

env.StartSimulation(oh.TIMESTEP)

oh.pause(2)
env.StopSimulation()

lwr = robot.GetJoint('LWR')
rwr = robot.GetJoint('RWR')

physics = env.GetPhysicsEngine()

l_force = []
for k in range(200):
    oh.pause(0.01)
    [force,torque] = physics.GetJointForceTorque(lwr)
    l_force.append(force)

out_force=array([0,0,0])
for k in range(99):
    out_force+=array(l_force[k])

out_force/=100
print out_force

nominal_force = 12*g / 3.0
f_length = 0.038
min_finger_torque = abs(out_force[0]) / 3.0  * f_length
finger_torque = nominal_force * f_length
set_finger_torque_limits( robot, finger_torque)

check=cws.ContactCheck(robot,.4)
finger_force_nominal = out_force[0] / 3.0
check.insert_link('leftFoot', m * g * leg_force_scale)
check.insert_link('rightFoot', m * g * leg_force_scale)
check.insert_link('leftPalm', m * g *  arm_force_scale)
check.insert_link('rightPalm', m * g *  arm_force_scale)
check.insert_link('Body_LF12', finger_force_nominal)
check.insert_link('Body_LF22', finger_force_nominal)
check.insert_link('Body_LF32', finger_force_nominal)
check.insert_link('Body_LF13', finger_force_nominal)
check.insert_link('Body_LF23', finger_force_nominal)
check.insert_link('Body_LF33', finger_force_nominal)
check.insert_link('Body_RF12', finger_force_nominal)
check.insert_link('Body_RF22', finger_force_nominal)
check.insert_link('Body_RF32', finger_force_nominal)
check.insert_link('Body_RF42', finger_force_nominal)
check.insert_link('Body_RF13', finger_force_nominal)
check.insert_link('Body_RF23', finger_force_nominal)
check.insert_link('Body_RF33', finger_force_nominal)
check.insert_link('Body_RF43', finger_force_nominal)

check.build_cws()
print check.check()

for l in range(5):
    check.forcelimits['leftFoot']+=m*g*2
    check.forcelimits['rightFoot']+=m*g*2
    check.build_cws()
    print check.check()
    print check.forcelimits['leftFoot']


check.forcelimits['leftFoot']=m*g*leg_force_scale
check.forcelimits['rightFoot']=m*g*leg_force_scale
check.forcelimits['Body_LF12']+=(finger_force_nominal*3)
check.forcelimits['Body_LF13']+=(finger_force_nominal*3)
check.forcelimits['Body_LF22']+=(finger_force_nominal*3)
check.forcelimits['Body_LF23']+=(finger_force_nominal*3)
check.forcelimits['Body_LF32']+=(finger_force_nominal*3)
check.forcelimits['Body_LF33']+=(finger_force_nominal*3)
check.forcelimits['Body_RF12']+=(finger_force_nominal*3)
check.forcelimits['Body_RF13']+=(finger_force_nominal*3)
check.forcelimits['Body_RF22']+=(finger_force_nominal*3)
check.forcelimits['Body_RF23']+=(finger_force_nominal*3)
check.forcelimits['Body_RF32']+=(finger_force_nominal*3)
check.forcelimits['Body_RF33']+=(finger_force_nominal*3)
check.build_cws()
print check.check()
print check.forcelimits['Body_LF12']


check.forcelimits['leftFoot']=m*g*2
check.forcelimits['rightFoot']=m*g*2
check.forcelimits['Body_LF12']+=(finger_force_nominal*6)
check.forcelimits['Body_LF13']+=(finger_force_nominal*6)
check.forcelimits['Body_LF22']+=(finger_force_nominal*6)
check.forcelimits['Body_LF23']+=(finger_force_nominal*6)
check.forcelimits['Body_LF32']+=(finger_force_nominal*6)
check.forcelimits['Body_LF33']+=(finger_force_nominal*6)
check.forcelimits['Body_RF12']+=(finger_force_nominal*6)
check.forcelimits['Body_RF13']+=(finger_force_nominal*6)
check.forcelimits['Body_RF22']+=(finger_force_nominal*6)
check.forcelimits['Body_RF23']+=(finger_force_nominal*6)
check.forcelimits['Body_RF32']+=(finger_force_nominal*6)
check.forcelimits['Body_RF33']+=(finger_force_nominal*6)
check.build_cws()
print check.check()
print check.forcelimits['Body_LF12']

oh.plot_contacts(robot)

check.forcelimits['leftFoot']=m*g*4
check.forcelimits['rightFoot']=m*g*4
check.forcelimits['Body_LF12']+=(finger_force_nominal*12)
check.forcelimits['Body_LF13']+=(finger_force_nominal*12)
check.forcelimits['Body_LF22']+=(finger_force_nominal*12)
check.forcelimits['Body_LF23']+=(finger_force_nominal*12)
check.forcelimits['Body_LF32']+=(finger_force_nominal*12)
check.forcelimits['Body_LF33']+=(finger_force_nominal*12)
check.forcelimits['Body_RF12']+=(finger_force_nominal*12)
check.forcelimits['Body_RF13']+=(finger_force_nominal*12)
check.forcelimits['Body_RF22']+=(finger_force_nominal*12)
check.forcelimits['Body_RF23']+=(finger_force_nominal*12)
check.forcelimits['Body_RF32']+=(finger_force_nominal*12)
check.forcelimits['Body_RF33']+=(finger_force_nominal*12)
check.build_cws()
print check.check()
print check.forcelimits['Body_LF12']
