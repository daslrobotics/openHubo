#!/usr/bin/env python
from numpy import pi,array,ones,zeros
import openhubo as oh
import time

(env,options)=oh.setup('qtcoin')
env.SetDebugLevel(3)
options.scenefile='gripper.env.xml'
options.robotfile=None
[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
rod=env.GetKinBody('rod')
trans=rod.GetTransform()
pose=oh.Pose(robot)
#pose[ind('rightIndexKnuckle1')]=.6
#pose[ind('rightMiddleKnuckle1')]=.6
#pose[ind('rightRingKnuckle1')]=.6
#pose[ind('rightPinkyKnuckle1')]=.6
#pose[ind('rightThumbKnuckle1')]=.6
success=False
strength=1.0
oh.set_robot_color(robot,[.7,.7,.7],[.7,.7,.7],0.0)
oh.set_finger_torquemode(robot)

#recorder.realtime=False
#recorder.filename='griptest.avi'
#recorder.start()

def apply_torque(pose,mask,T):
    """Use Pose class regex function to easily (but slowly) assign torques to a slice of joints"""
    pose.useregex=True
    pose[mask]=T

while not success:
    strength+=.2
    print "Added torque {}".format(strength)
    rod.SetVelocity(zeros(3),zeros(3))
    rod.SetTransform(trans)

    print "Mass is {}".format(rod.GetLinks()[0].GetMass())
    env.GetPhysicsEngine().SetGravity([0, 0, 0])
    ctrl.SetDesired(pose)
    print "starting..."
    #Hack to run simulation for a set amount of time
    for x in range(3000):
        with env:
            env.StepSimulation(oh.TIMESTEP)
    print "gravity on"
    env.GetPhysicsEngine().SetGravity([0, 0, -9.8])
    #Hack to get grasp torques
    apply_torque(pose,'right.*1',.5*strength)
    apply_torque(pose,'right.*2',.25*strength)
    apply_torque(pose,'right.*3',.125*strength)
    pose['rightThumbKnuckle1']*=2
    pose['rightThumbKnuckle2']*=2
    pose['rightThumbKnuckle3']*=2
    pose.send()

    env.StartSimulation(oh.TIMESTEP)
    t=env.GetSimulationTime()
    success=True
    while env.GetSimulationTime()<(t+1000000):
        if env.GetKinBody('rod').GetTransform()[2,3]<-.5:
            print "Failed, rod fell"
            success=False
            break
        time.sleep(.5)
#recorder.stop()
