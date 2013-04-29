#!/usr/bin/env python
from numpy import pi,array,ones,zeros
import openhubo
import re

(env,options)=openhubo.setup('qtcoin')
env.SetDebugLevel(3)
options.scenefile='gripper.env.xml'
[robot,ctrl,ind,ref,recorder]=openhubo.load_scene(env,options)
rod=env.GetKinBody('rod')
trans=rod.GetTransform()
pose=ones(robot.GetDOF())*.8
#pose[ind('rightIndexKnuckle1')]=.6
#pose[ind('rightMiddleKnuckle1')]=.6
#pose[ind('rightRingKnuckle1')]=.6
#pose[ind('rightPinkyKnuckle1')]=.6
#pose[ind('rightThumbKnuckle1')]=.6
fail=True
strength=1.0
openhubo.set_robot_color(robot,[.7,.7,.7],[.7,.7,.7],0.0)

recorder.realtime=False
recorder.filename='griptest.avi'
#recorder.start()
steps=int(10*1/0.0005)
fingerjoints=[x for x in robot.GetJoints() if re.search('Knuckle',x.GetName())]
fingers=[x.GetName() for x in fingerjoints]
while fail:
    level=3
    strength+=.2
    print "Added torque {}".format(strength)
    robot.SetDOFValues(pose)
    rod.SetLinkVelocities((zeros(6),zeros(6)))
    rod.SetTransform(trans)
    T=0.5
    openhubo.set_finger_torque(robot,T)

    right_joints=[]
    for n in fingers:
        if n.find('right')>-1:
            right_joints.append(robot.GetJoint(n))

    print "Finger torque is {}".format(T)
    print "Mass is {}".format(rod.GetLinks()[0].GetMass())
    env.GetPhysicsEngine().SetGravity([0, 0, 0])
    ctrl.SetDesired(pose)
    print "starting..."
    for x in range(3000):
        with env:
            env.StepSimulation(0.0005)
        openhubo.add_torque(robot,right_joints,strength,level)
    print "gravity on"
    env.GetPhysicsEngine().SetGravity([0, 0, -9.8])
    for x in range(steps):
        env.StepSimulation(0.0005)
        openhubo.add_torque(robot,right_joints,strength,level)
        if env.GetKinBody('rod').GetTransform()[2,3]<-.5:
            print "Rod fell"
            break
    if (steps-x)<10:
        print "Success!"
        fail=False
#recorder.stop()
