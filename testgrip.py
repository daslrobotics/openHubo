#!/usr/bin/env python
from numpy import pi,array
import openravepy as rave
from TransformMatrix import *
import time
from recorder import viewerrecorder
import datetime
import openhubo
import kbhit
from iuvalidate import *
import openhubo

def get_timestamp(lead='_'):
    return lead+datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")



def load(env,scenename=None,stop=False,physics=True):
    """ Load a robot model into the given environment, configuring a
    trajectorycontroller and a reference robot to show desired movements vs. actual
    pose. The returned tuple contains:
        robot: handle to the created robot
        controller: either trajectorycontroller or idealcontroller depending on physics
        name-to-joint-index converter
        ref_robot: handle to visiualization "ghost" robot
        recorder: video recorder python class for quick video dumps
    """

    # Set the robot controller and start the simulation
    recorder=viewerrecorder(env)
    #Default to "sim-timed video" i.e. plays back much faster
    recorder.videoparams[0:2]=[1024,768]
    recorder.realtime=False

    with env:
        if stop:
            env.StopSimulation()

        if type(scenename) is list:
            for n in scenename:
                loaded=env.Load(n)
        elif type(scenename) is str:
            loaded=env.Load(scenename)

    time.sleep(1)
    #Explicitly disable physics if option is selected
    with env:
        if not physics:
            env.SetPhysicsEngine(rave.RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        robot = env.GetRobots()[0]
        pose=ones(robot.GetDOF())*.6
        robot.SetDOFValues(pose)
        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = rave.RaveCreateCollisionChecker(env,'ode')
            print 'Note: Using ODE collision checker since PQP is not available'
        env.SetCollisionChecker(collisionChecker)

        if env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine' and physics:
            controller=rave.RaveCreateController(env,'servocontroller')
            robot.SetController(controller)
            controller.SendCommand('set gains 100 0 0')
            controller.SetDesired(pose/pose*pi/4*1.1)
    
    time.sleep(.5)
    ind=openhubo.makeNameToIndexConverter(robot)

    return (robot,controller,ind,recorder)

if __name__=='__main__':
    from openravepy import *
    from servo import *
    env=Environment()
    env.SetDebugLevel(3)
    env.Add(RaveCreateViewer(env,'qtcoin'))
    [robot,ctrl,ind,recorder]=load(env,'gripper.env.xml',True,True)
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
    recorder=viewerrecorder(env)
    #Default to "sim-timed video" i.e. plays back much faster
    recorder.videoparams[0:2]=[1024,768]
    recorder.realtime=False
    recorder.filename='griptest.avi'
    recorder.start()
    steps=int(10*1/0.0005)
    while fail:
        level=3
        strength+=.2
        print "Added torque {}".format(strength)
        robot.SetDOFValues(pose)
        rod.SetLinkVelocities((zeros(6),zeros(6))) 
        rod.SetTransform(trans)
        T=0.5
        set_finger_torque(robot,T)
        
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
            add_torque(robot,right_joints,strength,level)
        print "gravity on"
        env.GetPhysicsEngine().SetGravity([0, 0, -9.8])
        for x in range(steps):
            env.StepSimulation(0.0005)
            add_torque(robot,right_joints,strength,level)
            if env.GetKinBody('rod').GetTransform()[2,3]<-.5:
                print "Rod fell"
                break
        if (steps-x)<10:
            print "Success!"
            fail=False
    recorder.stop()
