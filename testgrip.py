#!/usr/bin/env python
from numpy import pi,array
import openravepy as rave
from TransformMatrix import *
import time
from recorder import viewerrecorder
import datetime
import tab
import kbhit

fingers=[u'rightIndexKnuckle1', u'rightIndexKnuckle2', u'rightIndexKnuckle3', u'rightMiddleKnuckle1', u'rightMiddleKnuckle2', u'rightMiddleKnuckle3', u'rightRingKnuckle1', u'rightRingKnuckle2', u'rightRingKnuckle3', u'rightPinkyKnuckle1', u'rightPinkyKnuckle2', u'rightPinkyKnuckle3', u'rightThumbKnuckle1', u'rightThumbKnuckle2', u'rightThumbKnuckle3']

def set_finger_torque(robot,maxT,dt=0.0005):
    #Rough calculation for now, eventually get this from finger models
    Iz0=0.000002
    maxA=30
    maxV=3

    for n in fingers:
        robot.GetJoint(n).SetTorqueLimits([maxT])
        robot.GetJoint(n).SetVelocityLimits([maxV])
        robot.GetJoint(n).SetAccelerationLimits([maxA])
        i=robot.GetJoint(n).GetDOFIndex()
        #TODO: Figure out actual finger stiffness?
        robot.GetController().SendCommand('set gainvec {} 100.0 0.0 0.0 '.format(i))

def get_timestamp(lead='_'):
    return lead+datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

def pause(t=-1):
    """ A simple pause function to emulate matlab's pause(t). 
    Useful for debugging and program stepping"""
    if t==-1:
        raw_input('Press any key to continue...')
    elif t>=0:
        time.sleep(t)
        
def makeNameToIndexConverter(robot):
    """ A closure to easily convert from a string joint name to the robot's
    actual DOF index, for use in creating/editing trajectories."""
    def convert(name):
        j=robot.GetJoint(name)
        if not(j==None):
            return robot.GetJoint(name).GetDOFIndex()
        else:
            return -1
    return convert


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
    ind=makeNameToIndexConverter(robot)

    return (robot,controller,ind,recorder)

def add_torque(robot,joints,maxT,level=3):
    #TODO: individual joint control?
    if level>0:
        for j in joints[0::3][:-1]:
            j.AddTorque([maxT*.5])
        joints[-3].AddTorque([maxT*1.0])

    if level>1:
        for j in joints[1::3][:-1]:
            j.AddTorque([maxT*.25])
        joints[-2].AddTorque([maxT*.5])
    if level>2:
        for j in joints[2::3][:-1]:
            j.AddTorque([maxT*.125])
        joints[-1].AddTorque([maxT*.25])

if __name__=='__main__':
    from openravepy import *
    from servo import *
    env=Environment()
    env.SetDebugLevel(4)
    env.SetViewer('qtcoin')
    [robot,ctrl,ind,recorder]=load(env,'gripper.env.xml',True,True)
    rod=env.GetKinBody('rod')
    trans=rod.GetTransform()
    pose=ones(robot.GetDOF())*.4
    pose[ind('rightIndexKnuckle1')]=.6
    pose[ind('rightMiddleKnuckle1')]=.6
    pose[ind('rightRingKnuckle1')]=.6
    pose[ind('rightPinkyKnuckle1')]=.6
    pose[ind('rightThumbKnuckle1')]=.6
    fail=False

    robot.SetDOFValues(pose)
    rod.SetLinkVelocities((zeros(6),zeros(6))) 
    rod.SetTransform(trans)
    T=.1
    set_finger_torque(robot,T)
    
    right_joints=[]
    for n in fingers:
        if n.find('right')>-1:
            right_joints.append(robot.GetJoint(n))

    ctrl.SetDesired(pose*1.5)
    print "Finger torque is {}".format(T)
    print "Mass is {}".format(rod.GetLinks()[0].GetMass())
    env.GetPhysicsEngine().SetGravity([0, 0, 0])
    env.StartSimulation(timestep=0.0005)
    print "starting..."
    time.sleep(.5)
    print "gravity on"
    env.StopSimulation()
    env.GetPhysicsEngine().SetGravity([0, 0, -9.8])
    level=3
    strength=1.6
    while True:
        env.StepSimulation(0.0005)
        add_torque(robot,right_joints,strength,level)
        if env.GetKinBody('rod').GetTransform()[2,3]<-.5:
            print "Rod fell"
            fail=True
            break
    env.StopSimulation()

