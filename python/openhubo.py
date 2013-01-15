#!/usr/bin/env python
from numpy import pi,array
import openravepy as rave
from TransformMatrix import *
import time
from  recorder import viewerrecorder
import datetime
""" A collection of useful functions to run openhubo models.
As common functions are developed, they will be added here.
"""

def set_robot_color(robot,dcolor=[.5,.5,.5],acolor=[.5,.5,.5],trans=0,links=[]):
    #Iterate over a robot's links and set color / transparency
    if not len(links):
        links=robot.GetLinks()
    for l in links:
        for g in l.GetGeometries():
            g.SetDiffuseColor(dcolor)
            g.SetAmbientColor(acolor)
            g.SetTransparency(trans)

def set_finger_torque(robot,maxT,dt=0.0005):
    names=[u'rightIndexKnuckle1', u'rightIndexKnuckle2', u'rightIndexKnuckle3', u'rightMiddleKnuckle1', u'rightMiddleKnuckle2', u'rightMiddleKnuckle3', u'rightRingKnuckle1', u'rightRingKnuckle2', u'rightRingKnuckle3', u'rightPinkyKnuckle1', u'rightPinkyKnuckle2', u'rightPinkyKnuckle3', u'rightThumbKnuckle1', u'rightThumbKnuckle2', u'rightThumbKnuckle3',u'leftIndexKnuckle1', u'leftIndexKnuckle2', u'leftIndexKnuckle3', u'leftMiddleKnuckle1', u'leftMiddleKnuckle2', u'leftMiddleKnuckle3', u'leftRingKnuckle1', u'leftRingKnuckle2', u'leftRingKnuckle3', u'leftPinkyKnuckle1', u'leftPinkyKnuckle2', u'leftPinkyKnuckle3', u'leftThumbKnuckle1', u'leftThumbKnuckle2', u'leftThumbKnuckle3']
    #Rough calculation for now, eventually get this from finger models
    #Figure out the maximum acceleration needed to produce enough dV per timestep to produce the maximum torque given the inertia of the body
    Iz0=0.000002
    maxA=maxT/Iz0
    maxV=maxA*dt

    for n in names:
        robot.GetJoint(n).SetTorqueLimits([maxT])
        robot.GetJoint(n).SetVelocityLimits([maxV])
        robot.GetJoint(n).SetVelocityLimits([maxA])
        i=robot.GetJoint(n).GetDOFIndex()
        #TODO: Figure out actual finger stiffness?
        robot.GetController().SendCommand('set gainvec {} 50.0 0.0 0.0 '.format(i))


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

def load_simplefloor(env):
    """ Load up and configure the simpleFloor environment for hacking with
    physics. Sets some useful defaults.
    """
    with env:
        #Since physics are defined within the XML file, stop simulation
        env.StopSimulation()
        env.Load('simpleFloor.env.xml')
        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=rave.RaveCreateController(env,'trajectorycontroller')
        robot.SetController(controller)

        controller.SendCommand('set gains 50 0 8')

        #Set an initial pose before the simulation starts
        pose=array(zeros(robot.GetDOF()))

        #Set initial pose to avoid thumb collisions
        robot.SetDOFValues(pose)
        controller.SetDesired(pose)
    return (robot,controller,ind)

def load(env,robotname,scenename=None,stop=False,physics=True):
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

        loaded=env.Load(robotname)
    time.sleep(1)
    #Explicitly disable physics if option is selected
    with env:
        if not physics:
            env.SetPhysicsEngine(rave.RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        robot = env.GetRobots()[0]
        robot.SetDOFValues(zeros(robot.GetDOF()))
        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = rave.RaveCreateCollisionChecker(env,'ode')
            print 'Note: Using ODE collision checker since PQP is not available'
        env.SetCollisionChecker(collisionChecker)

        if env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine' and physics:
            controller=rave.RaveCreateController(env,'trajectorycontroller')
            robot.SetController(controller)
            controller.SendCommand('set gains 50 0 8')
            #set_finger_torque(robot,4.0)

            #Load ref robot and colorize
            #TODO: Load the actual robot as a copy, then strip out extra junk there
            #ref_robot=None
            env.Load('rlhuboplus.ref.robot.xml')
            ref_robot=env.GetRobot('rlhuboplus_ref')
            ref_robot.Enable(False)
            ref_robot.SetController(rave.RaveCreateController(env,'mimiccontroller'))
            controller.SendCommand("set visrobot rlhuboplus_ref")
            set_robot_color(ref_robot,[.7,.7,.5],[.7,.7,.5],trans=.5)
        else:
            #Just load ideal controller if physics engine is not present
            controller=rave.RaveCreateController(env,'idealcontroller')
            ref_robot=None
            robot.SetController(controller)
    
    time.sleep(.5)
    ind=makeNameToIndexConverter(robot)

    return (robot,controller,ind,ref_robot,recorder)

def load_rlhuboplus(env,scenename=None,stop=False):
    """ Load the rlhuboplus model into the given environment, configuring a
    servocontroller and a reference robot to show desired movements vs. actual
    pose. The returned tuple contains the robots, controller, and a
    name-to-joint-index converter.
    """

    # Set the robot controller and start the simulation
    with env:
        if stop:
            env.StopSimulation()

        if not(scenename==None):
            env.Load(scenename)
        env.Load('rlhuboplus.robot.xml')
        robot = env.GetRobots()[0]
        robot.SetDOFValues(zeros(robot.GetDOF()))
        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = rave.RaveCreateCollisionChecker(env,'ode')
            print 'Note: Using ODE collision checker since PQP is not available'
        env.SetCollisionChecker(collisionChecker)

        if env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine':
            controller=rave.RaveCreateController(env,'servocontroller')
            controller.SendCommand('setgains 100 0 16')

            #Load ref robot and colorize
            env.Load('rlhuboplus.ref.robot.xml')
            ref_robot=env.GetRobot('rlhuboplus_ref')
            ref_robot.Enable(False)
            ref_robot.SetController(rave.RaveCreateController(env,'mimiccontroller'))
            controller.SendCommand("set visrobot rlhuboplus_ref")
            for l in ref_robot.GetLinks():
                for g in l.GetGeometries():
                    g.SetDiffuseColor([.8,.8,.5])
                    g.SetTransparency(.5)
        else:
            #Just load ideal controller if physics engine is not present
            controller=rave.RaveCreateController(env,'idealcontroller')
            ref_robot=None

        robot.SetController(controller)

        ind=makeNameToIndexConverter(robot)

    return (robot,controller,ind,ref_robot)

def hubo2_left_palm():
    R=mat([[-0.5000,    -0.5000,   0.7071],
        [0.5000,   0.5000,   0.7071],
        [-0.7071,   0.7071,         0]])

    t=mat([.009396,-.010145,-.022417]).T
    return MakeTransform(R,t)

def hubo2_right_palm():
    return MakeTransform(R_hubo2_right_palm(),t_hubo2_right_palm())

def R_hubo2_right_palm():
    return mat( [[-0.5000,    0.5000,    0.7071],
            [-0.5000,    0.5000,   -0.7071],
            [-0.7071,   -0.7071,         0]])

def t_hubo2_right_palm():
    return mat([.009396,.010145,-.022417]).T

def hubo2_left_foot():
    R=mat(eye(3))
    t=mat([-.040497,.005,-.104983]).T+mat([0.042765281437, -0.002531569047,0.063737248723]).T
    return MakeTransform(R,t)

def hubo2_right_foot():
    R=mat(eye(3))
    t=mat([-.040497,-.005,-.104983]).T+mat([0.042765281437, 0.002531569047,0.063737248723]).T
    return MakeTransform(R,t)

def find_com(robot):
    com_trans=array([0.0,0.0,0.0])
    mass=0.0
    for l in robot.GetLinks():
        com_trans+= (l.GetGlobalCOM()*l.GetMass())
        mass+=l.GetMass()

    com=com_trans/mass
    return com

def find_mass(robot):
    mass=0
    for l in robot.GetLinks():
        mass=mass+l.GetMass()

    return mass

def plotProjectedCOG(robot):

    proj_com=find_com(robot)
    #assume zero height floor for now
    proj_com[-1]=0.001

    env=robot.GetEnv()
    handle=env.plot3(points=proj_com,pointsize=12,colors=array([0,1,1]))
    return handle
    
def plotBodyCOM(env,link,handle=None,color=array([0,1,0])):
    origin=link.GetGlobalCOM()
    m=link.GetMass()
    if handle==None:
        handle=env.plot3(points=origin,pointsize=5.0*m,colors=color)
    else:
        neworigin=[1,0,0,0]
        neworigin.extend(origin.tolist())
        handle.SetTransform(matrixFromPose(neworigin))
    return handle

def plot_masses(robot,color=array([.8,.5,.3]),ccolor=[0,.8,.8]):
    handles=[]
    total=0
    for l in robot.GetLinks():
        origin=l.GetGlobalCOM()
        m=l.GetMass()
        total+=m
        #Area of box corresponds to mass
        handles.append(robot.GetEnv().plot3(origin,5.0*power(m,.5),array(color)))
    handles.append(robot.GetEnv().plot3(find_com(robot),5.0*power(total,.5),ccolor))
    return handles

def CloseLeftHand(robot,angle=pi/2):
    #assumes the robot is still, uses direct control
    #TODO: make this general, for now only works on rlhuboplus
    #TODO: use trajectory controller to close hands smoothly
    ctrl=robot.GetController()
    ctrl.SendCommand('set radians')
    fingers=['Index','Middle','Ring','Pinky','Thumb']

    prox=[robot.GetJoint('left{}Knuckle{}'.format(x,1)).GetDOFIndex() for x in fingers]
    med=[robot.GetJoint('left{}Knuckle{}'.format(x,2)).GetDOFIndex() for x in fingers]
    dist=[robot.GetJoint('left{}Knuckle{}'.format(x,3)).GetDOFIndex() for x in fingers]
    pose=robot.GetDOFValues()
    for k in prox:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)

    for k in med:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)

    for k in dist:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)

def CloseRightHand(robot,angle=pi/2):
    #assumes the robot is still, uses direct control
    #TODO: make this general, for now only works on rlhuboplus
    
    ctrl=robot.GetController()
    ctrl.SendCommand('set radians')
    fingers=['Index','Middle','Ring','Pinky','Thumb']

    prox=[robot.GetJoint('right{}Knuckle{}'.format(x,1)).GetDOFIndex() for x in fingers]
    med=[robot.GetJoint('right{}Knuckle{}'.format(x,2)).GetDOFIndex() for x in fingers]
    dist=[robot.GetJoint('right{}Knuckle{}'.format(x,3)).GetDOFIndex() for x in fingers]

    #TODO: Fix this "cheat" of waiting a fixed amount of real time
    pose=robot.GetDOFValues()
    for k in prox:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)

    for k in med:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)

    for k in dist:
        pose[k]=angle
    ctrl.SetDesired(pose)
    time.sleep(1)
    return True


if __name__=='__main__':
    from openravepy import *
    from servo import *
    env=Environment()
    env.SetViewer('qtcoin')
    robot=load_simplefloor(env)
    env.StartSimulation(timestep=0.0005)


