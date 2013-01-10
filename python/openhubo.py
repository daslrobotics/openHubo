#!/usr/bin/env python
from numpy import pi,array
import openravepy as rave
from TransformMatrix import *

""" A collection of useful functions to run openhubo models.
As common functions are developed, they will be added here.
"""
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

def load_huboplus(env):
    pass

def load_hubo2(env):
    pass

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

def load(env,robotname,scenename=None,stop=False):
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
        env.Load(robotname)
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
    com_trans=array([0,0,0])
    mass=0
    for l in robot.GetLinks():
        com_trans=com_trans+l.GetTransform()[:-1,3]*l.GetMass()
        mass=mass+l.GetMass()

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
    proj_com[-1]=0

    env=robot.GetEnv()
    handle=env.plot3(points=proj_com,pointsize=12,colors=array([0,1,1]))
    return handle
    
def plotBodyCOM(env,link,handle=None,color=array([0,1,0])):
    origin=link.GetGlobalCOM()
    if handle==None:
        handle=env.plot3(points=origin,pointsize=10.0,colors=color)
    else:
        neworigin=[1,0,0,0]
        neworigin.extend(origin.tolist())
        handle.SetTransform(matrixFromPose(neworigin))
    return handle

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


