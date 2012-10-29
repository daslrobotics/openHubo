#!/usr/bin/env python
from openravepy import *
from servo import *
from numpy import pi

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
        return robot.GetJoint(name).GetDOFIndex()
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
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'trajectorycontroller')
        robot.SetController(controller)

        controller.SendCommand('set gains 50 0 8')

        #Set an initial pose before the simulation starts
        pose=array(zeros(robot.GetDOF()))

        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        #Set initial pose to avoid thumb collisions
        robot.SetDOFValues(pose)
        controller.SetDesired(pose)
    return robot

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

if __name__=='__main__':
    env=Environment()
    env.SetViewer('qtcoin')
    robot=load_simplefloor(env)
    env.StartSimulation(timestep=0.0005)


