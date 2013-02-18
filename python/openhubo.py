#!/usr/bin/env python
from numpy import pi,array,deprecate
import openravepy as rave
from TransformMatrix import *
from recorder import viewerrecorder
import time
import datetime
import warnings
import sys

# Interactive script 
if hasattr(sys,'ps1') or sys.flags.interactive:
    print "Loading OpenHubo interactive tools..."
    import startup

""" A collection of useful functions to run openhubo models.
As common functions are developed, they will be added here.
"""
TIMESTEP=0.001

def set_robot_color(robot,dcolor=[.5,.5,.5],acolor=[.5,.5,.5],trans=0,links=[]):
    """Iterate over a robot's links and set color / transparency."""
    if not len(links):
        links=robot.GetLinks()
    for l in links:
        for g in l.GetGeometries():
            g.SetDiffuseColor(dcolor)
            g.SetAmbientColor(acolor)
            g.SetTransparency(trans)

def get_timestamp(lead='_'):
    """Return a simple formatted timestamp for creating files and such."""
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
    return load(env,None,'simpleFloor.env.xml',True)

def load(env,robotname,scenename=None,stop=False,physics='physics.xml',ghost=True):
    """ Load a robot model into the given environment, configuring a
    trajectorycontroller and a reference robot to show desired movements vs. actual
    pose. The returned tuple contains:
        :robot: handle to the created robot
        :controller: either trajectorycontroller or idealcontroller depending on physics
        name-to-joint-index converter
        :ref_robot: handle to visiualization "ghost" robot
        :recorder: video recorder python class for quick video dumps
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

        #This method ensures that the URI is preserved in the robot
        if robotname!=None:
            robot=env.ReadRobotURI(robotname)
            env.Add(robot)
        else:
            #TODO: fix this assumption, could cause trouble with multiple robots
            robot=env.GetRobots()[0]

        robot.SetDOFValues(zeros(robot.GetDOF()))

        ref_robot=None
        if physics and env.GetPhysicsEngine().GetXMLId()=='GenericPhysicsEngine':
            rave.raveLogInfo('Loading physics parameters from {}'.format(physics))
            env.Load(physics)
        elif not physics:
            env.SetPhysicsEngine(rave.RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))

        #Force new controller since it's easier
        if env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine':
            rave.raveLogInfo('Creating controller for physics simulation')
            controller=rave.RaveCreateController(env,'trajectorycontroller')
            robot.SetController(controller)
            #TODO: set gains elsewhere?
            controller.SendCommand('set gains 50 0 8')

            #Load ref robot and colorize
            if robotname:
                ref_robot=env.ReadRobotURI(robotname)
                ref_robot.SetName('ref_'+robot.GetName())
                ref_robot.Enable(False)
                env.Add(ref_robot)
                ref_robot.SetController(rave.RaveCreateController(env,'mimiccontroller'))
                controller.SendCommand("set visrobot "+ref_robot.GetName())
                set_robot_color(ref_robot,[.7,.7,.5],[.7,.7,.5],trans=.5)
        else:
            #Just load ideal controller if physics engine is not present
            controller=rave.RaveCreateController(env,'idealcontroller')
            robot.SetController(controller)

        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = rave.RaveCreateCollisionChecker(env,'ode')
            warnings.warn('Note: Using ODE collision checker since PQP is not available')
        env.SetCollisionChecker(collisionChecker)
    
    ind=makeNameToIndexConverter(robot)

    return (robot,controller,ind,ref_robot,recorder)

def load_rlhuboplus(env,scenename=None,stop=False):
    """ Load the rlhuboplus model into the given environment, configuring a
    servocontroller and a reference robot to show desired movements vs. actual
    pose. The returned tuple contains the robots, controller, and a
    name-to-joint-index converter.
    """
    return load(env,'rlhuboplus.robot.xml',scenename,stop)

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

############################################################
# Mass functions

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

def plot_projected_com(robot):
    proj_com=find_com(robot)
    #assume zero height floor for now
    proj_com[-1]=0.001

    env=robot.GetEnv()
    handle=env.plot3(points=proj_com,pointsize=12,colors=array([0,1,1]))
    return handle

def plotProjectedCOG(robot):
    return plot_projected_com(robot)
    
def plotBodyCOM(env,link,handle=None,color=array([0,1,0])):
    return plot_body_com(env,link,handle,color)

def plot_body_com(env,link,handle=None,color=array([0,1,0])):
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

import logging
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import atexit

def safe_quit(env):
    """ Exit callback to ensure that openrave closes safely."""
    #Somewhat overkill, try to avoid annoying segfaults
    rave.raveLogDebug("Safely exiting rave environment...")
    if env:
        env.Destroy()
    rave.RaveDestroy()


def setup(viewername=None,create=True):
    """ Setup openhubo environment and viewer when run from the command line.
    :param viewername: Name of viewer plugin to use (defaults to no viewer)
    :param create: If true, set up and return environment. Otherwise, parse and return options.
    """
    parser = OptionParser(description='OpenHubo: perform experiments with virtual hubo modules.',
                          usage='usage: %prog [options] script')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--robot', action="store",type='string',dest='robotfile',default='rlhuboplus.robot.xml',
                      help='Robot XML file to load (default=%default)')
    parser.add_option('--scene', action="store",type='string',dest='scenefile',default='floor.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--example', action="store",type='string',dest='example',default=None,
                      help='Run an example')
    parser.add_option('--nointeract', action="store_false",dest='interact',default=True,
                      help='Run an example')
    parser.add_option('--debug', action="store_true",dest='pydebug',default=False,
                      help='Run an example')
    (options, leftargs) = parser.parse_args()

    if viewername:
        #Overwrite command line option with explicit argument?
        options._viewer=viewername

    #TODO: Ignoring Mac compatibility since openhubo is linux only
    if create:
        env=rave.Environment()
        atexit.register(safe_quit,env)
        OpenRAVEGlobalArguments.parseEnvironment(options,env)
        return (env,options)
    else:
        return (options,leftargs[0])

if __name__ == '__main__':
    """Run openhubo to see example files and use the IPython shell for inspection and debugging."""
    (options,scriptname)=setup(None,False)

    if options.pydebug:
        import debug

    if options.example or scriptname:

        if scriptname:
            execfile(scriptname)
        else:
            import fnmatch
            import os
            from openravepy import raveLogInfo
            expath=os.environ['OPENHUBO_DIR'] + '/examples/'
            for f in os.listdir(expath):
                if fnmatch.fnmatch(f, options.example):
                    raveLogInfo("Found example {}".format(options.example))
                    break
            execfile(expath+options.example)

        if options.interact:
            var=raw_input('Would you like to drop into IPython to inspect variables? [y/N]?')
            if var=='y' or var=='Y' or var=='yes':
                import IPython
                IPython.embed() 
                print "Cleaning up after inspection..."
                env.Destroy()
