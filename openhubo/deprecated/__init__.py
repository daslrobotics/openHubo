"""
Deprecated functions for openhubo package.

To use these older functions in an openhubo script:


The module has been specifically designed to be imported this way, so there won't be namespace pollution.
"""
import numpy as _np
import openravepy as _rave
import time as _time

def load_rlhuboplus(env,scenename=None,stop=False):
    """ Load the rlhuboplus model and a scene into openhubo.
    Returns a servocontroller and a reference robot to show desired
    movements vs. actual pose. The returned tuple contains the robots,
    controller, and a name-to-joint-index converter.
    """
    return load_scene(env,'rlhuboplus.robot.xml',scenename,stop)

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

def plotProjectedCOG(robot):
    return plot_projected_com(robot)

def plotBodyCOM(env,link,handle=None,color=_np.array([0,1,0])):
    return plot_body_com(link,handle,color)

def CloseLeftHand(robot,angle=_np.pi/2):
    #assumes the robot is still, uses direct control
    #TODO: make this general, for now only works on rlhuboplus
    #TODO: use trajectory controller to close hands smoothly
    ctrl=robot.GetController()
    ctrl.SendCommand('set radians ')
    fingers=['Index','Middle','Ring','Pinky','Thumb']

    prox=[robot.GetJoint(
        'left{}Knuckle{}'.format(x,1)).GetDOFIndex() for x in fingers]
    med=[robot.GetJoint(
        'left{}Knuckle{}'.format(x,2)).GetDOFIndex() for x in fingers]
    dist=[robot.GetJoint(
        'left{}Knuckle{}'.format(x,3)).GetDOFIndex() for x in fingers]
    pose=robot.GetDOFValues()

    for k in prox:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)

    for k in med:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)

    for k in dist:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)

def CloseRightHand(robot,angle=_np.pi/2):
    #assumes the robot is still, uses direct control
    #TODO: make this general, for now only works on rlhuboplus
    
    ctrl=robot.GetController()
    ctrl.SendCommand('set radians ')
    fingers=['Index','Middle','Ring','Pinky','Thumb']

    prox=[robot.GetJoint(
        'right{}Knuckle{}'.format(x,1)).GetDOFIndex() for x in fingers]
    med=[robot.GetJoint(
        'right{}Knuckle{}'.format(x,2)).GetDOFIndex() for x in fingers]
    dist=[robot.GetJoint(
        'right{}Knuckle{}'.format(x,3)).GetDOFIndex() for x in fingers]

    #TODO: Fix this "cheat" of waiting a fixed amount of real time
    pose=robot.GetDOFValues()
    for k in prox:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)

    for k in med:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)

    for k in dist:
        pose[k]=angle
    ctrl.SetDesired(pose)
    _time.sleep(1)
    return True

def makeNameToIndexConverter(robot,autotranslate=True):
    """ A closure to easily convert from a string joint name to the robot's
    actual DOF index. 
    
    Example usage:
        #create function for a robot
        pose=robot.GetDOFValues()
        ind = make_name_to_index_converter(robot)
        #Use the function to find an index in a vector of DOF values
        pose[ind('LHP')]=pi/4
        #This way you don't have to remember the DOF index of a joint to tweak it.

    NOTE: Deprecated 3/25/2013
    """
    return make_name_to_index_converter(robot,autotranslate)

def make_dof_value_map(robot):
    names = [j.GetName() for j in robot.GetJoints()]
    indices = [j.GetDOFIndex() for j in robot.GetJoints()]

    def get_dofs():
        values=robot.GetDOFValues()
        for (i,n) in zip(indices,names):
            pose.setdefault(n,values[i])

    return get_dofs

def load_simplefloor(env):
    """ Load up and configure the simpleFloor environment for hacking with
    physics. Sets some useful defaults.
    """
    return load_scene(env,None,'simpleFloor.env.xml',True)
