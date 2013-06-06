import numpy as _np
import re as _re
import openhubo as _oh
from openravepy import CollisionReport
from numpy import pi

FINGER_NAMES=('rightIndexKnuckle1', 'rightIndexKnuckle2',
                'rightIndexKnuckle3', 'rightMiddleKnuckle1',
                'rightMiddleKnuckle2', 'rightMiddleKnuckle3',
                'rightRingKnuckle1', 'rightRingKnuckle2',
                'rightRingKnuckle3', 'rightPinkyKnuckle1',
                'rightPinkyKnuckle2', 'rightPinkyKnuckle3',
                'rightThumbKnuckle1', 'rightThumbKnuckle2',
                'rightThumbKnuckle3','leftIndexKnuckle1',
                'leftIndexKnuckle2', 'leftIndexKnuckle3',
                'leftMiddleKnuckle1', 'leftMiddleKnuckle2',
                'leftMiddleKnuckle3', 'leftRingKnuckle1', 'leftRingKnuckle2',
                'leftRingKnuckle3', 'leftPinkyKnuckle1', 'leftPinkyKnuckle2',
                'leftPinkyKnuckle3', 'leftThumbKnuckle1',
                'leftThumbKnuckle2', 'leftThumbKnuckle3',
                'LF1','LF2','LF3','LF4','LF5',
                'RF1','RF2','RF3','RF4','RF5')

""" Some simple hand helper functions """
def open_huboplus_hand(robot,index=0,angle=.1):
    close_huboplus_hand(robot,index,angle)

def close_huboplus_hand(robot,index=0,angle=pi/2,weights=None):
    manip = robot.GetManipulators()[index]
    fingers=manip.GetChildDOFIndices()
    pose = _oh.Pose(robot)
    if weights is None:
        weights = _np.ones(len(fingers))
    for i,k in enumerate(fingers):
        pose[k]+=angle*weights[i]
    pose.send()

def close_finger(finger,offset,tol=.01):
    """Close a finger joint by up to a given amount, checking for collision.
    Note that this implementation is slow...

    """

    link=finger.GetHierarchyChildLink()
    #Make a pose for the robot
    pose=_oh.Pose(finger.GetParent())
    initial = pose[finger]
    env=finger.GetParent().GetEnv()

    report = CollisionReport()
    step=offset
    while abs(step) > tol:
        pose[finger] += step
        pose.send()
        collision=env.CheckCollision(link,report=report)
        if collision:
            pose[finger]-=step
            pose.send()
            step/=2.
        elif abs(pose[finger]-initial)>=offset:
            #Met or exceeded desired offset without collision
            break

    return offset

def get_finger_names(robot,left_right=False):
    """General function to extract finger joint names from a Hubo-type robot.
    Current implementation is simple, but this function may be expanded to
    handle different arrangements."""
    if not left_right:
        return [n for n in FINGER_NAMES if robot.GetJoint(n) is not None]
    else:
        ljoints=[n for n in FINGER_NAMES if _re.search(r'left|^L',n) and robot.GetJoint(n) is not None]
        rjoints=[n for n in FINGER_NAMES if _re.search(r'right|^R',n) and robot.GetJoint(n) is not None]
        return (ljoints,rjoints)

def get_finger_joints(robot,left_right=False):
    """General function to extract finger joint names from a Hubo-type robot.
    Current implementation is simple, but this function may be expanded to
    handle different arrangements. This is implemented the dumb way now, but may be swapped for a regex search in the future."""

    #Only return valid finger joints:
    if not left_right:
        return [robot.GetJoint(n) for n in FINGER_NAMES if robot.GetJoint(n) is not None]
    else:
        ljoints=[robot.GetJoint(n) for n in FINGER_NAMES if _re.search(r'left|^L',n) and robot.GetJoint(n) is not None]
        rjoints=[robot.GetJoint(n) for n in FINGER_NAMES if _re.search(r'right|^R',n) and robot.GetJoint(n) is not None]
        return (ljoints,rjoints)

def get_fingers_mask(joints,mask):
    return [j for j in joints if _re.search(mask,j.GetName())]

def set_finger_torquemode(robot,mode='directtorque'):
    """ Shortcut function to set fingers to open-loop torque mode, for heavy-duty grasping"""
    joints=get_finger_joints(robot)
    return _oh.set_servo_torquemode(robot,joints,mode)

