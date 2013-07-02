import numpy as np
import re as _re
from numpy.random import rand
#KLUDGE: hard code the mapping (how often will it change, really?). Include openhubo synonyms here for fast lookup.

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

ha_ind_name_map={'RHY':26,
                   'RHR':27,
                   'RHP':28,
                   'RKN':29,
                   'RAP':30,
                   'RAR':31,
                   'LHY':19,
                   'LHR':20,
                   'LHP':21,
                   'LKN':22,
                   'LAP':23,
                   'LAR':24,
                   'RSP':11,
                   'RSR':12,
                   'RSY':13,
                   'REB':14,
                   'RWY':15,
                   'RWR':16,
                   'RWP':17,
                   'LSP':4,
                   'LSR':5,
                   'LSY':6,
                   'LEB':7,
                   'LWY':8,
                   'LWR':9,
                   'LWP':10,
                   'NKY':1,
                   'NK1':2,
                   'NK2':3,
                   'WST':0,
                   'RF1':32,
                   'RF2':33,
                   'RF3':34,
                   'RF4':35,
                   'RF5':36,
                   'LF1':37,
                   'LF2':38,
                   'LF3':39,
                   'LF4':40,
                   'LF5':41}

#Hack to get global synonyms
synonyms={'LKP':'LKN',
          'RKP':'RKN',
          'REP':'REB',
          'LEP':'LEB',
          'TSY':'WST'}

for s in ['left','right']:
    for i,f in enumerate(['Index','Middle','Ring','Pinky','Thumb']):
        synonyms[s+f+'Knuckle1']='{}F{}'.format(s[0].upper(),str(i+1))


inv_synonyms={v:k for (k, v) in synonyms.iteritems()}

oh_from_ha_names={k:(inv_synonyms[k] if k in inv_synonyms.keys() else k) for k in ha_ind_name_map.keys()}

deprecated_names={'HPY':'WST',
                  'HDY':'NKY',
                  'HDP':'NK2',
                  'HNR':'NK2',
                  'HDR':'NK1',
                  'HNR':'NK1',
                  'HNP':'NK2'}

def ha_from_oh(inname):
    """ Get the hubo-ach name of a joint from the openhubo name"""
    if ha_ind_name_map.has_key(inname):
        return inname
    elif synonyms.has_key(inname):
        return synonyms[inname]
    elif deprecated_names.has_key(inname):
        return deprecated_names[inname]
    else:
        return None

def oh_from_ha(inname):
    """ Get the openhubo name of a joint from the hubo-ach name"""
    if ha_ind_name_map.has_key(inname):
        return inname
    elif inv_synonyms.has_key(inname):
        return inv_synonyms[inname]
    else:
        return None

def slow_lookup_joint(inname):
    """Pass a name through all the mapping functions to brute force a match"""
    return oh_from_ha(ha_from_oh(inname))

def get_name_from_huboname(inname,robot=None):
    """ Map a name from the openhubo standard to the original hubo naming
    scheme.
    """
    return oh_from_ha(inname)

def get_huboname_from_name(inname):
    """Get a hubo-standard joint name from the openhubo name (mostly the same,
    but they differ slightly for some joints.
    """
    return ha_from_oh(inname)

def ha_ind_from_oh_ind(robot):
    """Make a direct ind map between a robot and the hubo-ach interface"""
    return {j.GetDOFIndex():(ha_ind_name_map[ha_from_oh(j.GetName())] if ha_from_oh(j.GetName()) else None) for j in robot.GetJoints()}

def create_random_bounded_pose(robot):
    """Create a random valid pose for the robot (i.e. within joint limits).
    This is NOT gauranteed to be stable, so avoid using this on a live
    robot!!!"""
    (lower,upper)=robot.GetDOFLimits()
    motion=upper-lower
    center=(upper+lower)/2.0
    vals=center+motion*(rand(robot.GetDOF())-.5)
    return vals

def is_ha_joint(name):
    return ha_ind_name_map.has_key(name)

def is_ha_sensor(name):
    #TODO detect sensors from names
   return False

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

def is_finger(name):
    if _re.search(r'^rightKnuckle|^[LR]F[0-5][0-3]?$|^leftKnuckle',name):
        return True
    else:
        return False

if __name__=='__main__':
    import openhubo as oh
    from openhubo import startup
    (env,options)=oh.setup()
    options.stop=False
    options.physics=False
    options.ghost=False

    [robot,ctrl,ind,__,__]=oh.load_scene(env,options)
    print oh_from_ha_names
    jointnames=[j.GetName() for j in robot.GetJoints()]

    print "Mapping from openhubo to hubo-ach"
    for n in jointnames:
        print n,ha_from_oh(n),get_huboname_from_name(n)

    print "Mapping from hubo-ach to openhubo"
    for n in ha_ind_name_map.keys():
        print n,oh_from_ha(n),get_name_from_huboname(n)

    print "Make a direct Index map from openhubo to hubo-ach"
    print ha_ind_from_oh_ind(robot)
