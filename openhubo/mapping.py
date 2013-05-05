import numpy as np
from numpy.random import rand
#KLUDGE: hard code the mapping (how often will it change, really?). Include openhubo synonyms here for fast lookup.
ha_ind_name_map={'RHY':26,
                   'RHR':27,
                   'RHP':28,
                   'RKN':29,
                   'RKP':29,
                   'RAP':30,
                   'RAR':31,
                   'LHY':19,
                   'LHR':20,
                   'LHP':21,
                   'LKN':22,
                   'LKP':22,
                   'LAP':23,
                   'LAR':24,
                   'RSP':11,
                   'RSR':12,
                   'RSY':13,
                   'REB':14,
                   'REP':14,
                   'RWY':15,
                   'RWR':16,
                   'RWP':17,
                   'LSP':4,
                   'LSR':5,
                   'LSY':6,
                   'LEB':7,
                   'LEP':7,
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
                  'HNR':'NK1'}


def get_name_from_huboname(inname,robot=None):
    """ Map a name from the openhubo standard to the original hubo naming
    scheme.
    """
    name=oh_from_ha(inname)
    if robot and name:
        j=robot.GetJoint(name)
        if j:
            return name

    else:
        return name

def ha_from_oh(inname):
    """ If the input name is either a hubo-ach standard name or an openhubo
    synonym, return the matching hubo-ach name."""
    if synonyms.has_key(inname):
        return synonyms[inname]
    elif ha_ind_name_map.has_key(inname):
        return inname
    else:
        return None

def oh_from_ha(inname):
    """ If the input name is an openhubo joint name, return the matching hubo-ach name."""
    if ha_ind_name_map.has_key(inname):
        if inv_synonyms.has_key(inname):
            return inv_synonyms[inname]
        return inname
    else:
        return None

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


if __name__=='__main__':
    import openhubo as oh
    from  openhubo import startup
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


