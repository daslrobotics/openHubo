import numpy as np
import re as _re
from numpy.random import rand
#KLUDGE: hard code the mapping (how often will it change, really?). Include openhubo synonyms here for fast lookup.
import yaml

#Hack to get global synonyms
synonyms={'LKP':'LKN',
          'RKP':'RKN',
          'REP':'REB',
          'LEP':'LEB',
          'TSY':'WST'}

inv_synonyms={v:k for (k, v) in synonyms.iteritems()}

#Hack way to get hubo.h file
#FIXME: search paths?
hubo_h='/usr/include/hubo.h'
try:
    with open(hubo_h):pass
except IOError:
    print "hubo.h not found in {}, assuming local".format(hubo_h)
    hubo_h='/usr/local/include/hubo.h'

hubo_ach_map={}
with open(hubo_h,'r') as f:
    for l in f:
        datalist=l.split('\t')
        if datalist[0] == '#define' and len(datalist)>4:
            #print datalist
            try:
                hubo_ach_map[datalist[2]]=int(datalist[4])
            except:
                pass


oh_from_ha_names={k:(inv_synonyms[k] if k in inv_synonyms.keys() else k) for k in hubo_ach_map.keys()}

deprecated_names={'HPY':'WST',
                  'HDY':'NKY',
                  'HDP':'NK2',
                  'HNR':'NK2',
                  'HDR':'NK1',
                  'HNR':'NK1',
                  'HNP':'NK2'}

def ha_from_oh(inname):
    """ Get the hubo-ach name of a joint from the openhubo name"""
    if hubo_ach_map.has_key(inname):
        return inname
    elif synonyms.has_key(inname):
        return synonyms[inname]
    elif deprecated_names.has_key(inname):
        return deprecated_names[inname]
    else:
        return None

def oh_from_ha(inname):
    """ Get the openhubo name of a joint from the hubo-ach name"""
    if inv_synonyms.has_key(inname):
        return inv_synonyms[inname]
    elif hubo_ach_map.has_key(inname):
        return inname
    else:
        return None

def slow_lookup_joint(inname):
    """Pass a name through all the mapping functions to brute force a match"""
    return oh_from_ha(ha_from_oh(inname))

def get_name_from_huboname(inname,robot=None):
    """ DEPRECATED-Map a name from the openhubo standard to the original hubo naming
    scheme.
    """
    return oh_from_ha(inname)

def get_huboname_from_name(inname):
    """Get a hubo-standard joint name from the openhubo name (mostly the same,
    but they differ slightly for some joints.
    """
    return ha_from_oh(inname)

def compute_ha_ind_from_oh_ind(robot):
    """Make a direct ind map between a robot and the hubo-ach interface"""
    return {j.GetDOFIndex():(hubo_ach_map[ha_from_oh(j.GetName())] if ha_from_oh(j.GetName()) else None) for j in robot.GetJoints()}

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
    return hubo_ach_map.has_key(name)

def is_ha_sensor(name):
    #TODO detect sensors from names
    pass

def is_finger(name):
    if _re.search(r'^Knuckle|^[LR]F[0-5][0-3]?$',name):
        return True
    else:
        return False

def get_fingers(robot):
    return [j for j in robot.GetJoints() if is_finger(j.GetName())]

def get_right_fingers(robot):
    return [f for f in get_fingers(robot) if _re.search('R|right',f.GetName())]

def get_left_fingers(robot):
    return [f for f in get_fingers(robot) if _re.search('L|left',f.GetName())]

def write_yaml_file(filename='mapout.yaml'):
    outdict={}
    urdf_mapping={oh_from_ha(k):{'huboachid':v} for k,v in hubo_ach_map.items()}
    outdict['joints']=urdf_mapping.keys()
    outdict['mapping']=urdf_mapping
    with open(filename, 'w') as outfile:
        outfile.write( yaml.dump(outdict, default_flow_style=False) )

if __name__=='__main__':
    #Test name mapping
    pass
