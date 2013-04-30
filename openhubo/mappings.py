#KLUDGE: hard code the mapping (how often will it change, really?). Include openhubo synonyms here for fast lookup.
ha_ind_from_names={'RHY':26,
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
        synonyms.setdefault(s+f+'Knuckle1','LF{}'.format(i))


inv_synonyms={(v,k) for k, v in synonyms.iteritems()}

oh_from_ha_names={(k,inv_synonyms[k] if k in inv_synonyms else k) for k in ha_ind_from_names.keys()}

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
    elif ha_ind_from_names.has_key(inname):
        return inname
    else:
        return None

def oh_from_ha(inname):
    """ If the input name is an openhubo joint name, return the matching hubo-ach name."""
    if oh_from_ha_names.has_key(inname):
        return synonyms[inname]
    else:
        return None

def get_huboname_from_name(inname):
    """Get a hubo-standard joint name from the openhubo name (mostly the same,
    but they differ slightly for some joints.
    """
    return ha_from_oh(inname)

def build_joint_index_map(robot):
    """ Low level function to build a map of joint names and indices as a
    python dictionary. The dict is used for simplicity and flexibility."""
    return {(j.GetDOFIndex(),ha_from_oh(j.GetName())) for j in robot.GetJoints()}

