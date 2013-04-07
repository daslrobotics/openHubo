#!/usr/bin/env python

import openravepy as _rave
from numpy import pi,arange,zeros,array
import openhubo as _oh
import openhubo.comps as _comps

hubo_read_trajectory_map={
    'RHY':0,
    'RHR':1,
    'RHP':2,
    'RKN':3,
    'RAP':4,
    'RAR':5,
    'LHY':6,
    'LHR':7,
    'LHP':8,
    'LKN':9,
    'LAP':10,
    'LAR':11,
    'RSP':12,
    'RSR':13,
    'RSY':14,
    'REB':15,
    'RWY':16,
    'RWR':17,
    'RWP':18,
    'LSP':19,
    'LSR':20,
    'LSY':21,
    'LEB':22,
    'LWY':23,
    'LWR':24,
    'LWP':25,
    'NKY':26,
    'NK1':27,
    'NK2':28,
    'WST':29,
    'RF1':30,
    'RF2':31,
    'RF3':32,
    'RF4':33,
    'RF5':34,
    'LF1':35,
    'LF2':36,
    'LF3':37,
    'LF4':38,
    'LF5':39}

def traj_append(traj,waypt):
    n=traj.GetNumWaypoints()
    traj.Insert(n,waypt)

def create_trajectory(robot):
    """ Create a trajectory based on a robot's config spec"""
    traj=_rave.RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    return [traj,config]

def read_swarthmore_traj(filename,robot,dt=.01,retime=True):
    """ Read in trajectory data stored in Swarthmore's format
        (data by row, single space separated)
    """
    #Setup trajectory and source file
    [traj,config]=create_trajectory(robot)

    f=open(filename,'r')

    #Read in blank header row
    f.readline().rstrip()

    #Hard code names based on guess from DoorOpen.txt
    names=['WST','LSP','LSR','LSY','LEP','LWY','LWP']
    signs=[-1,1,-1,-1,1,-1,1]
    offsets=[0,0,15.0,0,0,0,0]
    #names=['WST','RSP','RSR','RSY','REP','RWY','RWP']
    pose=_oh.Pose(robot)

    while True:
        string=f.readline().rstrip()
        if len(string)==0:
            break
        jointvals=[float(x) for x in string.split(' ')]

        for (i,n) in enumerate(names):
            pose[n]=(jointvals[i]-offsets[i])*pi/180*signs[i]
        traj_append(traj,pose.to_waypt(dt))

    if retime:
        print _rave.planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
    f.close()
    return traj

def read_youngbum_traj(filename,robot,dt=.01,scale=1.0,retime=True):
    """ Read in trajectory data stored in Youngbum's format (100Hz data):
        HPY LHY LHR ... RWP   (3-letter names)
        + - + ... +           (sign of joint about equivalent global axis + / -)
        0.0 5.0 2.0 ... -2.0  (Offset of joint from openHubo "zero" in YOUR sign convention)
        (data by row, single space separated)
    """
    #TODO: handle multiple spaces
    #Setup trajectory and source file
    [traj,config]=create_trajectory(robot)
    pose=_oh.Pose(robot)

    f=open(filename,'r')

    #Read in header row to find joint names
    header=f.readline().rstrip()
    names=header.split(' ')

    #Read in sign row
    signlist=f.readline().rstrip().split(' ')
    signs=[]
    for s in signlist:
        if s == '+':
            signs.append(1)
        else:
            signs.append(-1)

    #Read in offset row (fill with zeros if not used)
    offsetlist=f.readline().rstrip().split(' ')
    offsets=[float(x) for x in offsetlist]

    k=0
    while True:
        string=f.readline().rstrip()
        if len(string)==0:
            break
        jointvals=[float(x) for x in string.split(' ')]

        for (i,n) in enumerate(names):
            pose[n]=(jointvals[i]+offsets[i])*pi/180.0*signs[i]*scale

        traj.Insert(k,pose.to_waypt(dt))
        k=k+1
    if retime:
        _rave.planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    return traj

def write_youngbum_traj(traj,robot,dt,filename='exported.traj',dofs=None,oldnames=False):
    """ Create a text trajectory in youngbum's style, assuming no offsets or
    scaling, and openHubo default sign convention.
    """

    config=robot.GetConfigurationSpecification()
    f=open(filename,'w')

    namelist=[]
    signlist=[]
    scalelist=[]
    offsetlist=[]

    if dofs is None:
        dofs=range(robot.GetDOF())
    for d in dofs:
        name=robot.GetJointFromDOFIndex(d).GetName()
        if oldnames:
            namelist.append(_oh.get_huboname_from_name(name))
        else:
            namelist.append(name)
        #TODO make this an argument?
        signlist.append('+')
        offsetlist.append(0.0)
        scalelist.append(1.0)

    #Find overall trajectory properties
    T=traj.GetDuration()

    with  open(filename,'w') as f:
        f.write(' '.join(namelist)+'\n')
        f.write(' '.join(signlist)+'\n')
        f.write(' '.join(['{}'.format(x) for x in offsetlist])+'\n')
        f.write(' '.join(['{}'.format(x) for x in scalelist])+'\n')

        for t in arange(0,T,dt):
            waypt=traj.Sample(t)
            vals=config.ExtractJointValues(waypt,robot,dofs)
            f.write(' '.join(['{}'.format(x) for x in vals])+'\n')

def write_hubo_traj(traj,robot,dt,filename='exported.traj'):
    """ Create a text trajectory for reading into hubo-read-trajectory."""
    config=robot.GetConfigurationSpecification()

    f=open(filename,'w')

    #Find overall trajectory properties
    T=traj.GetDuration()
    #steps=int(T/dt)

    #Get all the DOF's..
    dofs = range(robot.GetDOF())
    with open(filename,'w') as f:
        for t in arange(0,T,dt):
            waypt=traj.Sample(t)
            #Extract DOF values
            vals=config.ExtractJointValues(waypt,robot,dofs)
            #start with array of zeros size of hubo-ach trajectory width
            mapped_vals=zeros(max(_oh.hubo_map.values()))
            for d in dofs:
                n = robot.GetJointFromDOFIndex(d).GetName()
                if _oh.hubo_map.has_key(n):
                    hname=_oh.get_huboname_from_name(n)
                    mapped_vals[hubo_read_trajectory_map[hname]]=vals[d]
            f.write(' '.join(['{}'.format(x) for x in mapped_vals])+'\n')

def read_text_traj(filename,robot,dt=.01,scale=1.0):
    """ Read in trajectory data stored in Youngbum's format (100Hz data):
        HPY LHY LHR ... RWP   (3-letter names)
        + - + ... +           (sign of joint about equivalent global axis + / -)
        0.0 5.0 2.0 ... -2.0  (Offset of joint from openHubo "zero" in YOUR sign
        convention and scale)
        1000 1000 pi/180 pi/180 ... pi/180 (scale of your units wrt openrave
        default)
        (data by row, single space separated)
    """
    #TODO: handle multiple spaces
    #Setup trajectory and source file
    ind=_oh.make_name_to_index_converter(robot)

    f=open(filename,'r')

    #Read in header row to find joint names
    header=f.readline().rstrip()
    #print header.split(' ')
    [traj,config]=create_trajectory(robot)
    k=0
    indices={}
    Tindices={}
    Tmap={'X':0,'Y':1,'Z':2,'R':3,'P':4,'W':5}
    for s in header.split(' '):
        j=ind(s)
        if j>=0:
            indices.setdefault(k,j)
        try:
            Tindices.setdefault(k,Tmap[s])
        except KeyError:
            pass
        except:
            raise
        k=k+1
    #Read in sign row
    signlist=f.readline().rstrip().split(' ')
    signs=[]
    for s in signlist:
        if s == '+':
            signs.append(1)
        else:
            signs.append(-1)

    #Read in offset row (fill with zeros if not used)
    offsetlist=f.readline().rstrip().split(' ')
    offsets=[float(x) for x in offsetlist]
    #Read in scale row (fill with ones if not used)
    scalelist=f.readline().rstrip().split(' ')
    scales=[float(x) for x in scalelist]

    pose=_oh.Pose(robot)
    while True:
        string=f.readline().rstrip()
        if len(string)==0:
            break
        vals=[float(x) for x in string.split(' ')]
        Tdata=zeros(6)
        for i,v in enumerate(vals):
            if indices.has_key(i):
                pose[indices[i]]=(vals[i]+offsets[i])*scales[i]*signs[i]*scale
            elif Tindices.has_key(i):
                Tdata[Tindices[i]]=(vals[i]+offsets[i])*scales[i]*signs[i]*scale
        #TODO: clip joint vals at limits
        #TODO: use axis angle form for RPY data?
        T=array(_comps.Transform.make_transform(Tdata[3:],Tdata[0:3]))
        traj_append(traj,pose.to_waypt(dt,_rave.poseFromMatrix(T)))

    return traj

def makeJointValueExtractor(robot,traj,config):
    """Closure to pull a full body pose out of a trajectory waypoint"""
    def GetJointValuesFromWaypoint(index):
        return config.ExtractJointValues(traj.GetWaypoint(index),robot,range(robot.GetDOF()))
    return GetJointValuesFromWaypoint

def makeTransformExtractor(robot,traj,config):
    """Closure to pull a transform out of a trajectory waypoint"""
    def GetTransformFromWaypoint(index):
        #Ugly way to extract transform because the ExtractAffineValues function
        #is not yet bound
        return _rave.matrixFromPose(traj.GetWaypoint(index)[-8:-1])
    return GetTransformFromWaypoint

