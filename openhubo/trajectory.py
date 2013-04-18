import openravepy as _rave
import numpy as _np
from numpy import pi,array
import openhubo as _oh
import openhubo.comps as _comps
import re


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

    #Read in offset row (fill with _np.zeros if not used)
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

        for t in _np.arange(0,T,dt):
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
        for t in _np.arange(0,T,dt):
            waypt=traj.Sample(t)
            #Extract DOF values
            vals=config.ExtractJointValues(waypt,robot,dofs)
            #start with array of _np.zeros _np.size of hubo-ach trajectory width
            mapped_vals=_np.zeros(max(_oh.hubo_map.values()))
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

    #Read in offset row (fill with _np.zeros if not used)
    offsetlist=f.readline().rstrip().split(' ')
    offsets=[float(x) for x in offsetlist]
    #Read in scale row (fill with _np.ones if not used)
    scalelist=f.readline().rstrip().split(' ')
    scales=[float(x) for x in scalelist]

    pose=_oh.Pose(robot)
    while True:
        string=f.readline().rstrip()
        if len(string)==0:
            break
        vals=[float(x) for x in string.split(' ')]
        Tdata=_np.zeros(6)
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

class IUTrajectory:

    def __init__(self,robot,mapfile=None):
        #TODO: get defaults that make sense
        self.joint_offsets=_np.zeros(robot.GetDOF())
        self.joint_signs=_np.ones(robot.GetDOF())
        self.joint_map=-_np.ones(robot.GetDOF(),dtype=_np.int)
        self.robot=robot
        if mapfile:
            self.load_mapping(_os.mapfile)

    def load_mapping(self,filename,path=None):
        #Ugly use of globals

        with open(_oh.find(filename,path),'r') as f:
            line=f.readline() #Strip header
            for k in range(6):
                #Strip known 6 dof base
                line=f.readline()
            while line:
                datalist=re.split(',| |\t',line.rstrip())
                #print datalist
                j=self.robot.GetJoint(datalist[1])
                if j:
                    dof=j.GetDOFIndex()
                    self.joint_map[dof]=int(datalist[0])-6
                    #Note that this corresponds to the IU index...
                    self.joint_signs[dof]=int(datalist[4])
                    self.joint_offsets[dof]=float(datalist[5])
                #Read in next
                line=f.readline()

    def load_from_file(self,filename,mapfile=None,path=None):

        if mapfile:
            self.load_mapping(mapfile,path)

        with open(_oh.find(filename,path)) as f:
            """ Format of q_path file:
                version string
                runtime
                timestep
                val0,val1...val57
            """
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

            if len(datalist)>1:
                #Assume header is omitted
                print "Hubo Version Missing"
                version="HuboDefault"
            else:
                version=datalist[0]
                #line 2 has the total time
                print version
                line = f.readline()
                datalist=re.split(',| |\t',line)[:-1]

            if len(datalist)>1:
                #Assume header is omitted
                print "Total Time Missing"
                #total_time=0
            else:
                #total_time=float(datalist[0])
                #line 3 has the step _np.size
                line = f.readline()
                datalist=re.split(',| |\t',line)[:-1]

            if len(datalist)>1:
                print "Time Step is Missing, assuming default..."
                timestep=.05
            else:
                timestep=datalist[0]
                line = f.readline()
                datalist=re.split(',| |\t',line)[:-1]

            dataset=[]
            count=0
            while len(line)>0:
                if not line:
                    print "Configuration is Missing"
                    break

                #Split configuration into a list, throwing out endline characters and strip length
                configlist=re.split(',| |\t',line)[:-1]

                #Convert to float values for the current pose
                data=[float(x) for x in configlist[1:]]
                if not(len(data) == int(configlist[0])):
                    print "Incorrect data formatting on line P{}".format(count)

                dataset.append(data)
                line = f.readline()
                count+=1

        #Convert to neat numpy array
        self.dataset=array(dataset)
        self.timestep=timestep

    def total_time(self):
        return self.timestep*_np.size(self.dataset,1)

    def steps(self):
        return _np.size(self.dataset,1)

    @staticmethod
    def format_angles(data):
        newdata=_np.zeros(_np.size(data))
        for k in range(len(data)):
            if data[k]>pi:
                newdata[k]=2*pi-data[k]
            else:
                newdata[k]=data[k]
        return newdata

    @staticmethod
    def get_transform(T0,xyzrpy):
        Tc=_np.eye(4)
        #use rodrigues function to build RPY rotation matrix
        Tc[0:3,0:3]=_comps.rodrigues(IUTrajectory.format_angles(xyzrpy[3:6]))
        Tc[0:3,3]=xyzrpy[0:3]
        return array(_np.mat(T0)*_np.mat(Tc))


    def to_openrave(self,dt=None,retime=True,clip=True):
        #Assumes that ALL joints are specified for now
        [traj,config]=create_trajectory(self.robot)
        #print config.GetDOF()
        T0=self.robot.GetTransform()
        pose=_oh.Pose(self.robot)
        (lower,upper)=self.robot.GetDOFLimits()

        for k in xrange(_np.size(self.dataset,0)):
            T=IUTrajectory.get_transform(T0,self.dataset[k,0:6])
            raw_pose=self.dataset[k,self.joint_map+6]
            pose.values=raw_pose*self.joint_signs+self.joint_offsets

            if clip:
                pose.values=_np.maximum(pose.values,lower*.999)
                pose.values=_np.minimum(pose.values,upper*.999)

            #Note this method does not use a controller
            aff = _rave.RaveGetAffineDOFValuesFromTransform(T,_rave.DOFAffine.Transform)
            if dt<0 or dt is None:
                dt=self.timestep
            traj_append(traj,pose.to_waypt(dt,aff))

        if retime:
            _rave.planningutils.RetimeActiveDOFTrajectory(traj,self.robot,True)
        #Store locally because why not
        self.traj=traj
        return traj

    def play_traj(self,resetafter=True):
        #Assume that robot is in initial position now
        T0=self.robot.GetTransform()
        for k in xrange(self.dataset.shape[0]):
            T=self.get_transform(T0,dataset[k,0:6])
            pose=dataset[k,self.jointmap+6]*self.joint_signs+self.joint_offsets
            self.robot.SetTransform(T)
            self.robot.SetDOFValues(pose.T)
            time.sleep(timestep)

        if resetafter:
            self.robot.SetTransform(T0)



