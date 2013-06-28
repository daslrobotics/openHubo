import openravepy as _rave
import time as _time
import numpy as _np
from numpy import pi,array,mat
from numpy.linalg import inv
import openhubo as _oh
import openhubo.comps as _comps
from openhubo import mapping
import re
from openhubo import kbhit


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
    """quickly append a waypoint to a trajectory"""
    n=traj.GetNumWaypoints()
    traj.Insert(n,waypt)

def create_trajectory(robot,waypts=None,filename=None):
    """ Create a trajectory based on a robot's config spec. Optionally added a list of waypoints """
    traj=_rave.RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)

    if waypts is not None:
        _rave.raveLogInfo("Appending waypoint(s)")
        try:
            for w in waypts:
                traj_append(traj,w)
        except TypeError:
            #fallthrough if single waypoint
            traj_append(traj,waypts)

    if filename is not None:
        read_trajectory_from_file(traj,filename)

    return [traj,config]

def read_trajectory_from_file(traj,filename):
    with open(filename) as f:
        s=f.read()
        traj.deserialize(s)

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

    for line in f:
        jointvals=[float(x) for x in line.rstrip().split(' ')]

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
    for line in f:
        jointvals=[float(x) for x in line.rstrip().split(' ')]

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
            namelist.append(mapping.ha_from_oh(name))
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

def convert_openrave_to_hubo_traj(robot,source,dest,dt):
    """Convert an openrave-formatted trajectory text file to a hubo-read text file."""
    traj,__=create_trajectory(robot)
    read_trajectory_from_file(traj,source)
    write_hubo_traj(traj,robot,dt,dest)

def write_hubo_traj(traj,robot,dt,filename='exported.traj'):
    """ Create a text trajectory for reading into hubo-read-trajectory."""
    #Find overall trajectory properties
    T=traj.GetDuration()
    #steps=int(T/dt)
    hr_from_oh_map=dofmap_huboread_from_oh(robot)
    val_sampler=make_joint_value_sampler(robot,traj)
    #TODO: scale the fingers appropriately?
    with open(filename,'w') as f:
        for t in _np.arange(0,T,dt):
            vals=val_sampler(t)
            outdata=_np.zeros(max(hr_from_oh_map.values())+1)
            mapped_vals={v:vals[k] for k,v in hr_from_oh_map.items()}

            for k,v in mapped_vals.items():
                outdata[k]=v

            f.write(' '.join([str(x) for x in outdata])+'\n')

def dofmap_huboread_from_oh(robot):
    #Get all the DOF's..
    hr_from_oh_map={}
    for d in xrange(robot.GetDOF()):
        n = robot.GetJointFromDOFIndex(d).GetName()
        hname=mapping.ha_from_oh(n)
        if hname:
            hr_from_oh_map[d]=hubo_read_trajectory_map[hname]
        #print d,n,hname
    return hr_from_oh_map

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
    for line in f:
        vals=[float(x) for x in line.rstrip().split(' ')]
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

def make_joint_value_sampler(robot,traj,config=None):
    if config is None:
        config=robot.GetConfigurationSpecification()
    """Closure to pull a full body pose out of a trajectory waypoint"""
    def GetJointValuesFromWaypoint(t):
        return config.ExtractJointValues(traj.Sample(t),robot,xrange(robot.GetDOF()))
    return GetJointValuesFromWaypoint

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
    """Import and use trajectories exported from RobotSim."""

    def __init__(self,robot,mapfile=None,trajfile=None):
        #TODO: get defaults that make sense
        self.joint_offsets=_np.zeros(robot.GetDOF())
        self.joint_signs=_np.ones(robot.GetDOF())
        self.joint_map={}
        self.robot=robot
        #TODO: read in the base body from transform?
        self.base=self.robot.GetLink('Body_Torso')
        print self.base
        if trajfile:
            self.load_from_file(trajfile,mapfile)
        elif mapfile:
            self.load_mapping(mapfile)

    def load_mapping(self,filename,path=None):

        with open(_oh.find(filename,path),'r') as f:
            line=f.readline() #Strip header
            self.affine_signs=[]
            self.affine_offsets=[]
            for k in range(6):
                line=f.readline()
                datalist=re.split(',| |\t',line.rstrip())
                self.affine_signs.append(int(datalist[4]))
                self.affine_offsets.append(int(datalist[5]))
            for line in f:
                datalist=re.split(',| |\t',line.rstrip())
                #print datalist
                j=self.robot.GetJoint(datalist[1])

                if j:
                    print j
                    dof=j.GetDOFIndex()
                    self.joint_map[dof]=int(datalist[0])
                    #Note that this corresponds to the IU index...
                    self.joint_signs[dof]=int(datalist[4])
                    self.joint_offsets[dof]=float(datalist[5])

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

            srcdata=[]
            count=0
            for line in f:
                #Split configuration into a list, throwing out endline characters and strip length
                configlist=re.split(',| |\t',line)[:-1]

                #Convert to float values for the current pose
                data=[float(x) for x in configlist[1:]]
                if not(len(data) == int(configlist[0])):
                    print "Incorrect data formatting on line P{}".format(count)

                srcdata.append(data)
                count+=1

        #Convert to neat numpy array
        self.srcdata=array(srcdata)
        self.timestep=timestep
        #FIXME: Ugly hack to get env without knowing which one is really what we want
        #retime = True if _oh.check_physics(_rave.RaveGetEnvironments()[0]) else False
        #print retime
        return self.to_openrave(retime=False)

    def total_time(self):
        return self.timestep*_np.size(self.srcdata,1)

    def steps(self):
        return _np.size(self.srcdata,1)

    @staticmethod
    def split_line(f):
        line=f.readline()
        return re.split(',| |\t',line)[:-1]

    @staticmethod
    def format_angles(raw_angles):
        primary_angles=_np.mod(raw_angles,2*pi)
        return (primary_angles>pi)*(-2.*pi)+primary_angles

    def get_transform(self,T_ref,iu_pose):
        """Get trajectory transform that would be applied to base link from
        traj, and return the equivalent transform from the given starting pose
        and the robot's actual base link."""
        T0=mat(T_ref)
        with self.robot:
            #1) Copy in IU trajectory transform and format
            Tc=mat(_np.eye(4))
            xyzrpy=iu_pose*self.affine_signs+self.affine_offsets
            #FLip yaw and roll (seems to be the best)
            roll=xyzrpy[3]
            pitch=xyzrpy[4]
            yaw=xyzrpy[5]
            xyzrpy[3]=yaw
            xyzrpy[4]=pitch
            xyzrpy[5]=roll
            #use rodrigues function to build RPY rotation matrix
            Tc[0:3,0:3]=_comps.rodrigues(self.format_angles(xyzrpy[3:6]))
            Tc[0:3,3]=mat(xyzrpy[0:3]).T

            #2) Set robot's 0-link at origin
            self.robot.SetTransform(_np.eye(4))

            #3) Find offset in current pose to desired base link (spec'd in trajectory)
            T_B0=mat(self.base.GetTransform())

            #4) Find relative transform to desired position
            T_CB=Tc * T_B0.I

            return array(T0*T_CB)

    def to_openrave(self,dt=None,retime=True,clip=True):
        #Assumes that ALL joints are specified for now
        [traj,config]=create_trajectory(self.robot)
        #print config.GetDOF()
        with self.robot:
            T0=self.base.GetTransform()
            print T0
            pose=_oh.Pose(self.robot)
            (lower,upper)=self.robot.GetDOFLimits()

            for k in xrange(_np.size(self.srcdata,0)):
            #for k in xrange(3):
                #print T[0:3,3]
                pose_map={key:self.srcdata[k,v] for (key,v) in self.joint_map.items()}
                pose_array=array([pose_map[dof] if pose_map.has_key(dof) else 0.0 for dof in xrange(self.robot.GetDOF())])

                pose.values=pose_array*self.joint_signs+self.joint_offsets
                pose.send(True)
                T=self.get_transform(T0,self.srcdata[k,0:6])

                if clip:
                    #oldvals=pose.values
                    pose.values=_np.maximum(pose.values,lower*.999)
                    pose.values=_np.minimum(pose.values,upper*.999)

                    #err = pose.values-oldvals
                    #if sum(abs(err))>0:
                        #print k,err

                #print pose.values
                #Note this method does not use a controller
                aff = _rave.RaveGetAffineDOFValuesFromTransform(T,_rave.DOFAffine.Transform)
                if dt<0 or dt is None:
                    dt=self.timestep
                traj_append(traj,pose.to_waypt(dt,aff))

        print "retiming? {}".format(retime)
        if retime:
            dummy_limits=_np.ones(self.robot.GetDOF()+7)*100
            _rave.planningutils.RetimeAffineTrajectory(traj,dummy_limits,dummy_limits,hastimestamps=True)
            #_rave.planningutils.RetimeActiveDOFTrajectory(traj,self.robot,True)
        #Store locally because why not
        self.traj=traj
        return traj

    def preview_traj(self):
        #Assume that robot is in initial position now
        T0=self.base.GetTransform()
        s=self.robot.CreateRobotStateSaver()
        for k in xrange(self.srcdata.shape[0]):
            pose_map={key:self.srcdata[k,v] for (key,v) in self.joint_map.items()}
            pose_array=array([pose_map[dof] if pose_map.has_key(dof) else 0.0 for dof in xrange(self.robot.GetDOF())])
            values=pose_array*self.joint_signs+self.joint_offsets
            self.robot.SetDOFValues(values)
            T=self.get_transform(T0,self.srcdata[k,0:6])
            self.robot.SetTransform(T)
            _time.sleep(self.timestep/2.)
            if kbhit.kbhit():
                kbhit.getch()
                _oh.pause()
        s.Restore()

