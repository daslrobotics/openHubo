#!/usr/bin/env python

from openravepy import *
import tab
import time
from numpy import *
from numpy.linalg import *
import sys
import openhubo
from rodrigues import *
import re
from LadderGenerator import *
import trajectory
#import debug
from recorder import viewerrecorder
import pickle
import signal, os
import matplotlib.pyplot as plt
import curses
import kbhit

number_of_degrees=57
joint_offsets=zeros(number_of_degrees)
joint_signs=ones(number_of_degrees)
#Default the map to -1 for a missing index
jointmap=-ones(number_of_degrees,dtype=int)
fingers=[u'rightIndexKnuckle1', u'rightIndexKnuckle2', u'rightIndexKnuckle3', u'rightMiddleKnuckle1', u'rightMiddleKnuckle2', u'rightMiddleKnuckle3', u'rightRingKnuckle1', u'rightRingKnuckle2', u'rightRingKnuckle3', u'rightPinkyKnuckle1', u'rightPinkyKnuckle2', u'rightPinkyKnuckle3', u'rightThumbKnuckle1', u'rightThumbKnuckle2', u'rightThumbKnuckle3',u'leftIndexKnuckle1', u'leftIndexKnuckle2', u'leftIndexKnuckle3', u'leftMiddleKnuckle1', u'leftMiddleKnuckle2', u'leftMiddleKnuckle3', u'leftRingKnuckle1', u'leftRingKnuckle2', u'leftRingKnuckle3', u'leftPinkyKnuckle1', u'leftPinkyKnuckle2', u'leftPinkyKnuckle3', u'leftThumbKnuckle1', u'leftThumbKnuckle2', u'leftThumbKnuckle3']

def load_mapping(robot,filename):
    #Ugly use of globals
    with open(filename,'r') as f:
        line=f.readline() #Strip header
        for k in range(6):
            #Strip known 6 dof base
            line=f.readline() 
        while line: 
            datalist=re.split(',| |\t',line.rstrip())
            #print datalist
            j=robot.GetJoint(datalist[1])
            if j:
                dof=j.GetDOFIndex()
                jointmap[dof]=int(datalist[0])-6
                #Note that this corresponds to the IU index...
                joint_signs[dof]=int(datalist[4])
            #Read in next
            line=f.readline()

def load_iu_traj(filename):

    with open(filename) as f:
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
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

        if len(datalist)>1:
            #Assume header is omitted
            print "Total Time Missing"
            total_time=0
        else:
            total_time=float(datalist[0])
            #line 3 has the step size
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

        if len(datalist)>1:
            print "Time Step is Missing, assuming default..."
            timestep=.05
        else:
            timestep=datalist[0]
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

        if (total_time>0) & (timestep>0):
            number_of_steps = (int)(total_time/timestep)
        else:
            number_of_steps = 0

        dataset=[]
        count=0
        while len(line)>0:
            if not line:
                print "Configuration is Missing"
                break

            #Split configuration into a list, throwing out endline characters and strip length
            configlist=re.split(',| |\t',line)[:-1]
            #print line
            #print configlist
            #Convert to float values for the current pose
            data=[float(x) for x in configlist[1:]]
            #print data
            if not(len(data) == int(configlist[0])):
                print "Incorrect data formatting on line P{}".format(c)

            dataset.append(data)
            line = f.readline()
            count+=1

    #Convert to neat numpy array
    return [array(dataset),timestep,total_time,number_of_steps if number_of_steps else count]

def format_angles(data):
    newdata=zeros(size(data))
    for k in range(len(data)):
        if data[k]>pi:
            newdata[k]=2*pi-data[k]
        else:
            newdata[k]=data[k]
    return newdata

def build_openrave_traj(robot,dataset,timestep,retime=True):
    #Assumes that ALL joints are specified for now
    [traj,config]=trajectory.create_trajectory(robot)
    #print config.GetDOF() 
    T0=robot.GetTransform()
    #elbow_start=-pi/180.*170
    #elbow_step=elbow_start/60.0
    #ankle_start=-.04
    #ankle_step=ankle_start/80.0
    for k in range(size(dataset,0)):
        T=get_transform(robot,T0,dataset[k,0:6])
        pose=dataset[k,jointmap+6]*joint_signs+joint_offsets
        #hack to add a slowly decreasing elbow bend
        #elbow_offset=elbow_start-k*elbow_step
        #ankle_offset=ankle_start-k*ankle_step
        #if elbow_offset <0.0:
            #pose[robot.GetJoint('REP').GetDOFIndex()]+=elbow_offset
            #pose[robot.GetJoint('LEP').GetDOFIndex()]+=elbow_offset
            #pose[robot.GetJoint('RSP').GetDOFIndex()]-=elbow_offset/3.5
            #pose[robot.GetJoint('LSP').GetDOFIndex()]-=elbow_offset/3.5

        #if ankle_offset <0.0:
            #if pose[robot.GetJoint('RAP').GetDOFIndex()]>.00:
                #pose[robot.GetJoint('RAP').GetDOFIndex()]=.00
                #pose[robot.GetJoint('LAP').GetDOFIndex()]=.00
            #pose[robot.GetJoint('RAP').GetDOFIndex()]+=ankle_offset
            #pose[robot.GetJoint('LAP').GetDOFIndex()]+=ankle_offset

        for p in range(len(pose)):
            #Make sure limits are enforced and clip them
            if p<robot.GetDOF():
                L=robot.GetJointFromDOFIndex(p).GetLimits()
                if pose[p]>max(L)[0]:
                    pose[p]=max(L)[0]*.99
                if pose[p]<min(L)[0]:
                    pose[p]=min(L)[0]*.99

        #Note this method does not use a controller
        jointvals=pose
        waypt=list(jointvals)
        waypt.extend(RaveGetAffineDOFValuesFromTransform(T,DOFAffine.Transform))
        waypt.append(timestep)
        traj.Insert(k,waypt)

    if retime:
        planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
    return traj

def get_transform(robot,T0,xyzrpy):
    Tc=eye(4)
    #use rodrigues function to build RPY rotation matrix
    Tc[0:3,0:3]=rodrigues(format_angles(xyzrpy[3:6]))
    Tc[0:3,3]=xyzrpy[0:3]
    return array(mat(T0)*mat(Tc))

def make_robot_transform(robot):
    env=robot.GetEnv()
    ladder=env.GetKinBody('ladder')
    T_ladder=ladder.GetTransform()
    #3rd geom is bottom rung
    T_rung=ladder.GetLinks()[0].GetGeometries()[2].GetTransform()
    T_rung_global=mat(T_ladder)*mat(T_rung)
    #print T_rung_global
    #print T_rung_global[3,1]
    T0=robot.GetTransform()
    T1=eye(4)
    T1[0:3,0:3]=rodrigues([0,0,-pi/2])
    #From Jingru, shift back .32 from base
    T1[0:3,3]=array([-.118,.32+T_rung_global[1,3],0.002]).T

    T=array(mat(T0)*mat(T1))
    robot.SetTransform(T)
    return T
   
def play_traj(robot,dataset,timestep):
    #Assume that robot is in initial position now
    T0=robot.GetTransform()
    for k in range(size(dataset,0)):
        T=get_transform(robot,T0,dataset[k,0:6])
        pose=dataset[k,jointmap+6]*joint_signs+joint_offsets
        robot.SetTransform(T)
        robot.SetDOFValues(pose.T)
        time.sleep(timestep)

def get_triggers(robot,dataset):
    #Assume that robot is in initial position now
    T0=robot.GetTransform()
    ftriggers=zeros((size(dataset,0),2),bool)
    lr=[27,42]
    for k in range(size(dataset,0)):
        #left = 27, right = 42
        ftriggers[k,:]=dataset[k,jointmap[lr]]*joint_signs[lr]+joint_offsets[lr]>.85
    return ftriggers

def set_default_limits(robot):
    """ Hack to enforce the stock limits on a version of the Hubo+ robot. Hard
    coded limits pulled from rlhuboplus model, based on shell collision limits
    from home position.
    """
    bases=['HP','KP','AP']
    lower=array([-85,-4,-74])*pi/180.0
    upper=array([92,149,97])*pi/180.0
    for p in ['R','L']:
        for k in range(len(bases)):
            n=p+bases[k]
            print n+ " old limits: {}".format(robot.GetJoint(n).GetLimits())
            robot.GetJoint(n).SetLimits([lower[k]],[upper[k]])

def set_expanded_limits(robot):
    """ Use 10 deg. more as shown by IU planner. """
    bases=['HP','KP','AP']
    lower=(array([-85,-4,-74])-10)*pi/180.0
    upper=(array([92,149,97])+10)*pi/180.0
    for p in ['R','L']:
        for k in range(len(bases)):
            n=p+bases[k]
            print n+ " old limits: {}".format(robot.GetJoint(n).GetLimits())
            robot.GetJoint(n).SetLimits([lower[k]],[upper[k]])

class effector_log:
    def __init__(self,loglen=10,bodies=[]):
        self.width=1
        self.loglen=loglen
        self.bodies=[]
        self.count=0
        self.env=bodies[0].GetParent().GetEnv()
        for b in bodies:
            self.width+=3
            self.bodies.append(b)
        self.robot=self.bodies[0].GetParent()
        self.data=zeros((loglen,self.width))
        self.com=zeros((loglen,3))
        #Data structure is 1 col of time, 6s columns of sensor data
        
    def record(self,time=None):
        if not time:
            self.data[self.count,0]=self.env.GetSimulationTime()
        else:
            self.data[self.count,0]=time
        for k in range(len(self.bodies)):
            #COM is coincident with body frame due to ODE restrictions
            com=self.bodies[k].GetGlobalCOM()
            c0=1+k*3
            c1=1+3*(k+1)
            self.data[self.count,c0:c1]=(com)
            self.com[self.count,:]=openhubo.find_com(self.robot)
        self.count+=1

    def save(self,filename):
        names=[]
        for b in self.bodies:
            names.append(b.GetName())
        robotname=self.bodies[0].GetParent().GetName()
        with open(filename,'w') as f:
            #TODO: save robot hash?
            pickle.dump([self.data[:self.count,:],self.com[:self.count,:],names,robotname],f)

    def load(self,filename):
        with open(filename,'r') as f:
            self.data=pickle.load(f)
            self.count=size(self.data,0)
    def lookup(self,name,components):
        if name=='time':
            return self.data[:self.count,0]
        for k in range(len(self.sensors)):
            if self.sensors[k].GetName()==name:
                c0=1+k*3
                return self.data[:self.count,array(components)+c0]

class force_log:
    def __init__(self,loglen=10,sensors=[]):
        self.width=1
        self.loglen=loglen
        self.sensors=[]
        self.count=0
        self.env=sensors[0].GetAttachingLink().GetParent().GetEnv()
        for s in sensors:
            if type(s.GetData()) is openravepy_int.Sensor.Force6DSensorData:
                self.width+=6
                self.sensors.append(s)
        self.data=zeros((loglen,self.width))
        #Data structure is 1 col of time, 6s columns of sensor data
                       
    def setup(self,histlen):
        for s in self.sensors:
            if type(s.GetData()) is openravepy_int.Sensor.Force6DSensorData:
                FT=s.GetSensor()
                FT.Configure(Sensor.ConfigureCommand.PowerOff)
                time.sleep(0.1)
                FT.SendCommand('histlen {}'.format(histlen))
                time.sleep(0.1)
                FT.Configure(Sensor.ConfigureCommand.PowerOn)
        
    def record(self,time=None):

        #TODO: access violation safety
        
        with env:
            if not time:
                self.data[self.count,0]=self.env.GetSimulationTime()
            else:
                self.data[self.count,0]=time
            for k in range(len(self.sensors)):
                force=self.sensors[k].GetData().force
                torque=self.sensors[k].GetData().torque
                c0=1+k*6
                c1=1+6*(k+1)
                self.data[self.count,c0:c1]=hstack((force,torque))
        self.count+=1

    def save(self,filename):
        names=[]
        for s in self.sensors:
            names.append(s.GetName())
        robotname=self.sensors[0].GetAttachingLink().GetParent().GetName()
        with open(filename,'w') as f:
            #TODO: save robot hash?
            pickle.dump([self.data[:self.count,:],names,robotname],f)

    def load(self,filename):
        with open(filename,'r') as f:
            self.data=pickle.load(f)
            self.count=size(self.data,0)
    def lookup(self,name,components):
        if name=='time':
            return self.data[:self.count,0]
        for k in range(len(self.sensors)):
            if self.sensors[k].GetName()==name:
                c0=1+k*6
                c1=1+6*(k+1)
                return self.data[:self.count,array(components)+1+6*k]
    
def add_torque(robot,joints,maxT,level=3):
    """ Add torque to joints, assuming that they are fingers by the indices."""
    if level>0:
        for j in joints[0::3][:-1]:
            j.AddTorque([maxT*.5])
        joints[-3].AddTorque([maxT*1.0])

    if level>1:
        for j in joints[1::3][:-1]:
            j.AddTorque([maxT*.25])
        joints[-2].AddTorque([maxT*.5])
    if level>2:
        for j in joints[2::3][:-1]:
            j.AddTorque([maxT*.125])
        joints[-1].AddTorque([maxT*.25])

def save(self,filename,struct):
    with open(filename,'w') as f:
        #TODO: save robot hash?
        pickle.dump(stuct, f)

def set_finger_torque(robot,maxT):
    for f in fingers:
        if robot.GetJoint(f):
            robot.GetJoint(f).SetTorqueLimits([maxT])
            robot.GetJoint(f).SetVelocityLimits([3])
            robot.GetJoint(f).SetAccelerationLimits([30])

def wait_start():
    print "Press Enter to abort, starting simulation in ... "
    for x in range(5,0,-1):
        print '{}...'.format(x)
        for k in range(10):
            time.sleep(.1)
            k=False
            if kbhit.kbhit():
                k=kbhit.getch()
                if k:
                    return False
    print "Starting simulation!"
    return True

if __name__=='__main__':

    try:
        file_param = sys.argv[1]
    except IndexError:
        file_param = 'firstladder.iuparam'
    
    try:
        file_traj = sys.argv[2]
    except IndexError:
        print "Trajectory argument not found!"
        raise 

    try:
        file_robot = sys.argv[3]
    except IndexError:
        print "Robot file not found!"
        raise 

    try:
        Tmax = float(sys.argv[4])
    except IndexError:
        Tmax=1.6
        pass

    try:
        ##Hack to get limits optionally overridden for shell robot
        expand_limits = float(sys.argv[5])
    except IndexError:
        expand_limits = -1
        pass

    try:
        if sys.argv[6]=='0':
            showGUI = False
        else:
            showGUI = True
    except IndexError:
        showGUI = True
        pass


    #Assume that name has suffix of some sort!
    # Try to keep parameters specified here
    laddername=make_ladder(file_param,True)
    envname=make_ladder_env(file_param,True)
    
    physicson=True

    env = Environment()
    env.SetViewer('qtcoin',showGUI)
    env.SetDebugLevel(3)
   
    # rlhuboplus models don't work with the affine transformations (i.e. use huboplus.robot.xml for ideal sim   
    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,file_robot,envname,True,physicson)
    
    make_robot_transform(robot)

    #TODO: get rid of hand tweaks by processing initial location?
    
    joint_offsets[ind('RSR')]=pi/12
    joint_offsets[ind('LSR')]=-pi/12

    for n in fingers:
        if n.find('Thumb')<0:
            joint_signs[ind(n)]*=2
    print joint_signs

    # Use simulation to settle robot on the ground
    env.StartSimulation(0.0005,False)
    time.sleep(1)
    env.StopSimulation()

    #Import the joint map and read in trajectory
    load_mapping(robot,"iumapping.txt")
    [dataset,timestep,total_time,number_of_steps]=load_iu_traj(file_traj)

    ## Change this to affect maximum applied hand torque
    suffix='_{}'.format(Tmax)

    timestamp=openhubo.get_timestamp()
    recorder.filename='.'.join(laddername.split('.')[:-2])+timestamp+suffix+'_physics.avi'
    recorder.videoparams[0:2]=[800,600]
    traj=build_openrave_traj(robot,dataset,timestep,True)
    ctrl.SetPath(traj)
    t_total=traj.GetDuration()

    steps=int(t_total/0.0005)
    forces=force_log(steps,[robot.GetAttachedSensor(x) for x in ['rightFootFT','leftFootFT']])
    points=effector_log(steps,[robot.GetLink(x) for x in ['leftFoot','rightFoot','leftPalm','rightPalm']])
    forces.setup(50)
    set_finger_torque(robot,10.0)

    right_joints=[]
    left_joints=[]
    for n in fingers:
        if n.find('left')>-1:
            left_joints.append(robot.GetJoint(n))
        if n.find('right')>-1:
            right_joints.append(robot.GetJoint(n))
    #print 'Type a letter and enter to toggle hand torque (b = both, l / r = left / right, a in, z dec):'

    ctrl.SendCommand('start')
    rflag=False
    lflag=False
    rtorque=0.0
    ltorque=0.0

    if expand_limits==0:
        set_default_limits(robot)
    elif expand_limits>0:
        set_expanded_limits(robot)
    else:
        print "Using stock limits for current robot!"


    #openhubo.set_robot_color(robot,[.5,.5,.5],[.5,.5,.5],.4)
    triggers=get_triggers(robot,dataset)
    count=0
    #env.Load('kinbody/backrest.kinbody.xml')
    start=wait_start()
    if start:
        recorder.start()
    while not ctrl.IsDone() and start:
        env.StepSimulation(0.0005)
        handle=openhubo.plotProjectedCOG(robot)
        #TODO: make this depend on input timestep!!!
        #hackjob...
        if not (count % 100):
            try:
                lflag=triggers[count/100,0]
                rflag=triggers[count/100,1]
            except IndexError:
                pass
        
        if not (count % 20):
            forces.record()
            points.record()
            com=openhubo.find_com(robot)
            if com[2]<.3:
                #Robot fell over
                ctrl.Reset()
                break
            #handles=openhubo.plot_masses(robot)
            #if kbhit.kbhit():
                #k=kbhit.getch()
                #if k=='r' or k=='b':
                    #rflag=not rflag 
                    #print "Switch rtorque to {}".format(rflag)

                #if k=='l' or k=='b':
                    #lflag=not lflag 
                    #print "Switch ltorque to {}".format(lflag)

                #elif k=='a':
                    #print "Raising Tmax by .25"
                    #Tmax+=.25
                #elif k=='z':
                    #print "Lowering Tmax by .25"
                    #Tmax-=.25
                
        #if rflag:
            #if rtorque<Tmax:
                #rtorque+=.001
            #add_torque(robot,right_joints,rtorque)
        #else:
            #rtorque=0.0
        #if lflag:
            #if ltorque<Tmax:
                #ltorque+=.001
            #add_torque(robot,left_joints,ltorque)
        #else:
            #ltorque=0.0
        count+=1

    recorder.stop()

    if ctrl.IsDone():
        print "Trajectory is successful!"
        suffix+="_success"

    prefix='.'.join(laddername.split('.')[:-2])
    outname=prefix + timestamp + suffix
    forces.save(outname + '_forces.pickle')
    points.save(outname + '_points.pickle')
    with open(outname + '_misc.pickle','w') as f:
        pickle.dump([triggers,count,Tmax],f)

    #Log misc data
    logname= outname + '.log'
    tableentries=[outname,file_robot,'Expanded' if expand_limits>0 else 'Original',prefix,'{} {}'.format(count,int(number_of_steps*timestep/0.0005)),'success' if ctrl.IsDone() else 'failure']
    with open(logname,'w') as f:
        f.write(' '.join(tableentries) + '\n')
        f.write('Robot: ' + file_robot + '\n')
        f.write('Limits expanded: {}\n'.format(expand_limits))
        bases=['HP','KP','AP']
        for p in ['R','L']:
            for k in range(len(bases)):
                n=p+bases[k]
                f.write(n+" Limits = {}\n".format(robot.GetJoint(n).GetLimits()))
        f.write('Completed timesteps = {}\n'.format(count))
        f.write('Completed time = {}\n'.format(count*0.0005))
    with open('batch.out','a') as f:
        f.write(' '.join(tableentries) + '\n')
