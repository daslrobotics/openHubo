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
import debug
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
            print "Total Time Missing"
            total_time=0
        else:
            total_time=float(datalist[0])
            #line 3 has the step size
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

        if len(datalist)>1:
            print "Time Step is Missing, assuming default..."
            timestep=.1
        else:
            timestep=datalist[0]
            line = f.readline()
            datalist=re.split(',| |\t',line)[:-1]

        if (total_time>0) & (timestep>0):
            number_of_steps = (int)(total_time/timestep)
        else:
            number_of_steps = 0

        dataset=[]
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
    #Convert to neat numpy array
    return [array(dataset),timestep,total_time,number_of_steps]

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
    print config.GetDOF() 
    T0=robot.GetTransform()
    for k in range(size(dataset,0)):
        T=get_transform(robot,T0,dataset[k,0:6])
        pose=dataset[k,jointmap+6]*joint_signs+joint_offsets
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
    print T_rung_global
    print T_rung_global[3,1]
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
            j.AddTorque([maxT*.0625])
        joints[-1].AddTorque([maxT*.125])

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

    #Assume that name has suffix of some sort!
    # Try to keep parameters specified here
    laddername=make_ladder(file_param)
    envname=make_ladder_env(file_param)
    physicson=True

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
   
    # rlhuboplus models don't work with the affine transformations (i.e. use huboplus.robot.xml for ideal sim   
    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,'rlhuboplus.noshell.robot.xml',envname,True,physicson)
    
    make_robot_transform(robot)

    #TODO: get rid of hand tweaks by processing initial location?
    
    joint_offsets[ind('RSR')]=pi/12
    joint_offsets[ind('LSR')]=-pi/12

    #Read the file to obtain time steps and the total time

    env.StartSimulation(timestep=0.0005)
    time.sleep(1)
    env.StopSimulation()
    load_mapping(robot,"iumapping.txt")
    [dataset,timestep,total_time,number_of_steps]=load_iu_traj(file_traj)
    timestep=0.05

    if physicson:
        timestamp=openhubo.get_timestamp()
        recorder.filename='.'.join(laddername.split('.')[:-2])+timestamp+'_physics.avi'
        traj=build_openrave_traj(robot,dataset,timestep,True)
        ctrl.SetPath(traj)
        t_total=traj.GetDuration()
        steps=int(t_total/0.0005)
        forces=force_log(steps,robot.GetAttachedSensors())
        forces.setup(50)
        set_finger_torque(robot,.1)
        right_joints=[]
        left_joints=[]
        for n in fingers:
            if n.find('left')>-1:
                left_joints.append(robot.GetJoint(n))
            if n.find('right')>-1:
                right_joints.append(robot.GetJoint(n))

        #if raw_input('Hit any key to run simulation or enter to skip:'):
        if True:
            print 'Type a letter and enter to toggle hand torque (b = both, l / r = left / right, a in, z dec):'

            #recorder.start()
            ctrl.SendCommand('start')
            rflag=False
            lflag=False
            Tmax=3.0
            rtorque=0.0
            ltorque=0.0
            #openhubo.set_robot_color(robot,[.5,.5,.5],[.5,.5,.5],.4)
            while not ctrl.IsDone():
                env.StepSimulation(0.0005)
                handle=openhubo.plotProjectedCOG(robot)
                forces.record()
                if not (forces.count % 20):
                    com=openhubo.find_com(robot)
                    if com[2]<.3:
                        #Robot fell over
                        ctrl.Reset()
                        break
                    #handles=openhubo.plot_masses(robot)
                    if kbhit.kbhit():
                        k=kbhit.getch()
                        if k=='r' or k=='b':
                            rflag=not rflag 
                            print "Switch rtorque to {}".format(rflag)

                        if k=='l' or k=='b':
                            lflag=not lflag 
                            print "Switch ltorque to {}".format(lflag)

                        elif k=='a':
                            print "Raising Tmax by .25"
                            Tmax+=.25
                        elif k=='z':
                            print "Lowering Tmax by .25"
                            Tmax-=.25
                        
                if rflag:
                    if rtorque<Tmax:
                        rtorque+=.001
                    add_torque(robot,right_joints,rtorque)
                else:
                    rtorque=0.0
                if lflag:
                    if ltorque<Tmax:
                        ltorque+=.001
                    add_torque(robot,left_joints,ltorque)
                else:
                    ltorque=0.0

            forces.save('.'.join(laddername.split('.')[:-2])+timestamp+'_forces.pickle')
    else:
        if raw_input('Hit any key to run simulation or enter to skip:'):
            recorder.filename='.'.join(laddername.split('.')[:-2])+'_ideal.avi'
            recorder.realtime=True
            recorder.start()
            play_traj(robot,dataset,timestep)

    #recorder.stop()

