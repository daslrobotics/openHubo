#!/usr/bin/env python

#Basics
import sys
import openhubo
import time
import re
from numpy import pi,zeros,ones,array,size,mat,eye,hstack
from openravepy.openravepy_int import Sensor
from openhubo.comps import rodrigues
from openhubo.IU import IULadderGenerator
import openhubo.trajectory as tr

#Data saving and handling
import pickle

#OpenRAVE and OpenHubo stuff
import kbhit

number_of_degrees=57
joint_offsets=zeros(number_of_degrees)
joint_signs=ones(number_of_degrees)
#Default the map to -1 for a missing index
jointmap=-ones(number_of_degrees,dtype=int)
fingers=[u'rightIndexKnuckle1', u'rightIndexKnuckle2', u'rightIndexKnuckle3', u'rightMiddleKnuckle1', u'rightMiddleKnuckle2', u'rightMiddleKnuckle3', u'rightRingKnuckle1', u'rightRingKnuckle2', u'rightRingKnuckle3', u'rightPinkyKnuckle1', u'rightPinkyKnuckle2', u'rightPinkyKnuckle3', u'rightThumbKnuckle1', u'rightThumbKnuckle2', u'rightThumbKnuckle3',u'leftIndexKnuckle1', u'leftIndexKnuckle2', u'leftIndexKnuckle3', u'leftMiddleKnuckle1', u'leftMiddleKnuckle2', u'leftMiddleKnuckle3', u'leftRingKnuckle1', u'leftRingKnuckle2', u'leftRingKnuckle3', u'leftPinkyKnuckle1', u'leftPinkyKnuckle2', u'leftPinkyKnuckle3', u'leftThumbKnuckle1', u'leftThumbKnuckle2', u'leftThumbKnuckle3']

def format_angles(data):
    newdata=zeros(size(data))
    for k in range(len(data)):
        if data[k]>pi:
            newdata[k]=2*pi-data[k]
        else:
            newdata[k]=data[k]
    return newdata

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


def get_triggers(robot,dataset):
    #Assume that robot is in initial position now
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

    def load_scene(self,filename):
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
            if type(s.GetData()) is Sensor.Force6DSensorData:
                self.width+=6
                self.sensors.append(s)
        self.data=zeros((loglen,self.width))
        #Data structure is 1 col of time, 6s columns of sensor data

    def setup(self,histlen):
        for s in self.sensors:
            if type(s.GetData()) is Sensor.Force6DSensorData:
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
                return self.data[self.count,array(components)+1+6*k]

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
        pickle.dump(struct, f)

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
    #TODO" option parser

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
        robotfile = sys.argv[3]
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

    #TODO: GUI control?

    ladder=IULadderGenerator(file_param)
    laddername=ladder.kinbodyname
    envname=ladder.envname

    (env,options)=openhubo.setup(None,True)
    options.physics=True
    env.SetDebugLevel(3)

    # rlhuboplus models don't work with the affine transformations (i.e. use huboplus.robot.xml for ideal sim
    [robot,ctrl,ind,ref,recorder]=openhubo.load_scene(env,options)

    make_robot_transform(robot)

    #TODO: get rid of hand tweaks by processing initial location?

    #joint_offsets[ind('RSR')]+=pi/12
    #joint_offsets[ind('LSR')]+=-pi/12

    for n in fingers:
        if n.find('Thumb')<0:
            joint_signs[ind(n)]*=2
    print joint_signs

    # Use simulation to settle robot on the ground
    env.StartSimulation(openhubo.TIMESTEP,False)
    time.sleep(1)
    env.StopSimulation()

    iutraj=tr.IUTrajectory(robot,'iumapping.txt')
    ## Change this to affect maximum applied hand torque
    suffix='_{}'.format(Tmax)

    timestamp=openhubo.get_timestamp()
    recorder.filename='.'.join(laddername.split('.')[:-2])+timestamp+suffix+'_physics.avi'
    recorder.videoparams[0:2]=[800,600]
    traj=iutraj.to_openrave()
    ctrl.SetPath(traj)
    t_total=traj.GetDuration()

    steps=int(t_total/0.0005)
    forces=force_log(steps,[robot.GetAttachedSensor(x) for x in ['rightFootFT','leftFootFT']])
    points=effector_log(steps,[robot.GetLink(x) for x in ['leftFoot','rightFoot','leftPalm','rightPalm']])
    forces.setup(50)
    set_finger_torque(robot,.2)

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
    triggers=get_triggers(robot,iutraj.dataset)
    count=0
    #env.Load('kinbody/backrest.kinbody.xml')
    start=wait_start()
    if start:
        recorder.start()
    while not ctrl.IsDone() and start:
        env.StepSimulation(openhubo.TIMESTEP)
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

    tableentries=[outname,robotfile,'Expanded' if expand_limits>0 else 'Original',prefix,'{} {}'.format(count,int(number_of_steps*timestep/0.0005)),'success' if ctrl.IsDone() else 'failure']
    with open(logname,'w') as f:
        f.write(' '.join(tableentries) + '\n')
        f.write('Robot: ' + robotfile + '\n')
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
