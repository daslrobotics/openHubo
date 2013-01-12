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

number_of_degrees=57
joint_offsets=zeros(number_of_degrees)
joint_signs=ones(number_of_degrees)
#Default the map to -1 for a missing index
jointmap=-ones(number_of_degrees,dtype=int)

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

def play_traj(robot,dataset,timestep):
    #Assume that robot is in initial position now
    T0=robot.GetTransform()
    for k in range(size(dataset,0)):
        Tc=eye(4)
        #use rodrigues function to build RPY rotation matrix
        Tc[0:3,0:3]=rodrigues(format_angles(dataset[k,3:6]))
        
        #grab translation from base 
        Tc[0:3,3]=dataset[k,0:3]
        #print Tc
        pose=dataset[k,jointmap+6]*joint_signs+joint_offsets
        #Note this method does not use a controller
        T=array(mat(T0)*mat(Tc))
        robot.GetController().SetDesired(pose.T,T) #account for initial offset
        time.sleep(timestep)

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
    T1[0:3,3]=array([0.0,.32+T_rung_global[1,3],.002]).T

    T=array(mat(T0)*mat(T1))
    robot.SetTransform(T)
    
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
    laddername=make_ladder(file_param)
    envname=make_ladder_env(file_param)

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)

    [robot,ctrl,ind,ref]=openhubo.load(env,'rlhuboplus.robot.xml',envname,True)
    
    make_robot_transform(robot)

    #TODO: get rid of hand tweaks by processing initial location?
    
    joint_offsets[ind('RSR')]=pi/12
    joint_offsets[ind('LSR')]=-pi/12

    #Read the file to obtain time steps and the total time

    env.StartSimulation(timestep=0.0005)
    theta=zeros(number_of_degrees)
    velocity=zeros(number_of_degrees)
    load_mapping(robot,"iumapping.txt")
    [dataset,timestep,total_time,number_of_steps]=load_iu_traj(file_traj)
    play_traj(robot,dataset,timestep)
