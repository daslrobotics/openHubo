#!/usr/bin/env python
from openravepy import *
from servo import *
from numpy import pi
import re
import openhubo 
from recorder import viewerrecorder

def read_youngbum_traj(filename,robot,dt=.01,scale=1.0):
    """ Read in trajectory data stored in Youngbum's format (100Hz data):
        HPY LHY LHR ... RWP   (3-letter names)
        + - + ... +           (sign of joint about equivalent global axis + / -)
        0.0 5.0 2.0 ... -2.0  (Offset of joint from openHubo "zero" in YOUR sign convention)
        (data by row, single space separated)
    """
    #TODO: handle multiple spaces
    #Setup trajectory and source file
    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    ind=openhubo.makeNameToIndexConverter(robot)
    #Affine DOF are not controlled, so fill with zeros
    affinedof=zeros(7) 

    f=open(filename,'r')

    #Read in header row to find joint names
    header=f.readline().rstrip()
    print header.split(' ')

    indices=[ind(s) for s in header.split(' ')]

    #Read in sign row
    signlist=f.readline().rstrip().split(' ')
    signs=[]
    print signlist
    for s in signlist:
        if s == '+':
            signs.append(1)
        else:
            signs.append(-1)
    
    #Read in offset row (fill with zeros if not used)
    offsetlist=f.readline().rstrip().split(' ')
    print offsetlist
    offsets=[float(x) for x in offsetlist]

    k=0
    while True: 
        string=f.readline().rstrip()
        if len(string)==0:
            break
        jointvals=[float(x) for x in string.split(' ')]
        data=zeros(robot.GetDOF())

        for i in range(len(jointvals)):
            data[indices[i]]=(jointvals[i]+offsets[i])*pi/180.0*signs[i]*scale

        waypt=list(data)
        waypt.extend(affinedof)
        waypt.append(dt)
        traj.Insert(k,waypt)
        k=k+1

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
    return traj

if __name__=='__main__':

    try:
        traj_name = sys.argv[1]
    except IndexError:
        traj_name = 'trajectories/pump_reach.traj.txt'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.0005

    #-- Set the robot controller and start the simulation
    robot=openhubo.load_simplefloor(env)
    ind=openhubo.makeNameToIndexConverter(robot)
    controller=robot.GetController()

    env.StartSimulation(timestep=timestep)

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)
    traj=read_youngbum_traj(traj_name,robot,.015,.9)

    vidrec=viewerrecorder(env)
    controller.SetPath(traj)
    vidrec.start()
    controller.SendCommand('start')
    while not(controller.IsDone()):
        time.sleep(.1)
        print controller.GetTime()
    vidrec.stop()

