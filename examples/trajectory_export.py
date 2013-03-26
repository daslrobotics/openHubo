#!/usr/bin/env python

import openhubo
from openravepy import *
from numpy import *
from numpy.linalg import *
import sys
import time
from copy import copy
import openhubo
from trajectory import *
#TODO: Work with the concept of activeDOF?

def createTrajectory(robot):
    """ Create a trajectory based on a robot's config spec"""
    traj=RaveCreateTrajectory(robot.GetEnvironment,'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    return traj

""" Simple test script to run some of the functions above. """
if __name__=='__main__':

    env = Environment()
    (env,options)=openhubo.setup()
    env.SetDebugLevel(3)

    timestep=openhubo.TIMESTEP

    [robot,controller,ind,ref,recorder]=openhubo.load(env,options)

    pose0=Pose(robot,controller)
    pose0.update()
    pose0.send()

    env.StartSimulation(timestep=timestep)

    pose1=Pose(robot,controller)

    pose1['LAP']=-pi/8
    pose1['RAP']=-pi/8

    pose1['LKP']=pi/4
    pose1['RKP']=pi/4

    pose1['LHP']=-pi/8
    pose1['RHP']=-pi/8

    traj=RaveCreateTrajectory(env,'')

    #Set up basic parameters
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()

    traj.Init(config)

    #Note the new waypoint-building syntax
    traj.Insert(0,pose0.to_waypt(dt=0.0))
    traj.Insert(1,pose1.to_waypt(dt=1.0))
    traj.Insert(2,pose0.to_waypt(dt=1.0))

    #Need to do this to add timing information for interpolating
    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
   
    print 'Dump all DOFs to youngbum format'
    write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum.traj')

    print 'Only use a selection of DOF\'s instead of everything'
    write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum2.traj',dofs=range(28))
    
    print 'Write to hubo-read-trajectory compatible format'
    write_hubo_traj(traj,robot,0.025,'traj_example_hubo.traj')

