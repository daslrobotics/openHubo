#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

from openravepy import *
from numpy import *
import time
import datetime
import sys
import openhubo
import matplotlib.pyplot as plt

def build_ftmap(robot,linknames,ftdata):

    ftmap={}
    for n , ft in zip(linknames,ftdata):
        l=robot.GetLink(n).GetIndex()
        vec=[]
        vec.extend(ft[0])
        vec.extend(ft[1])
        ftmap.setdefault(l,array(vec))
    
    return ftmap

def map_joint_torques(robot,torques):
    m={}
    L=robot.GetLinks()
    for k in range(len(torques)):
        m.setdefault(L[k],torques[k])

    return m

if __name__=='__main__':

    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(3)
    
    #Load environment and robot with default settings
    [robot,ctrl,ind,ref_robot,recorder]=openhubo.load_scene(env,options.robotfile,options.scenefile,True)

    physics=env.GetPhysicsEngine()
    timestep=openhubo.TIMESTEP
    
    l1=robot.GetLink('leftFoot')
    l2=robot.GetLink('rightFoot')

    steps=100
    t0=time.time()
    st0=env.GetSimulationTime()

    #openhubo.pause()
    for k in range(steps):
        env.StepSimulation(timestep)
        FT1= physics.GetLinkForceTorque(l1)
        FT2= physics.GetLinkForceTorque(l2)

    print FT1
    st1=env.GetSimulationTime()
    t1=time.time()
    print "timestep = {}, Took {} real sec. for {} sim sec.".format(timestep,t1-t0,float(st1-st0)/1000000)

    ftmap=build_ftmap(robot,['leftFoot','rightFoot'],[FT1,FT2])
   
    #Shorthand for a vector of zero accelerations for the inverse dynamics function
    dofaccelerations=[]
    #Find static solution total torques:
    torques1 = robot.ComputeInverseDynamics(dofaccelerations,ftmap,False)

    #Find static solution torques by component (each DOF has a torque component due to M(qddot), C(q,qdot), G(q))
    torques2 = robot.ComputeInverseDynamics(dofaccelerations,ftmap,True)

    print map_joint_torques(robot,torques1)

