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

if __name__=='__main__':

    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(3)
    
    #Load environment and robot with default settings
    [robot,ctrl,ind,ref_robot,recorder]=openhubo.load_scene(env,options.robotfile,options.scenefile,True)

    #l1=robot.GetJoint('LAR_dummy')
    #l2=robot.GetJoint('RAR_dummy')
    l1=robot.GetLink('leftFoot')
    l2=robot.GetLink('rightFoot')
    env.Load('physics.xml')
    physics=env.GetPhysicsEngine()
    steps=4000
    timestep=openhubo.TIMESTEP

    #Initialize numpy arrays to store data efficiently
    LFz=zeros(steps)
    RFz=zeros(steps)
    LMx=zeros(steps)
    RMx=zeros(steps)
    LMy=zeros(steps)
    RMy=zeros(steps)
    rsr=zeros(steps)
    rsrdof=ind('RSR')
    
    t=[float(x)*timestep for x in range(steps)]

    t0=time.time()
    st0=env.GetSimulationTime()
    for k in range(steps):
        env.StepSimulation(timestep)
        [force1,torque1]= physics.GetLinkForceTorque(l1)
        [force2,torque2]= physics.GetLinkForceTorque(l2)
        #print force1[-1],force2[-1]
        LFz[k]=force1[-1]
        RFz[k]=force2[-1]
        LMx[k]=torque1[0]
        RMx[k]=torque2[0]
        LMy[k]=torque1[1]
        RMy[k]=torque2[1]
        rsr[k]=robot.GetDOFVelocities([rsrdof])
    
    st1=env.GetSimulationTime()
    t1=time.time()
    print "timestep = {}, Took {} real sec. for {} sim sec.".format(timestep,t1-t0,float(st1-st0)/1000000)
    
    m=openhubo.find_mass(robot)
    skips=sum(abs(LFz[100:])/m/9.8<.07)+sum(abs(RFz[100:])<.1)
    jitter=std(rsr)
    print "Foot skips = {}".format(skips)
    print "RSR omega stdev  = {}".format(jitter)
    plt.plot(t,array(LFz)/m/9.8,'b',t,array(RFz)/m/9.8,'r')
    plt.grid(b=True)
    plt.title('Ratio of measured forces vs. body mass at each foot')
    plt.axis([t[0],t[-1],-500,0])
    plt.show()

    RFT=robot.GetAttachedSensor('rightFootFT').GetSensor()
    #show the history of measured forces
    print RFT.SendCommand('gethist')
    time.sleep(0.5)
    #Try to set history longer
    RFT.SendCommand('histlen 50')
    time.sleep(0.5)
    #It doesn't work because the sensor was running
    print RFT.SendCommand('gethist')

    #Proper way: power "down", change length, power "up"
    RFT.Configure(Sensor.ConfigureCommand.PowerOff)
    print RFT.SendCommand('histlen 50')
    RFT.Configure(Sensor.ConfigureCommand.PowerOn)
    time.sleep(0.25)
    print RFT.SendCommand('gethist')


