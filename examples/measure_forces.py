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
import tab
import openhubo
import matplotlib.pyplot as plt

def movingaverage(interval, window_size):
    window= ones(int(window_size))/float(window_size)
    return convolve(interval, window, 'same')

if __name__=='__main__':

    file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    #-- Set the robot controller and start the simulation
    with env:
        #NOTE: Make sure to use this sequence of commands WITHIN a "with env:"
        #block to ensure that the model loads correctly.
        env.StopSimulation()
        env.Load(file_env)
        robot = env.GetRobots()[0]

        #Define a joint name lookup closure for the robot
        ind=openhubo.makeNameToIndexConverter(robot)

        robot.SetDOFValues([pi/4,-pi/4],[ind('LSR'),ind('RSR')])
        pose=array(zeros(robot.GetDOF()))
        #TODO test with alternate ode solver
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        robot.GetController().SendCommand('setgains 50 0 7')
        #Note that you can specify the input format as either degrees or
        #radians, but the internal format is radians
        robot.GetController().SendCommand('set degrees')
        pose[ind('LSR')]=45
        pose[ind('RSR')]=-45
        robot.GetController().SetDesired(pose)

        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.

    l1=robot.GetLink('leftFoot')
    l2=robot.GetLink('rightFoot')
    physics=env.GetPhysicsEngine()
    LFz=[]
    RFz=[]
    LMx=[]
    RMx=[]
    LMy=[]
    RMy=[]
    steps=1000
    for k in range(steps):
        env.StepSimulation(timestep=0.0005)
        [force1,torque1]= physics.GetLinkForceTorque(l1)
        [force2,torque2]= physics.GetLinkForceTorque(l2)
        print force1[-1],force2[-1]
        LFz.append(force1[-1])
        RFz.append(force2[-1])
        LMx.append(torque1[0])
        RMx.append(torque2[0])
        LMy.append(torque1[1])
        RMy.append(torque2[1])
    
    t=[float(x)/steps for x in range(steps)]

    time.sleep(2)
    
    plt.plot(t,array(LFz)/openhubo.find_mass(robot)/9.8,'b',t,array(RFz)/openhubo.find_mass(robot)/9.8,'r')
    plt.grid(b=True)
    #plt.axis([t[0],t[-1],-500,0])
    plt.show()

    env.StartSimulation(0.0005)
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


