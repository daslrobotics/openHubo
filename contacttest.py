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
import sys
from servo import *
import openhubo 

DT=0.0005

def translate_body(body,t):
    T=body.GetTransform()
    T[0,3]+=t[0]
    T[1,3]+=t[1]
    T[2,3]+=t[2]
    print T
    env=body.GetEnv()
    with env:
        body.SetTransform(T)

def sinusoidal_rock1(robot,jointname,A,f,periods):
    omega=f*2*pi
    env=robot.GetEnv()
    LFz=[]
    RFz=[]
    LMx=[]
    RMx=[]
    LMy=[]
    RMy=[]
    pose=robot.GetDOFValues()
    t=arange(0,2*pi/omega*periods,DT)
    angle=A*(sin(omega*t))
    #Step simulation forward allow settling
    for k in range(int(2.0/DT)):
        env.StepSimulation(DT)
    for k in angle:
        pose[robot.GetJoint(jointname).GetDOFIndex()]=k
        robot.GetController().SetDesired(pose)
        env.StepSimulation(DT)
        force1=robot.GetAttachedSensor('leftFootFT').GetSensor().GetSensorData().force
        force2=robot.GetAttachedSensor('rightFootFT').GetSensor().GetSensorData().force
        torque1=robot.GetAttachedSensor('leftFootFT').GetSensor().GetSensorData().torque
        torque2=robot.GetAttachedSensor('rightFootFT').GetSensor().GetSensorData().torque
        LFz.append(force1[-1])
        RFz.append(force2[-1])
        LMx.append(torque1[0])
        RMx.append(torque2[0])
        LMy.append(torque1[1])
        RMy.append(torque2[1])
    data=[LMx,LMy,LFz,RMx,RMy,RFz]
    return data

def setup_sensors(robot):
    RFT=robot.GetAttachedSensor('rightFootFT').GetSensor()
    print RFT.Configure(Sensor.ConfigureCommand.PowerOff)
    time.sleep(.1)
    print RFT.SendCommand('histlen 200 ')>0
    time.sleep(.1)
    RFT.Configure(Sensor.ConfigureCommand.PowerOn)
    time.sleep(.1)

    LFT=robot.GetAttachedSensor('leftFootFT').GetSensor()
    time.sleep(.1)
    LFT.Configure(Sensor.ConfigureCommand.PowerOff)
    print LFT.SendCommand('histlen 200 ')
    time.sleep(.1)
    LFT.Configure(Sensor.ConfigureCommand.PowerOn)
    time.sleep(.1)
    robot.GetEnv().StopSimulation()

if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    time.sleep(.25)

    [robot,ctrl,ind,ref]=openhubo.load_rlhuboplus(env,'footcontact.env.xml',True)
    plate=env.GetKinBody('plate')
    translate_body(plate,[0,.5,0])
    setup_sensors(robot)
    data=sinusoidal_rock1(robot,'LAR',.03,.5,2)

    lcop_x=-array(data[1])/minimum(array(data[2]),1)
    lcop_y=-array(data[0])/minimum(array(data[2]),1)
   
    rcop_x=-array(data[4])/minimum(array(data[5]),1)
    rcop_y=-array(data[3])/minimum(array(data[5]),1)

    plt.plot(lcop_x,lcop_y,'+',rcop_x,rcop_y,'.')
    plt.show()
    #Change the pose to lift the elbows and resend
