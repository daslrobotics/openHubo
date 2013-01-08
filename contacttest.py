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

def sinusoidal_rock(robot,A,f,periods,joint1,joint2=None,phase=0.0):
    omega=f*2*pi
    env=robot.GetEnv()
    pose=robot.GetDOFValues()
    tvec=arange(0,2*pi/omega*periods,DT)
    angle1=A*(sin(omega*tvec))
    if joint2:
        angle2=A*(sin(omega*tvec + phase))
    N=len(tvec)
    sensordata=array(zeros((6,N)))
    #Step simulation forward allow settling
    for k in range(int(2.0/DT)):
        env.StepSimulation(DT)
    for k in range(len(tvec)):
        pose[robot.GetJoint(joint1).GetDOFIndex()]=angle1[k]
        if joint2:
            pose[robot.GetJoint(joint2).GetDOFIndex()]=angle2[k]
        robot.GetController().SetDesired(pose)
        env.StepSimulation(DT)
        force1=robot.GetAttachedSensor('leftFootFT').GetSensor().GetSensorData().force
        force2=robot.GetAttachedSensor('rightFootFT').GetSensor().GetSensorData().force
        torque1=robot.GetAttachedSensor('leftFootFT').GetSensor().GetSensorData().torque
        torque2=robot.GetAttachedSensor('rightFootFT').GetSensor().GetSensorData().torque

        #Store data in array order LMx,LMy,LFz,RMx,RMy,RFz
        sensordata[0,k]=torque1[0]
        sensordata[1,k]=torque1[1] 
        sensordata[2,k]=force1[-1]
        sensordata[3,k]=torque1[0]
        sensordata[4,k]=torque1[1] 
        sensordata[5,k]=force2[-1]
    return sensordata

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

def export_hubo_traj(robot,A,f,periods,joint1,joint2=None,phase=0.0):
    
    omega=f*2*pi
    env=robot.GetEnv()
    pose=robot.GetDOFValues()
    tvec=arange(0,2*pi/omega*periods,DT)
    angle1=A*(sin(omega*tvec))
    if joint2:
        angle2=A*(sin(omega*tvec + phase))
    N=len(tvec)
    sensordata=array(zeros((6,N)))
    #Step simulation forward allow settling
    for k in range(len(tvec)):
        pose[robot.GetJoint(joint1).GetDOFIndex()]=angle1[k]
        if joint2:
            pose[robot.GetJoint(joint2).GetDOFIndex()]=angle2[k]

    #TODO export to a file following the hubo-read-trajectory format, or write an ach-output for the "youngbum" format

if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(3)
    time.sleep(.25)

    [robot,ctrl,ind,ref]=openhubo.load_rlhuboplus(env,'footcontact.env.xml',True)
    plate=env.GetKinBody('plate')
    translate_body(plate,[0,.5,0])
    setup_sensors(robot)
    data=sinusoidal_rock(robot,.03,.5,2,'LAP','LAR',pi/2)

    lcop_x=-data[1,:]/minimum(data[2,:],1)
    lcop_y=-data[0,:]/minimum(data[2,:],1)
   
    rcop_x=-data[4,:]/minimum(data[5,:],1)
    rcop_y=-data[3,:]/minimum(data[5,:],1)

    plt.plot(lcop_x,lcop_y,'+',rcop_x,rcop_y,'.')
    plt.show()
    #Change the pose to lift the elbows and resend
