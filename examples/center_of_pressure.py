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
from numpy.linalg import norm
import time
import sys
from servo import *
import openhubo 

def swing_and_measure(robot,pose):
    env=robot.GetEnv()
    robot.GetController().SetDesired(pose)
    maxF=300
    for k in range(5000):
        env.StepSimulation(openhubo.TIMESTEP)
        h=[]
        for s in robot.GetAttachedSensors():
            force=s.GetSensor().GetSensorData().force
            torque=s.GetSensor().GetSensorData().torque
            if s.GetName().find('Palm')>-1:
                if force[2]<1 and force[2]>-1:
                    CoPx=0
                    CoPz=0
                else:
                    CoPx=-torque[2]/force[1]
                    CoPz=-torque[0]/force[1]
                localCoP=[CoPx,0,CoPz,1]
                #ASSUME that COM of dummy end-effector body is "contact center"

            else:
                if force[2]<1 and force[2]>-1:
                    force[2]=sign(force[2])
                CoPx=-torque[1]/force[2]
                CoPy=-torque[0]/force[2]
                #ASSUME that COM of dummy end-effector body is "contact center"
                localCoP=[CoPx,CoPy,0,1]

            cop=s.GetAttachingLink().GetTransform()*mat(array(localCoP)).T
            r=norm(force)/maxF
            h.append(env.plot3(cop[:-1].T,10,[r,.5,0]))
            #h=openhubo.plot_masses(robot)
        
if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(3)
    (env,options)=openhubo.setup('qtcoin')

    [robot,controller,ind,ref,recorder]=openhubo.load_scene(env,options.robotfile,options.scenefile,True)

    with env:
        for s in robot.GetAttachedSensors():
            s.GetSensor().Configure(Sensor.ConfigureCommand.PowerOff)
            s.GetSensor().SendCommand('histlen 10')
            s.GetSensor().Configure(Sensor.ConfigureCommand.PowerOn)
        controller.SendCommand('set radians ')

    #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
    env.StartSimulation(openhubo.TIMESTEP)
    time.sleep(1)
    env.StopSimulation()

    pose=zeros(robot.GetDOF())
    pose[ind('REP')]=-pi/4
    pose[ind('LEP')]=-pi/4
    pose[ind('RSP')]=-pi/4
    pose[ind('LSP')]=-pi/4

    swing_and_measure(robot,pose)

    pose[ind('REP')]=0
    pose[ind('LEP')]=0
    pose[ind('RSP')]=0
    pose[ind('LSP')]=0

    swing_and_measure(robot,pose)

    env.StartSimulation(openhubo.TIMESTEP)
