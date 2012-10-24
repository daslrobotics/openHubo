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

import tab
from openravepy import *
from numpy import *
import time
import datetime
import sys
from servo import *
import openhubo

if __name__=='__main__':

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)
    timestep=0.0005

    with env:
        #Since physics are defined within the XML file, stop simulation
        env.StopSimulation()
        env.Load('simpleFloor.env.xml')
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = openhubo.makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'servocontroller')
        robot.SetController(controller)

        controller.SendCommand('set gains 50 0 8')

        #Set an initial pose before the simulation starts
        pose=array(zeros(robot.GetDOF()))

        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        #Set initial pose to avoid thumb collisions
        robot.SetDOFValues(pose)
        controller.SetDesired(pose)


    t=array([float(k)/100.0 for k in range(500)])
    A=20.0
    traj=sin(0.250*2.0*pi*t)*A

    controller.SendCommand('set degrees')
    initPose={'LSR':18,'RSR':-18,'LEP':-20,'REP':-20}

    time.sleep(1)
    env.StartSimulation(timestep=timestep)

    sendSparseServoCommand(robot,initPose)
    time.sleep(3)

    sendSingleJointTrajectorySim(robot,traj,robot.GetJoint('HPY').GetDOFIndex(),timestep,100)
    
