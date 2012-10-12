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

if __name__=='__main__':

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetDebugLevel(4)
    env.Load(file_env)
    env.SetViewer('qtcoin')
    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        env.StartSimulation(timestep=0.0005)
    time.sleep(3)

    robot.GetController().SetDesired(zeros(60))
    LSR=robot.GetJoint('RSR').GetDOFIndex()
    LEP=robot.GetJoint('REP').GetDOFIndex()
    robot.GetController().SendCommand('setgains 50 0 7 .9998 .1')

    robot.GetController().SendCommand('setpos1 {} -50 '.format(LEP))
    time.sleep(1)
    robot.GetController().SendCommand('setpos1 {} -50 '.format(LSR))

