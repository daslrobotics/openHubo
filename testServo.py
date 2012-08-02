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
__author__ = 'Juan Gonzalez'
__copyright__ = 'Copyright (C) 2010 Juan Gonzalez (juan@iearobotics.com)'
__license__ = 'GPLv3 license'

#-- This is an example for testing the servocontroller located in the
#-- Modular Robot Open Rave plugin
#-- The two servos of the Minicube-I modular robot are both set to 45
#-- and -45 degrees.
#-- The Minicube-I modular robot is composed of two Y1 modules.


from openravepy import *
from numpy import *
import time
import sys

def trans(T,x,y,z):
    T[0,3]=x; T[1,3]=y; T[2,3]=z;
    return T

def run():

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(5)
    env.Load(file_env)

    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)

        #define ODE physics engine and set gravity
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    starttime = time.time()
    while True:
        #-- Set the servo 0 position to 45 degrees
        robot.GetController().SendCommand('setpos1 0 45 ')
        robot.GetController().SendCommand('setpos1 3 45 ')
        time.sleep(1.0)


        #-- Set the servos0 position to -45
        robot.GetController().SendCommand('setpos1 0 -45 ')
        robot.GetController().SendCommand('setpos1 3 -45 ')
        time.sleep(1.0)

if __name__=='__main__':
    run()


