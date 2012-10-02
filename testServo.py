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

""" Demonstrate the range of motion of a joint by attempting to reach +/- 180
deg"""
def testMotionRange(robot,jointName,steps=50):
    joint=robot.GetJoint(jointName)
    dq=360.0
    q0=-180.0

    for k in [x*dq/steps+q0 for x in range(steps+1)]:
        print k
        robot.GetController().SendCommand('setpos1 {} {}'.format(joint.GetDOFIndex(),k))
        time.sleep(.01)


if __name__=='__main__':
    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)
    env.Load(file_env)


    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        env.StartSimulation(timestep=0.0005)

    time.sleep(2)

    #Begin experimental sandbox here:
    robot.GetController().SendCommand('setgains 50 1 5 .1 .1')
    testMotionRange(robot,'LSR')
