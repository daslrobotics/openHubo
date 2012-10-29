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

if __name__=='__main__':

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'scenes/forcetest.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    #-- Set the robot controller and start the simulation
    with env:
        env.Load(file_env)
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        env.StartSimulation(timestep=0.0005)

    l1=robot.GetLink('base')
    l2=robot.GetLink('middle')
    l3=robot.GetLink('top')
    physics=env.GetPhysicsEngine()
    box1=env.GetKinBody('box1').GetLink('')
    box2=env.GetKinBody('box2').GetLink('')

    for k in range(5000):
        [force1,torque1]= physics.GetLinkForceTorque(l1)
        [force2,torque2]= physics.GetLinkForceTorque(l2)
        [force3,torque3]= physics.GetLinkForceTorque(l3)
        [force4,torque4]= physics.GetLinkForceTorque(box1)
        [force5,torque5]= physics.GetLinkForceTorque(box2)
        print force1[-1],force2[-1],force3[-1],force4[-1],force5[-1]

