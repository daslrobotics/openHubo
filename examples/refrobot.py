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

if __name__=='__main__':
    """ Demonstrate "from scratch" method of loading the huboplus and a "ref"
    version to show desired vs. actual pose of the servo controller. 
    """

    file_env = 'simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    # Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        env.SetCollisionChecker(collisionChecker)
        controller=RaveCreateController(env,'servocontroller')
        robot.SetController(controller)
        controller.SendCommand('setgains 100 0 16')

        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        
        #TODO: robot name tracking by URI doesn't work
        env.Load('rlhuboplus.ref.robot.xml')
        ref_robot=env.GetRobot('rlhuboplus_ref')
        ref_robot.Enable(False)
        ref_robot.SetController(RaveCreateController(env,'mimiccontroller'))
        controller.SendCommand("set visrobot rlhuboplus_ref")
        ind=openhubo.makeNameToIndexConverter(robot)
        set_robot_color(ref_robot,[.7,.7,.5],[.7,.7,.5],trans=.7)

    env.StartSimulation(timestep=0.0005)

    time.sleep(2)
    pose=zeros(robot.GetDOF())
    pose[ind('RSP')]=-pi/8
    controller.SetDesired(pose)
    time.sleep(2)
    pose[ind('RSP')]=0
    controller.SetDesired(pose)
