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

if __name__=='__main__':

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetDebugLevel(4)
    env.SetViewer('qtcoin')
    time.sleep(.25)
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
        pose=array(zeros(60))
        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

    
        robot.GetController().SendCommand('setgains 50 0 7 .9998 .1')
        #Note that you can specify the input format as either degrees or
        #radians, but the internal format is radians
        robot.GetController().SendCommand('set degrees')
        pose[ind('LSR')]=45
        pose[ind('RSR')]=-45
        robot.GetController().SetDesired(pose)

        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        env.StartSimulation(timestep=0.0005)

    time.sleep(3)
   
    #Change the pose to lift the elbows and resend
    pose[ind('REP')]=-pi/8
    pose[ind('LEP')]=-pi/8

    robot.GetController().SetDesired(pose)

