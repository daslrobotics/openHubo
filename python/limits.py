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
import tab

def checkCollision(robot,bodies):
    #Check the first body against the rest
    #Note that this IGNORES adjacent bodies if invoked this way. Therefore, we
    # can use it to directly inspect the model for joint body collisions.
    env=robot.GetEnv()
    print "Checking collisions for ",bodies[0]
    if len(bodies)>1:
        for body in bodies[1:]:
            print body
            print env.CheckCollision(robot.GetLink(body),robot.GetLink(bodies[0]))

def checklimits(robot,joint,checkbodies):
    env=robot.GetEnv()
    for k in range(0,180):
        #Go to a check position
        robot.SetDOFValues(float(k)*pi/180,joint.GetDOFIndex())
        for body in checkbodies:
            print k


if __name__=='__main__':
    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'huboplus/huboplus.robot.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)
    env.Load(file_env)

    with env:
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    time.sleep(1)

    checkCollision(robot,['Body_LHP','Body_LHR','Body_LHY'])





