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
        file_env = 'huboplus/huboplus.robot.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)
    env.Load(file_env)


    #-- Set the robot controller and start the simulation
    with env:
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    time.sleep(1)

    colrep1 = CollisionReport()

    #1) check at initial pose what links are reported to be colliding

    print robot.CheckSelfCollision(colrep1)
    
    RSR=robot.GetJoint('RSR').GetDOFIndex()
    LSR=robot.GetJoint('LSR').GetDOFIndex()
    robot.SetJointValues([.5, -.5 ],[LSR,RSR])

    colrep2 = CollisionReport()
    print robot.CheckSelfCollision(colrep2)

    RSR=robot.GetJoint('RSR').GetDOFIndex()
    LSR=robot.GetJoint('LSR').GetDOFIndex()
    robot.SetJointValues([0.0,0.0 ],[LSR,RSR])

    colrep3 = CollisionReport()
    print robot.CheckSelfCollision(colrep3)
    time.sleep(1)

    #The above 3 cases should all show no collisions, even though 2 of the 3 are.

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        robot = env.GetRobots()[0]
        collisionChecker2 = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker2)

        env.StartSimulation(timestep=0.001)

    colrep4 = CollisionReport()
    print robot.CheckSelfCollision(colrep4)
    
    RSR=robot.GetJoint('RSR').GetDOFIndex()
    LSR=robot.GetJoint('LSR').GetDOFIndex()
    robot.SetJointValues([.5, -.5 ],[LSR,RSR])

    colrep5 = CollisionReport()
    print robot.CheckSelfCollision(colrep5)

    RSR=robot.GetJoint('RSR').GetDOFIndex()
    LSR=robot.GetJoint('LSR').GetDOFIndex()
    robot.SetJointValues([0.0,0.0 ],[LSR,RSR])

    colrep6 = CollisionReport()
    print robot.CheckSelfCollision(colrep6)
    #Regardless of the initial pose, the ODE physics simulation will consider
    # any bodies not explicitly adjacent to be in collision:

    time.sleep(1)
    with env:
        env.StopSimulation()
        physics = RaveCreatePhysicsEngine(env,'ode')
        physics.SetGravity([0,0,0])
        env.SetPhysicsEngine(physics)

        env.StartSimulation(timestep=0.001)

    time.sleep(1)
    env.Destroy()
