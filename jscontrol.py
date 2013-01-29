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
from numpy.linalg import pinv
import time
import datetime
import sys
import tab
import spacenav as sp
import openhubo

if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(3)
    env.SetViewer('qtcoin')
    time.sleep(.25)


    with env:
        #NOTE: Make sure to use this sequence of commands WITHIN a "with env:"
        #block to ensure that the model loads correctly.
        #env.StopSimulation()
        #env.Load('simpleFloor.env.xml')
        env.Load('huboplus.robot.xml')
        robot = env.GetRobots()[0]

        ##Define a joint name lookup closure for the robot
        #ind=openhubo.makeNameToIndexConverter(robot)

        pose=zeros(robot.GetDOF())
        ##Very important to make sure the initial pose is not colliding
        robot.SetDOFValues(pose)

        #robot.SetController(RaveCreateController(env,'servocontroller'))
        #collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        #env.SetCollisionChecker(collisionChecker)
    
        #robot.GetController().SendCommand('setgains 100 0 8')
        #env.StartSimulation(timestep=0.0005)

    time.sleep(1)
   
    #robot.GetController().SendCommand('set radians')
    #pose[ind('REP')]=-pi/2
    #pose[ind('LEP')]=-pi/2
    #pose[ind('LHY')]=pi/2

    lh=robot.GetLink('leftPalm')
    Tstart=lh.GetTransform()
    la=robot.GetManipulator('leftArm')
    joints=la.GetArmIndices()
    robot.SetActiveDOFs(joints)

    gains=array([0.012,0.012,0.012,pi/180,pi/180,pi/180])/10.
    deadzone=ones(6)*20
    spnav = sp.SpaceNav(env,deadzone,gains)
    while True:
        T=lh.GetTransform()
        Jt=mat(robot.CalculateActiveJacobian(lh.GetIndex(),T[0:3,3]))
        Jr=mat(robot.CalculateActiveRotationJacobian(lh.GetIndex(),quatFromRotationMatrix(T[0:3,0:3])))
        q=robot.GetActiveDOFValues()
        time.sleep(.1)
        aff=spnav.get_affine()
        t=spnav.get_translation()
        dqr=pinv(Jr)*aff/100
        dqt=pinv(Jt)*t/100
        q1=array(squeeze(dqt)+q)[0]
        robot.SetActiveDOFValues(q1)
        print q1

