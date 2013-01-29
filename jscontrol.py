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
from TSR import *
from generalik import *

if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(4)
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
        ind=openhubo.makeNameToIndexConverter(robot)

        pose=zeros(robot.GetDOF())
        ##Very important to make sure the initial pose is not colliding
        robot.SetDOFValues(pose)

        #robot.SetController(RaveCreateController(env,'servocontroller'))
        #collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        #env.SetCollisionChecker(collisionChecker)
    
        #robot.GetController().SendCommand('setgains 100 0 8')
        #env.StartSimulation(timestep=0.0005)

    time.sleep(1)

    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,robot.GetName())

    robot.SetDOFValues([-1],[ind('LEP')])
    ik=GeneralIK(robot,probs_cbirrt)

    lh=robot.GetLink('leftPalm')
    Tstart=lh.GetTransform()
    la=robot.GetManipulator('leftArm')
    joints=la.GetArmIndices()
    lhgoal=TSR()
    lhgoal.manipindex=0
    ik.activate()
    ik.tsrlist.append(lhgoal)

    gains=array([0.012,0.012,0.012,pi/180,pi/180,pi/180])/100.
    deadzone=ones(6)*20
    spnav = sp.SpaceNav(env,deadzone,gains)
    while True:
        time.sleep(.1)
        T=lh.GetTransform()
        Tg=T.dot(spnav.get_transform())
        print Tg
        ik.tsrlist[0].Tw_e=Tg
        ik.run(True)

