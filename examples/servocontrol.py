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

    (env,options)=openhubo.setup('qtcoin',True)
    env.SetDebugLevel(4)
    time.sleep(.25)

    [robot,ctrl,__,ref,recorder]=openhubo.load(env,options.robotfile,options.scenefile,True)
    time.sleep(.5)
    env.StartSimulation(openhubo.TIMESTEP)
    time.sleep(.5)
   
    #Change the pose to lift the elbows and send
    ctrl.SendCommand('set radians ')
    #0.7.1 Syntax change: Note the new "Pose" class:
    pose=Pose(robot,ctrl)
    pose['REP']=-pi/2
    pose['LEP']=-pi/2
    pose.send()

    openhubo.pause(2)

    #Hack to get hand 
    if robot.GetName() == 'rlhuboplus' or robot.GetName() == 'huboplus':
        ctrl.SendCommand('directtorque '+' '.join(['{}'.format(x) for x in range(42,57)]))
        for i in range(42,57):
            pose[i]=pi/2
        pose.send()
        openhubo.pause(2)

        pose[42:57]=0
        pose.send()
