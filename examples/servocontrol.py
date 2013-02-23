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

    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,options.scenefile,True)
    time.sleep(.5)
    env.StartSimulation(openhubo.TIMESTEP)
    time.sleep(.5)
   
    #Change the pose to lift the elbows and send
    ctrl.SendCommand('set radians ')
    pose=robot.GetDOFValues()
    pose[ind('REP')]=-pi/2
    pose[ind('LEP')]=-pi/2
    ctrl.SetDesired(pose)

    openhubo.pause(2)

    ctrl.SendCommand('openloop '+' '.join(['{}'.format(x) for x in range(42,57)]))
    for i in range(42,57):
        pose[i]=pi/2
    ctrl.SetDesired(pose)
    openhubo.pause(2)

    pose[42:57]=0
    ctrl.SetDesired(pose)
