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
import openhubo

if __name__=='__main__':

    env = Environment()
    env.Add(RaveCreateViewer(env,'qtcoin'))
    env.SetDebugLevel(4)

    env.Load('physics.xml')

    [robot,ctrl,ind,ref_robot,recorder]=openhubo.load(env,'rlhuboplus.robot.xml','floor.env.xml',True)
        
    env.StartSimulation(openhubo.TIMESTEP)

    time.sleep(2)
    pose=zeros(robot.GetDOF())
    pose[ind('RSP')]=-pi/8
    ctrl.SetDesired(pose)
    time.sleep(2)
    pose[ind('RSP')]=0
    ctrl.SetDesired(pose)
