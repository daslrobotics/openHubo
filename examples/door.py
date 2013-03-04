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

#Get the global environment for simulation

if __name__=='__main__':
    
    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(4)

    [robot,ctrl,ind,ref,recorder]=openhubo.load(env,options.robotfile,['door.env.xml'],True)
    door=env.GetRobot('doorbot')
    door.SetController(RaveCreateController(env,'servocontroller'))
    spring=door.GetController()
    with env:
        spring.SendCommand('springdamper 0 1')
        spring.SendCommand('set gainvec   0 10 0 1   1 10 0 0 ')
    time.sleep(0.1)
    env.StartSimulation(openhubo.TIMESTEP)


