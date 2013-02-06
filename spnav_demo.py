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
import spacenav as sp
import openhubo

if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(3)
    env.SetViewer('qtcoin')
    time.sleep(.25)

    with env:
        env.Load('rotbox.env.xml')
        box=env.GetKinBody('heavybox')

    time.sleep(1)
   
    gains=array([0.002,0.002,0.002,pi/180,pi/180,pi/180])/100.
    deadzone=ones(6)*15
    spnav = sp.SpaceNav(env,deadzone,gains)
    while True:
        T=box.GetTransform()
        
        time.sleep(.1)
        box.SetTransform(T.dot(spnav.get_transform()))
        print spnav.get_state()
