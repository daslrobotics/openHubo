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

__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

from openravepy import *
from numpy import *
from openhubo import *

(env,options)=setup('qtcoin',True)
env.SetDebugLevel(4)

#Note that the load function now directly parses the option structure
options.physicsfile=True
[robot,ctrl,ind,ref,recorder]=load_scene(env,options)
env.StartSimulation(TIMESTEP)

#Change the pose to lift the elbows and send
ctrl.SendCommand('set radians ')
#0.7.1 Syntax change: Note the new "Pose" class:
pose=Pose(robot,ctrl)
pose['REP']=-pi/2
pose['LEP']=-pi/2
pose.send()

pause(2)

#Hack to get hand 
if robot.GetName() == 'rlhuboplus' or robot.GetName() == 'huboplus':
    ctrl.SendCommand('directtorque '+' '.join(['{}'.format(x) for x in range(42,57)]))
    for i in range(42,57):
        pose[i]=pi/2
    pose.send()
    pause(2)

    pose[42:57]=0
    pose.send()
