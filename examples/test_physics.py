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

import openravepy as _rave
import openhubo as oh
import time
from numpy import pi,sqrt

(env,options)=oh.setup('qtcoin',True)
env.SetDebugLevel(4)

#Note that the load function now directly parses the option structure
if options.physics is None:
    options.physics=True

options.scenefile=None
[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

#Change the pose to lift the elbows and send
pose=oh.Pose(robot,ctrl)
pose['REP']=-pi/2
pose['LEP']=-pi/2
pose.send()

physics = env.GetPhysicsEngine()
physics.SetGravity([0,0,0])
ctrl2=_rave.RaveCreateController(env,"idealcontroller")
robot.SetController(ctrl2)

env.StartSimulation(timestep=oh.TIMESTEP)
initialPose=robot.GetDOFValues()
time.sleep(5)
finalPose=robot.GetDOFValues()
err=sqrt(sum(pow(initialPose-finalPose,2)))
