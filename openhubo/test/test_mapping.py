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

import openravepy as rave
from numpy import pi,zeros,sqrt
import unittest
import openhubo as oh
import numpy as np
from openhubo import trajectory,mapping


class TestMapping(unittest.TestCase):
    def setUp(self):

        (env,options)=oh.setup()
        options.ghost=False
        options.physics=False
        options.stop=False
        (self.robot,self.ctrl,ind,ghost,recorder)=oh.load_scene(env,options)
        env.SetDebugLevel(1)
        self.env=env

    def test_hubo_read_trajectory(self):
        robot=self.robot
        pose=oh.Pose(robot)
        pose.update(mapping.create_random_bounded_pose(robot))
        traj,config=trajectory.create_trajectory(robot,pose.to_waypt())
        print traj
        trajectory.write_hubo_traj(traj,robot,.04)
        #TODO: run trajectory with hubo-ach and check that current pose is ok


if __name__=='__main__':

    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())

