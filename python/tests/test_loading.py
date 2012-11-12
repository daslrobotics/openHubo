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
import unittest
import os, fnmatch

def find_files(directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

def model_test_factory(filename=None):
    class TestLoading(unittest.TestCase):
        def setUp(self):
            self.env=Environment()
            env=self.env
            env.SetDebugLevel(DebugLevel.Info)
            with env:
                env.StopSimulation()
                print('\nTesting model {}'.format(filename))
                result=env.Load(filename)
                self.assertTrue(result)
                self.robot=env.GetRobots()[0]
                physics = RaveCreatePhysicsEngine(env,'ode')
                physics.SetGravity([0,0,0])
                env.SetPhysicsEngine(physics)
                env.StartSimulation(timestep=0.001)

        def tearDown(self):
            with self.env:
                self.env.StopSimulation()
            self.env.Destroy()

        def test_loading(self):
            """Apply a physics engine and look for drift in joints. If joints
            move, then the physics engine is trying to resolve conflicting
            constraints.  This typically means that adjacent bodies are
            colliding and have not be declared to be officially adjacent.
            """
            initialPose=self.robot.GetDOFValues()
            time.sleep(2)
            finalPose=self.robot.GetDOFValues()
            err=sqrt(sum(pow(initialPose-finalPose,2)))
            #Somewhat arbitrary tolerance here
            self.assertLess(err,0.001)

        def test_zeroheight(self):
            """Make sure that the robot is correctly positioned so that it's
            lowest point is at or above the floor plane (XY plane).
            """
            aabb=self.robot.ComputeAABB()
            self.assertGreaterEqual(aabb.pos()[-1]-aabb.extents()[-1],0)


    return TestLoading

if __name__=='__main__':

    test1=model_test_factory('huboplus/huboplus.robot.xml')
    test2=model_test_factory('hubo2/hubo2.robot.xml')
    test3=model_test_factory('huboplus/rlhuboplus.robot.xml')
    test4=model_test_factory('hubo2/rlhubo2.robot.xml')
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())
