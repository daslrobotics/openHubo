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
import threading
import time
import unittest
import os, fnmatch

def find_files(directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

class TestModelLoading(unittest.TestCase):

    def setUp(self):
        self.env=Environment()
        self.env.SetDebugLevel(4)

    def tearDown(self):
        self.env.Destroy()
        RaveDestroy()

    def test_load(self):
        self.env.StopSimulation()
        result=self.env.Load('huboplus/huboplus.robot.xml')
        self.assertTrue(result)

    def test_physics_drift(self):
        """Apply a physics engine and look for joint drift after some number of
        seconds (no gravity). This means that the physics engine is trying to
        solve an initial collision, causing self-motion."""
        result=self.env.Load('huboplus/huboplus.robot.xml')
        env=self.env
        robot=env.GetRobots()[0]
        initialPose=robot.GetDOFValues()
        with env:
            env.StopSimulation()
            physics = RaveCreatePhysicsEngine(env,'ode')
            physics.SetGravity([0,0,0])
            env.SetPhysicsEngine(physics)
            env.StartSimulation(timestep=0.001)

        time.sleep(2)
        with env:
            env.StopSimulation()
        finalPose=robot.GetDOFValues()
        err=sum(sqrt(pow(initialPose,2)+pow(finalPose,2)))
        self.assertTrue(err>0.001)

    def test_loadrobots(self):
        count=0
        with self.env:
            for filename in find_files('./', '*.robot.xml'):
                print 'Found Robot file', filename
                count=count+1
                self.env.Reset()
                result=self.env.Load(filename)
                self.assertTrue(result)

if __name__=='__main__':
    unittest.main()
