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
import unittest
import openhubo

class TestContactForces(unittest.TestCase):

    def setUp(self):
        env = Environment()
        file_env = 'forcetest.env.xml'
        env.SetDebugLevel(3)

        with env:
            env.StopSimulation()
            env.Load(file_env)
            self.robot = env.GetRobots()[0]
            collisionChecker = RaveCreateCollisionChecker(env,'pqp')
            env.SetCollisionChecker(collisionChecker)
        self.env=env

    def tearDown(self):
        self.env.StopSimulation()
        self.env.Destroy()

    def test_total_force(self):
        physics=self.env.GetPhysicsEngine()

        base=self.robot.GetLink('interface')
        l1=self.robot.GetLink('link1')
        l2=self.robot.GetLink('link2')
        box1=self.env.GetKinBody('box1').GetLink('')
        box2=self.env.GetKinBody('box2').GetLink('')
        sensor = self.robot.GetAttachedSensors()[0].GetSensor()
        
        Fz_raw=[]
        Fz_sense=[]
        for k in range(4000):
            self.env.StepSimulation(timestep=0.0005)
            [force,torque]= physics.GetLinkForceTorque(base)
            Fz_raw.append(force[-1])
            Fz_sense.append(sensor.GetSensorData().force[-1])
        
        errorMean=mean(array(Fz_sense)-array(Fz_raw))
        #Test that the average difference between raw values and sensed values
        #is not too great
        totalMass=l1.GetMass()+l2.GetMass()+box1.GetMass()+box2.GetMass()
        self.assertLess(abs(errorMean),1)
        self.assertLess(abs(mean(Fz_sense)/9.8+totalMass),1)
        self.assertLess(std(Fz_sense),std(Fz_raw))

if __name__=='__main__':
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())


