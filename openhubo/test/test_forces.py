from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import unittest
import openhubo
from numpy import mean, std,array,pi

class TestContactForces(unittest.TestCase):

    def setUp(self):
        (self.env,options)=openhubo.setup()
        self.env.SetDebugLevel(1)
        options.scenefile = 'forcetest.env.xml'
        options.robotfile = None
        options.physics='ode'
        self.robot,self.ctrl,self.ind,self.ghost,self.recorder=openhubo.load_scene(self.env,options)

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


