#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":
    
    try:
        
        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('forcePlate.env.xml') 
        
        physics = RaveCreatePhysicsEngine(env,'ode')
        physics.SetGravity([0,0,-9.8])
        env.SetPhysicsEngine(physics)

        #The first robot defined in the XML file is Jaemi Hubo
        robot = env.GetRobots()[0]
        #The second "robot" is the force plate, which is grossly oversized to
        #prevent weird collision issues, like the plates completely intersecting
        #each other.
        plate = env.GetRobots()[1]
        forceSensor = robot.GetAttachedSensors()[0].GetSensor()
    
        force = 0
        torque = 0
        sumForce = 0
        sumTorque = 0
        sumForceBody = 0
        sumTorqueBody = 0
        with env:
            robot = env.GetRobots()[0]
            #robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            robot.SetController(RaveCreateController(env,'servocontroller'))
            collisionChecker = RaveCreateCollisionChecker(env,'bullet')
            env.SetCollisionChecker(collisionChecker)

            env.StopSimulation()
            env.StartSimulation(timestep=0.001 )

            robot.GetController().SendCommand('setpos1 0 20 ')

        #velocities = numpy.zeros(robot.GetDOF())
        #robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        for i in xrange(100):
            time.sleep(.1)
            data = forceSensor.GetSensorData()
            [force,torque] = env.GetPhysicsEngine().GetLinkForceTorque( plate.GetLink('Plate') )
            print data.force, data.torque, force, torque
            
        raw_input('')
    
    finally:
        env.Destroy()
