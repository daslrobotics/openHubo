from openravepy import *
import time
import numpy

if __name__ == "__main__":
    
    try:
        
        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('forcePlate.env.xml') 
        
        robot = env.GetRobots()[0]
        forceSensor = robot.GetAttachedSensors()[0].GetSensor()
    
        force = 0
        torque = 0
        sumForce = 0
        sumTorque = 0
        sumForceBody = 0
        sumTorqueBody = 0
        with env:
            robot = env.GetRobots()[0]
            robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            env.StopSimulation()
            env.StartSimulation(timestep=0.0001 )

            velocities = numpy.zeros(robot.GetDOF())
            robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        for i in xrange(10000):
            data = forceSensor.GetSensorData()
            [force,torque] = env.GetPhysicsEngine().GetLinkForceTorque( robot.GetLink('Plate') )
            print data.force, data.torque, force, torque
            
        raw_input('')
    
    finally:
        env.Destroy()
