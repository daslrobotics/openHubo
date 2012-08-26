#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":
    
    try:
        
        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('jaemiHubo.robot.xml') 
        time.sleep(1)
        
        physics = RaveCreatePhysicsEngine(env,'ode')
        physics.SetGravity([0,0,0])
        env.SetPhysicsEngine(physics)
        
        robot = env.GetRobots()[0]
        
        with env:
            robot = env.GetRobots()[0]
            robot.GetLinks()[0].SetStatic(True)
            robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            env.StopSimulation()
            env.StartSimulation(timestep=0.001 )

        
        w=0.5*2.0*numpy.pi
        T=1.0/w
        steps=25
        A=.1
        runtime=10

        for i in range(runtime*steps):
            t=float(i)/steps
            #print t
            velocities = numpy.zeros(robot.GetDOF())+numpy.sin(w*t)*A
            #print velocities
            robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
            time.sleep(1.0/float(steps))

        velocities = numpy.zeros(robot.GetDOF())
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        raw_input('')
    
    finally:
        env.Destroy()
