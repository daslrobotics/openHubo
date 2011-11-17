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


        for i in range(360):
            velocities = numpy.zeros(robot.GetDOF())+numpy.sin(i*numpy.pi/180)*.1
            robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
            time.sleep(.02)

        velocities = numpy.zeros(robot.GetDOF())
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        raw_input('')
    
    finally:
        env.Destroy()
