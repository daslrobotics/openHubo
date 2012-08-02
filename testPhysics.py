from openravepy import *
import time
import numpy

if __name__ == "__main__":

    try:

        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('forcePlate.env.xml') 

        robot = env.GetRobots()[0]

        with env:
            robot = env.GetRobots()[0]
            #robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            time.sleep(1)
            
            collisionChecker = RaveCreateCollisionChecker(env,'ode')
            env.SetCollisionChecker(collisionChecker)

            #define ODE physics engine and set gravity
            physics = RaveCreatePhysicsEngine(env,'ode')
            physics.SetGravity([0,0,-9.8])
            env.SetPhysicsEngine(physics)

            env.StopSimulation()
            print env.GetViewer()
            env.StartSimulation(timestep=0.001 )

        time.sleep(1)

        raw_input('')

    finally:
        env.Destroy()
