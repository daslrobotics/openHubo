#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":

    try:

        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('forcePlate.env.xml') 

        robot = env.GetRobots()[0]
        time.sleep(2)
        with env:
            robot = env.GetRobots()[0]
            #robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            
            time.sleep(1)
            
            collisionChecker = RaveCreateCollisionChecker(env,'ode')
            env.SetCollisionChecker(collisionChecker)

            env.StopSimulation()
            print env.GetViewer()
            env.StartSimulation(timestep=0.001 )

        time.sleep(4)

        raw_input('')

    finally:
        env.Destroy()
