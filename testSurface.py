#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":

    try:

        env = Environment()

        with env:
            collisionChecker = RaveCreateCollisionChecker(env,'ode')
            env.SetCollisionChecker(collisionChecker)
            env.SetViewer('qtcoin')
            env.SetDebugLevel(4)
            env.Load('surftest.env.xml') 
            collisionChecker = RaveCreateCollisionChecker(env,'bullet')
            env.SetCollisionChecker(collisionChecker)
            env.StopSimulation()
            robot = env.GetRobots()[0]
            env.StartSimulation(timestep=0.001 )

        raw_input('')

    finally:
        env.Destroy()
