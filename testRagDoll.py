#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":

    try:

        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('forcePlate.env.xml') 

        with env:
            robot = env.GetRobots()[0]
            
            collisionChecker = RaveCreateCollisionChecker(env,'bullet')
            env.SetCollisionChecker(collisionChecker)

            env.StopSimulation()
            env.StartSimulation(timestep=0.001 )


        raw_input('')

    finally:
        env.Destroy()
