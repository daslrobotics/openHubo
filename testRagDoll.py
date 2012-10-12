#!/usr/bin/env python
from openravepy import *
import time
import numpy
import tab

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('scenes/simpleFloor.env.xml') 

    with env:
        robot = env.GetRobots()[0]
        
        collisionChecker = RaveCreateCollisionChecker(env,'bullet')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=0.0005 )


