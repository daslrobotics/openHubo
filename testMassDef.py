#!/usr/bin/env python
from openravepy import *
import time
import numpy
import tab

if __name__ == "__main__":
    
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('physicsSphere.env.xml') 
    
    with env:
        env.StopSimulation()
        env.StartSimulation(timestep=0.001 )
    
