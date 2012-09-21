#!/usr/bin/env python
from openravepy import *
import time
import numpy

if __name__ == "__main__":
    
    try:
        
        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('physicsSphere.env.xml') 
        
        env.StopSimulation()
        env.StartSimulation(timestep=0.001 )

        raw_input('')
    
    finally:
        env.Destroy()
