from openravepy import *
import time
import numpy

if __name__ == "__main__":
    
    try:
        
        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('physicsSphere.env.xml') 
        time.sleep(1)
        
        physics = RaveCreatePhysicsEngine(env,'ode')
        physics.SetGravity([0,0,-9.8])
        env.SetPhysicsEngine(physics)
        
        env.StopSimulation()
        env.StartSimulation(timestep=0.001 )

        raw_input('')
    
    finally:
        env.Destroy()
