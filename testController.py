from openravepy import *
import time
import numpy

if __name__ == "__main__":

    try:

        env = Environment()
        env.SetViewer('qtcoin')
        env.Load('simpleFloor.env.xml') 

        robot = env.GetRobots()[0]

        with env:
            robot = env.GetRobots()[0]
            robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
            viewer = RaveCreateModule(env,'viewerrecorder')

            env.StopSimulation()
            print viewer.SendCommand('getcodecs')
            print env.GetViewer()
            env.StartSimulation(timestep=0.0001 )

        time.sleep(1)
        viewer.SendCommand('Start 640 480 10 codec 13 timing realtime viewer qtcoin\n filename mytestvid.mp4\n') 

        velocities = numpy.zeros(robot.GetDOF())
        print velocities
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        raw_input('')

    finally:
        env.Destroy()
