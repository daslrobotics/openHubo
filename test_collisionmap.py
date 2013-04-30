from openravepy import *
from numpy import *
from openhubo import *

env=Environment()
env.SetCollisionChecker(RaveCreateCollisionChecker(env,'ode'))
env.SetViewer('qtcoin')
env.Load('test_collisionmap.env.xml')

robot=env.GetRobots()[0]

pose=Pose(robot)
pose['LHP']=96*pi/180
pose['LHR']=10*pi/180
pose.send()
robot.CheckSelfCollision()
