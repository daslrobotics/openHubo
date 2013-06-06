"""Trajectory Control example for hubo+ model."""
import openhubo as oh
from openravepy import planningutils
from openhubo import trajectory
from numpy import pi

[env,options]=oh.setup('qtcoin')
env.SetDebugLevel(4)

options.physics=True
options.ghost=True

[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
env.StartSimulation(oh.TIMESTEP)

#Initialize pose object and trajectory for robot
pose=oh.Pose(robot,ctrl)
[traj,config]=trajectory.create_trajectory(robot)

trajectory.traj_append(traj,pose.to_waypt(0.01))

pose['LAP']=-pi/8
pose['RAP']=-pi/8

pose['LKP']=pi/4
pose['RKP']=pi/4

pose['LHP']=-pi/8
pose['RHP']=-pi/8

pose['LSP']=-pi/8
pose['LEP']=-pi/4

trajectory.traj_append(traj,pose.to_waypt(1.0))

pose[:]=0.0

trajectory.traj_append(traj,pose.to_waypt(1.0))

planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

print "Showing samples of knee pose at given times:"
for k in range(40):
    data=traj.Sample(float(k)/10)
    print data[ind('LKP')]

ctrl.SetPath(traj)
ctrl.SendCommand('start')
while not(ctrl.IsDone()):
    oh.sleep(.1)
