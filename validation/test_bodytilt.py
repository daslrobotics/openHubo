"""Trajectory Control example for hubo+ model."""
import openhubo as oh
from openravepy import planningutils
from openhubo import trajectory
from numpy import pi,mat,array
from numpy.linalg import inv
import numpy as np

def run_test(pose,jointmap,trans,tilttime=20,waittime=10):
    [traj,config]=trajectory.create_trajectory(pose.robot)
    trajectory.traj_append(traj,pose.to_waypt(0.01))

    for j,v in jointmap.items():
        pose[j]=v

    trajectory.traj_append(traj,pose.to_waypt(tilttime))

    planningutils.RetimeActiveDOFTrajectory(traj,pose.robot,True)
    reset_simulation(pose,trans)

    pose.ctrl.SetPath(traj)
    pose.ctrl.SendCommand('start')
    while not(pose.ctrl.IsDone()):
        oh.sleep(.1)

    oh.sleep(waittime)
    print pose.robot.GetTransform()[2,3]
    if pose.robot.GetTransform()[2,3]<.8:
        return False
    else:
        return True
        #robot has fallen

def run_quick_test(pose,jointmap,trans,waittime=10):
    for j,v in jointmap.items():
        pose[j]=v

    pose.trans=trans
    with pose.robot:
        pose.reset()
        ftrans=pose.robot.GetLink('leftFoot').GetTransform()
        rot=inv(mat(ftrans))*mat(trans)
        pose.trans=array(rot)

    pose.reset()

    angles=log_joint_angles(pose,'LAP',waittime,.01)
    #print pose.robot.GetTransform()[2,3]
    if pose.robot.GetTransform()[2,3]<.8:
        return False,angles
    else:
        return True,angles
        #robot has fallen

def bisect_search(pose,jointmap,trans,waittime=10):
    lower={k:0.0 for k in jointmap.keys()}
    upper=jointmap
    tol=abs(np.max(jointmap.values()))

    while tol>0.002:
        testmap={}
        for k,v in lower.items():
            testmap[k]=(v+upper[k])/2
        flag,newangles = run_quick_test(pose,testmap,trans,waittime)
        if flag:
            lower=testmap
            angles=newangles
        else:
            upper=testmap
        print "Tol:",tol
        print "Lower:",lower
        print "Upper:",upper
        tol/=2

    return lower,angles

def reset_simulation(pose,trans):
    pose.reset()
    pose.robot.SetTransform(trans)
    env.StartSimulation(oh.TIMESTEP)

def set_test_state(pose,maxvel,maxtorque):
    """Set a given maxvel and maxtorque state for testing"""
    for j in pose.robot.GetJoints():
        j.SetVelocityLimits([maxvel])
        j.SetTorqueLimits([maxtorque])

def log_joint_angles(pose,joint,T,dt):

    angles=[]
    for n in xrange(int(T/dt)):
        pose.update()
        angles.append(pose[joint])
        oh.sleep(dt)
    return angles

[env,options]=oh.setup()
env.SetDebugLevel(4)

options.physics=True
options.ghost=False

[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

#Initialize pose object and trajectory for robot
pose=oh.Pose(robot,ctrl)

env.StartSimulation(oh.TIMESTEP)

result,angles =bisect_search(pose,{'LAP':-.3,'RAP':-.3},robot.GetTransform(),10)
