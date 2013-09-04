"""Trajectory Control example for hubo+ model."""
import openhubo as oh
from openravepy import planningutils
from openhubo import trajectory
from numpy import pi,mat,array
from numpy.linalg import inv

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
        print rot
        pose.trans=array(rot)

    print pose.trans
    pose.reset()

    oh.sleep(waittime)
    print pose.robot.GetTransform()[2,3]
    if pose.robot.GetTransform()[2,3]<.8:
        return False
    else:
        return True
        #robot has fallen

def bisect_search(pose,jointmap,trans,tilttime=20,waittime=10):
    lower={k:0.0 for k in jointmap.keys()}
    upper=jointmap
    tol=1.0

    while tol>0.01:
        testmap={}
        for k,v in lower.items():
            testmap[k]=(v+upper[k])/2
        if run_quick_test(pose,testmap,trans,waittime):
            lower=testmap
        else:
            upper=testmap
        tol/=2
        print "Tol:",tol
        print "Lower:",lower
        print "Upper:",upper
    return lower

def reset_simulation(pose,trans):
    pose.reset()
    pose.robot.SetTransform(trans)
    env.StartSimulation(oh.TIMESTEP)

[env,options]=oh.setup()
env.SetDebugLevel(4)

options.physics=True
options.ghost=False

[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

#Initialize pose object and trajectory for robot
pose=oh.Pose(robot,ctrl)

env.StartSimulation(oh.TIMESTEP)

result=bisect_search(pose,{'LAP':-.3,'RAP':-.3},robot.GetTransform(),4,10)

