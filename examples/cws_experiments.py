from numpy import zeros, array, pi, cos, sin, mat
import openhubo as oh
#import numpy as np
import time

from openhubo.cws import test_cws, make_average_initial_pose
from openhubo import cws

def test_ladder1(pose):
    #Get useful globals
    env = pose.robot.GetEnv()
    pose['LF1']=-pi/6
    pose['RF1']=-pi/6
    pose['RF2']=-pi/6
    pose['LSP']=-pi/2
    pose['RSP']=-pi/2
    pose['LHP']=-pi/4
    pose['RHP']=-pi/4
    pose['LKP']=pi/4
    pose['RKP']=pi/4
    pose['LAP']=-pi/4
    pose['RAP']=-pi/4
    pose.send(direct=True)
    oh.pause()
    env.StartSimulation(oh.TIMESTEP)
    oh.pause(2)
    T = make_average_initial_pose(pose)
    check=test_cws(robot)

    steps=50

    tol = 0.005
    upper = 1.0
    lower = 0.0
    test = (lower+upper)/2
    tests = []
    CWS_check = []
    real_stable = []
    min_dist = []
    while upper-lower > tol:
        env.StopSimulation()
        pose['LSP']=-test
        pose['RSP']=-test
        pose.take_init_pose()
        #FIXME this won't work if the ankle orientation changes (simulation could explode)
        pose.reset(True,True)
        t0=time.time()
        check.build_cws()
        CWS_check.append(check.check())
        t1=time.time()
        print "Time used is {0}".format(t1-t0)
        env.StartSimulation(oh.TIMESTEP)
        oh.pause(10)
        real_stable.append(not(check.fall(T)))
        min_dist.append(check.min_dist)
        print CWS_check[-1],real_stable[-1],min_dist[-1]
        if real_stable[-1] == 0:
            upper = test
        else:
            lower = test
        tests.append(test)
        test = (upper+lower)/2
    return CWS_check, real_stable, min_dist, tests

def test_forward_tilt(pose):
    #Get useful globals
    env = pose.robot.GetEnv()
    pose['LF1']=-pi/6
    pose['RF1']=-pi/6
    pose['RF2']=-pi/6
    pose['LSP']=-0.2
    pose['RSP']=-0.2
    leg_tilt=0.16
    pose['LHP']=leg_tilt
    pose['RHP']=leg_tilt
    pose['LAP']=-leg_tilt
    pose['RAP']=-leg_tilt
    pose.send(direct=True)
    env.StartSimulation(oh.TIMESTEP)
    oh.pause(2)
    T = make_average_initial_pose(pose)

    leg_force_scale = 2.1
    check=cws.ContactCheck(robot,.4)
    m = oh.find_mass(robot)
    g = 9.8
    check.insert_link('leftFoot', m * g * leg_force_scale)
    check.insert_link('rightFoot', m * g * leg_force_scale)
    tol = 0.002
    upper = 1.0
    lower = 0.0
    test = (lower+upper)/2
    tests = []
    CWS_check = []
    real_stable = []
    min_dist = []
    while upper-lower > tol:
        env.StopSimulation()
        pose['LSP']=-test
        pose['RSP']=-test
        pose.take_init_pose()
        #FIXME this won't work if the ankle orientation changes (simulation could explode)
        pose.reset(True,True)
        t0=time.time()
        check.build_cws()
        CWS_check.append(check.check())
        t1=time.time()
        print "Time used is {0}".format(t1-t0)
        env.StartSimulation(oh.TIMESTEP)
        oh.pause(10)
        real_stable.append(not(check.fall(T)))
        min_dist.append(check.min_dist)
        print CWS_check[-1],real_stable[-1],min_dist[-1]
        if real_stable[-1] == 0:
            upper = test
        else:
            lower = test
        tests.append(test)
        test = (upper+lower)/2
    return CWS_check, real_stable, min_dist, tests


if __name__ == '__main__':

    (env,options)=oh.setup('qtcoin',True)
    env.SetDebugLevel(4)
    #TODO fix hard name here
    options.robotfile='../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml'

    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

    pose=oh.Pose(robot)

    CWS_check, real_stable, min_dist, tests = test_forward_tilt(pose)


