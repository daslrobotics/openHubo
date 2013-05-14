#!/usr/bin/env python

#Basics
import openhubo
import time
from numpy import pi,zeros,ones,array,mat,eye
from openhubo.comps import rodrigues
from openhubo.IU import IULadderGenerator
import openhubo.trajectory as tr


#OpenRAVE and OpenHubo stuff
from openhubo import kbhit
import openravepy as rave

def make_robot_transform(robot):
    """hack function to get an initial pose that follows the IU ladder climbing
    planner. This function will vary as the initial pose is changed."""
    env=robot.GetEnv()
    ladder=env.GetKinBody('ladder')
    T_ladder=ladder.GetTransform()

    #3rd geom is bottom rung
    T_rung=ladder.GetLinks()[0].GetGeometries()[2].GetTransform()
    T_rung_global=mat(T_ladder)*mat(T_rung)
    #print T_rung_global
    #print T_rung_global[3,1]
    T0=robot.GetTransform()
    T1=eye(4)
    T1[0:3,0:3]=rodrigues([0,0,-pi/2])

    #shift back .32 from base
    T1[0:3,3]=array([-.118,.32+T_rung_global[1,3],0.002]).T

    T=array(mat(T0)*mat(T1))
    robot.SetTransform(T)
    return T

def set_default_limits(robot):
    """ Hack to enforce the stock limits on a version of the Hubo+ robot. Hard
    coded limits pulled from rlhuboplus model, based on shell collision limits
    from home position.
    """
    bases=['HP','KP','AP']
    lower=array([-85,-4,-74])*pi/180.0
    upper=array([92,149,97])*pi/180.0
    for p in ['R','L']:
        for k in range(len(bases)):
            n=p+bases[k]
            print n+ " old limits: {}".format(robot.GetJoint(n).GetLimits())
            robot.GetJoint(n).SetLimits([lower[k]],[upper[k]])

def set_expanded_limits(robot):
    """ Use 10 deg. more as shown by IU planner. """
    bases=['HP','KP','AP']
    lower=(array([-85,-4,-74])-10)*pi/180.0
    upper=(array([92,149,97])+10)*pi/180.0
    for p in ['R','L']:
        for k in range(len(bases)):
            n=p+bases[k]
            print n+ " old limits: {}".format(robot.GetJoint(n).GetLimits())
            robot.GetJoint(n).SetLimits([lower[k]],[upper[k]])

def add_torque(robot,joints,maxT,level=3):
    """ Add torque to joints, assuming that they are fingers by the indices."""
    if level>0:
        for j in joints[0::3][:-1]:
            j.AddTorque([maxT*.5])
        joints[-3].AddTorque([maxT*1.0])

    if level>1:
        for j in joints[1::3][:-1]:
            j.AddTorque([maxT*.25])
        joints[-2].AddTorque([maxT*.5])
    if level>2:
        for j in joints[2::3][:-1]:
            j.AddTorque([maxT*.125])
        joints[-1].AddTorque([maxT*.25])

def wait_start():
    print "Press Enter to abort, starting simulation in ... "
    for x in range(5,0,-1):
        print '{}...'.format(x)
        for k in range(10):
            time.sleep(.1)
            k=False
            if kbhit.kbhit():
                k=kbhit.getch()
                if k:
                    return False
    print "Starting simulation!"
    return True

if __name__=='__main__':
    #TODO option parser
    file_param = 'parameters/70_0.20.iuparam'
    file_traj = 'trajectories/70_0.20.iutraj'

    ladder=IULadderGenerator(file_param,True)
    scenefile=ladder.scenefile
    (env,options)=openhubo.setup('qtcoin',True)
    options.scenefile=scenefile
    env.SetDebugLevel(3)

    [robot,ctrl,ind,ref,recorder]=openhubo.load_scene(env,options)

    #Logging and recording setup
    timestamp=openhubo.get_timestamp()
    make_robot_transform(robot)

    iutraj=tr.IUTrajectory(robot,'iumapping-legacy.txt',file_traj)
    traj=iutraj.to_openrave(retime=True)
    t_total=traj.GetDuration()

    env.StartSimulation(openhubo.TIMESTEP)
    ctrl.SetPath(traj)

