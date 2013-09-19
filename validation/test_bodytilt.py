"""Trajectory Control example for hubo+ model."""
import openhubo as oh
from openravepy import planningutils
from openhubo import trajectory
from numpy import pi,mat,array
from numpy.linalg import inv
import numpy as np


class TiltTester:

    def __init__(self,initial_pose,final_pose,ts=0.02,name=None,start_pose=None):
        """Assume that the robot starts from the home position (all zeros).
        Create a test trajectory and export it for the given initial and fial
        pose.
        """
        self.initial_pose=initial_pose
        self.robot=initial_pose.robot
        self.final_pose=final_pose
        [self.traj,__]=trajectory.create_trajectory(initial_pose.robot)
        self.basename=name
        self.ts=ts
        self.traj=None
        if start_pose is None:
            #Get the starting pose and trans from current pose
            self.start_pose=oh.Pose(robot)
            self.start_pose.update()

    def build_trajectory(self,pose_array=None):
        """create a trajectory to link poses based on times. Kindof kludgy wrapper
        to trajectory...
        """
        #this is ugly
        [self.traj,config]=trajectory.create_trajectory(self.robot)
        if pose_array is None:
            pose_array=[self.start_pose,self.initial_pose,self.final_pose]
        for p in pose_array:
            trajectory.traj_append(self.traj,p.to_waypt())

        planningutils.RetimeActiveDOFTrajectory(self.traj,self.robot,True)
        return self.traj

    def playback(self):
        """Show the trajectory in simulation"""
        self.reset_simulation()
        self.robot.GetController().SetPath(self.traj)
        try:
            self.robot.GetController().SendCommand('start')
        except:
            pass
        while not(self.robot.GetController().IsDone()):
            oh.sleep(.1)

    def reset_simulation(self,trans=None):

        pose=oh.Pose(self.robot)
        env=self.robot.GetEnv()
        env.StopSimulation()
        pose.reset()
        if trans:
            pose.robot.SetTransform(trans)
        env.StartSimulation(oh.TIMESTEP)

    def make_name(self,reverse=False):
        if reverse:
            suffix='_restore'
        else:
            suffix=''
        return '{}_tilt_{:0.0f}Hz{}.traj'.format(self.basename,1./self.ts,suffix)

    def export(self,build=False):
        if self.traj is None or build:
            self.build_trajectory()
        trajectory.write_hubo_traj(self.traj,self.robot,self.ts,self.make_name())

    def run_test(self,waittime,trans=None):
        """Using physics, run a test trajectory to see where the robot falls"""
        self.reset_simulation()
        ctrl=self.robot.GetController()
        ctrl.SetPath(self.traj)
        ctrl.SendCommand('start')
        while not(ctrl.IsDone()):
            oh.sleep(.2)

        oh.sleep(waittime)
        print self.robot.GetTransform()[2,3]
        if self.robot.GetTransform()[2,3]<.8:
            return False
        else:
            return True
            #robot has fallen

    def run_quick_test(self,trans,waittime=10):
        self.reset_simulation(trans)
        with self.robot:

            ftrans=self.robot.GetLink('leftFoot').GetTransform()
            rot=inv(mat(ftrans))*mat(trans)
            self.start_pose.trans=array(rot)

        self.start_pose.reset()

        angles=log_joint_angles('LAP',waittime,.01)
        #print pose.robot.GetTransform()[2,3]
        if self.robot.GetTransform()[2,3]<.8:
            return False,angles
        else:
            return True,angles
            #robot has fallen

    def bisect_search(self,jointmap,trans,waittime=10):
        lower={k:0.0 for k in jointmap.keys()}
        upper=jointmap
        tol=abs(np.max(jointmap.values()))

        while tol>0.002:
            testmap={}
            for k,v in lower.items():
                testmap[k]=(v+upper[k])/2
            flag,newangles = self.run_quick_test(testmap,trans,waittime)
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

    def log_joint_angles(self,joint,T,dt):
        angles=[]
        pose=self.initial_pose.copy()
        for n in xrange(int(T/dt)):
            pose.update()
            angles.append(pose[joint])
            oh.sleep(dt)
        return angles

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

if __name__ == '__main__':

    [env,options]=oh.setup()
    env.SetDebugLevel(4)

    #options.physics=True
    options.ghost=False

    [robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)

    #Initialize pose object and trajectory for robot
    pose0=oh.Pose(robot,ctrl)

    env.StartSimulation(oh.TIMESTEP)

    #result,angles = bisect_search(pose,{'LAP':-.3,'RAP':-.3},robot.GetTransform(),10)
    ts=0.02;

    ##Process for one trajectory dump:
    pose0.send()
    pose0.dt=0.01
    pose1=pose0.copy()
    pose1.dt=5
    pose1['LSR']=pi/6
    pose1['RSR']=-pi/6
    tilt=pi/6
    pose2=pose1.copy()
    pose2.dt=30
    pose2['LKP']+=tilt
    pose2['RKP']+=tilt
    pose2['LAP']-=tilt
    pose2['RAP']-=tilt
    tester=TiltTester(pose1,pose2,ts,'parallelogram-legs-knee-ankle-pitch')
    tester.export()
    if options._viewer:
        tester.playback()


