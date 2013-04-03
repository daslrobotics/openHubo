"""Planning helper functions for openhubo.

Mostly out-of-date but still useful functions for working with openrave planners.
"""
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openravepy as _rave
import TSR as _tsr
import numpy as _np

from numpy import pi,mat
import copy as _copy
import TransformMatrix as _Trans

from openhubo import plot_projected_com, TIMESTEP
import time as _time

#TODO: rename functions to fit new style

def MakeInPlaceConstraint(robot,manipname):
    manips=robot.GetManipulators()
    manip=robot.GetManipulator(manipname)
    link=manip.GetEndEffector()
    T0_w=_np.mat(link.GetTransform())
    Tw_e=_np.mat(_np.eye(4))
    for i in range(len(manips)):
        if manips[i].GetName()==manipname:
            manipindex=i
            break
    print manipindex

    tsr=_tsr.TSR(T0_w,Tw_e,_np.mat(_np.zeros(12)),manipindex)
    print tsr.manipindex
    chain=_tsr.TSRChain(0,1,1)
    chain.insertTSR(tsr)
    return chain

def RunTrajectoryFromFile(robot,planner,autoinit=True):
    #TODO: Error checking
    f=open(planner.filename,'r')
    trajstring=f.read()
    f.close()

    traj=_rave.RaveCreateTrajectory(robot.GetEnv(),'')
    traj.deserialize(trajstring)

    _rave.planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    PlayTrajWithPhysics(robot,traj,autoinit)

def PlayTrajWithPhysics(robot,traj,autoinit=False,waitdone=True,resetafter=False):
    #TODO eliminate adding controller here
    #Lock the environment, halt simulation in preparation
    #import pdb
    #pdb.set_trace()

    env=robot.GetEnv()
    #May be a better way to do this without causing so much interruption
    env.StopSimulation()

    if robot.GetController().GetXMLId()!='trajectorycontroller':
        ctrl=_rave.RaveCreateController(env,'trajectorycontroller')
        #Not sure if this is necessary
        robot.SetController(ctrl)
    else:
        ctrl=robot.GetController()
    ctrl.SendCommand('set gains 100 0 16')
    ctrl.SetPath(traj)
    #Resets initial pose, so now we need to set any DOF not in trajectory
    if autoinit:
        setInitialPose(robot)
    ctrl.SetDesired(robot.GetDOFValues())
    #Eventuall make this variable? might not matter if .0005 is good
    env.StartSimulation(TIMESTEP)

    _time.sleep(1)
    ctrl.SendCommand('start')

    if waitdone:
        t=0
        while not(ctrl.IsDone()):
            print "Real time {}, sim time {}".for_np.mat(t,ctrl.GetTime())
            #Only approximate time here
            t=t+.1
            _time.sleep(.1)
            handle=plot_projected_com(robot)

    if resetafter:
        env.StopSimulation()
        ctrl.Reset()

    return ctrl

def plotTransforms(env,transforms,color=None):
    #For now just plot points, eventually plot csys
    #TODO: lump points into a single vector instead of individual handles like this
    if color is None:
        color=_np.array([0,1,0])
    handles=[]
    for t in transforms:
        handles.append(env.plot3(points=t[:-1,3].T,pointsize=10.0,colors=color))
    return handles

def showGoal(env,T):
    #TODO: Potential memory leak here?
    with env:
        dummy=_rave.RaveCreateKinBody(env,'')
        dummy.SetName('dummy_1')
        dummy.InitFromBoxes(_np([[0,0,0,0.01,0.03,0.06]]),True)
        env.Add(dummy,True)
        dummy.SetTransform(_np.array(T))
    _time.sleep(5)
    env.Remove(dummy)
    return 0

def supportTorsoPose(supports):
    #Calculate torso pose based on grip manips (i.e. best guess based on some parameters
    # Update the passed in reference with the Torso Manip
    t=_np.mat([0,0,0]).T
    #Define ideal vertical offsets from each end effector

    L={'leftArmManip':.1,'rightArmManip':.1,'leftFootManip':.75,'rightFootManip':.75}
    w={'leftArmManip':1,'rightArmManip':1,'leftFootManip':5,'rightFootManip':5}
    wsum=0
    for s in supports.keys():
        #TODO: remove expensive string comparison? maybe pre-subtract the affine manip
        #TODO: use centroid calculation instead of mean?
        if s == 'affineManip':
            continue
        T=supports[s].endPose()
        #print T[:3,-1]
        t=t+(T[:3,-1]+_np.mat([-.1,0,L[s]]).T)*w[s]
        wsum+=w[s]

    affineTSR=_tsr.TSR(_Trans.MakeTransform(_np.mat(_np.eye(3)),t/wsum))
    affineTSR.Bw=_np.mat([-.05,.05,-.05,.05,-.1,.15,0,0,0,0,-_np.pi/16,_np.pi/16])
    affineTSR.manipindex=4
    return affineTSR

def findCentroid(x,y):
    #Find area of polygon
    #x and y are arrays
    A=.5*sum(x[:-1]*y[1:]-x[1:]*y[:-1])
    #Cx
    Cx=1/(6*A)*sum((x[:-1]+x[1:])*(x[:-1]*y[1:]-x[1:]*y[:-1]))
    #Cy
    Cy=1/(6*A)*sum((y[:-1]+y[1:])*(x[:-1]*y[1:]-x[1:]*y[:-1]))
    return _np.array([Cx,Cy])

def findGraspFromGoal(goal):
    #TODO: eventually make this scan the environment etc. and do real grasp calculation
    #For now, figure out based on goal properties that are hand-assigned, then gradually roll back constraints.
    # Ideas for goal structure:
    #   suggested manips
    #   goal tm
    #   Bw / approach direction?
    pass

def getManipIndex(robot,manipName):
    manips=robot.GetManipulators()
    for k in range(len(manips)):
        if manips[k].GetName()==manipName:
            return k
    return -1


def setInitialPose(robot):
    #Define a manual initial pose for IK solution
    robot.SetActiveDOFs(robot.GetManipulator('leftFoot').GetArmIndices())
    robot.SetActiveDOFValues([0,0,-.2,.4,-.2,0])
    robot.SetActiveDOFs(robot.GetManipulator('rightFoot').GetArmIndices())
    robot.SetActiveDOFValues([0,0,-.2,.4,-.2,0])
    robot.SetActiveDOFs(robot.GetManipulator('leftArm').GetArmIndices())
    robot.SetActiveDOFValues([0,pi/4,0,-.5,0,0,0])
    robot.SetActiveDOFs(robot.GetManipulator('rightArm').GetArmIndices())
    robot.SetActiveDOFValues([0,-pi/4,0,-.5,0,0,0])
    #hack to close the fingers enough to avoid other body parts.
    #TODO: grasping routine?
    robot.SetDOFValues([pi/8,pi/8,pi/8],[robot.GetJointIndex('rightThumbKnuckle1'),robot.GetJointIndex('rightThumbKnuckle2'),robot.GetJointIndex('rightThumbKnuckle3')])
    robot.SetDOFValues([pi/8,pi/8,pi/8],[robot.GetJointIndex('leftThumbKnuckle1'),robot.GetJointIndex('leftThumbKnuckle2'),robot.GetJointIndex('leftThumbKnuckle3')])

def findPoseIntersection(robot,problem,init,final):
    trans={}
    for k in init.keys():
        for f in final.keys():
            if k==f and init[k]==final[f]:
                # Identical manipulator name and goal means overlap
                trans.setdefault(k,init[k])
    return trans

def solveWholeBodyPose(robot,problem,tsrs):
    """Useful GeneralIK wrapper to check if a goal combination is reasonable"""
    supportlinks=[]
    #find all supporting end effectors from manips
    for k in tsrs.keys():
        supportlinks.append(robot.GetManipulator(k).GetEndEffector().GetName())

    ik=GeneralIK(robot,problem,tsrs.values())

    #TODO: make this run even if solution is found?
    ik.continousSolve(1000,True,[3])

    if ik.solved():
        ik.goto()
        print "Found first transition solution"
        if len(ik.supportlinks):
            response=problem.SendCommand('CheckSupport supportlinks {} {}'.for_np.mat(len(supportlinks),' '.join(supportlinks)))
            collision=robot.GetEnv().CheckCollision(robot)
            if response[0]=='1' and not(collision):
                return True
        else:
            return True

    return False

def planSequence(robot,problem,init,final=[],trans=[]):
    #init is a list of tsr starts, including the affine pose
    if len(trans)>0:
        supportlinks=[]
        #find all supporting end effectors from manips
        for k in trans.keys():
            supportlinks.append(robot.GetManipulator(k).GetEndEffector().GetName())

    init_ik=GeneralIK(robot,problem,init.values())

    print init_ik.Serialize()
    init_ik.activate()
    init_ik.findSolution(50)

    if init_ik.solved():
        init_ik.goto()
        print "Found first transition solution"
        problem.SendCommand('CheckSupport supportlinks {} {}'.for_np.mat(len(supportlinks),' '.join(supportlinks)))
    else:
        return False

    if len(final)>0:
        #Look for transition pose AS SUBSET of init
        final_ik=GeneralIK(robot,problem,final.values())
        final_ik.supportlinks=supportlinks
        print final_ik.Serialize()
        final_ik.findSolution(50)
        if final_ik.solved() and init_ik.solved():
            print "Found second transition solution"
            final_ik.goto()
        else:
            return False

    return True


class Cbirrt:
    #TODO: Define copy constructor
    #TODO: figure out if this makes sense to pack the problem in this way
    def __init__(self, problem,tsr_chains=[],filename='cmovetraj.txt',timelimit=30,smoothing=10):
        self.problem=problem
        self.tsr_chains=tsr_chains
        self.filename=filename
        self.timelimit=timelimit
        self.smoothing=smoothing
        self.psample=.1
        self.supportlinks=[]
        self.jointgoals=[]
        self.jointstarts=[]
        self.solved=False
        self.exactsupport=False

    def insertTSRChain(self,chain):
        #TODO: Is it better to pass in by reference to make it easy to change?
        self.tsr_chains.append(_copy.deepcopy(chain))

    def Serialize(self):
        cmd='RunCBiRRT'
        cmd = cmd + ' filename {} timelimit {} smoothingitrs {}'.format(self.filename,self.timelimit,self.smoothing)
        goalSampling=False
        if len(self.jointgoals)>0:
            cmd=cmd+' jointgoals {} {}'.format(len(self.jointgoals),_Trans.Serialize1DMatrix(mat(self.jointgoals)))
        if len(self.jointstarts)>0:
            cmd=cmd+' jointstarts {} {}'.format(len(self.jointstarts),_Trans.Serialize1DMatrix(mat(self.jointstarts)))
        for chain in self.tsr_chains:
            cmd=cmd+'{}'.format(chain.Serialize())
            if chain.bSampleStartFromChain or chain.bSampleGoalFromChain:
                goalSampling=True
        if goalSampling:
            cmd=cmd+' psample {}'.format(self.psample)
        if len(self.supportlinks)>0:
            cmd=cmd+' supportlinks {} {}'.format(len(self.supportlinks),' '.join(self.supportlinks))
            cmd=cmd+' exactsupport {}'.format(int(self.exactsupport))

        #print cmd
        return cmd

    def run(self):
        stat = self.problem.SendCommand(self.Serialize())
        if stat == '1':
            self.solved=True
            return True
        return False

    def playback(self,force=False):
        if self.solved==True:
            return self.problem.SendCommand('traj {}'.format(self.filename))
        elif force:
            print 'Forcing playback of trajectory {}'.format(self.filename)
            return self.problem.SendCommand('traj {}'.format(self.filename))
        else:
            print 'Current solution not ready, use run() to generate'

    def ActivateManipsByIndex(self,robot,maniplist,extramanips=[]):
        manips=robot.GetManipulators()
        activedof=[]
        for i in maniplist:
            activedof.extend(manips[i].GetArmJoints().tolist())
        for i in extramanips:
            activedof.extend(manips[i].GetArmJoints().tolist())
        robot.SetActiveDOFs(activedof)
        return activedof



class GeneralIK:
    def __init__(self,robot,problem,tsrlist=[],sample_bw=False):
        self.robot=robot
        self.problem=problem
        self.tsrlist=tsrlist
        self.sample_bw=sample_bw
        self.soln=[]
        self.activedofs=[]
        self.zero=robot.GetDOFValues()
        self.supportlinks=[]
        self.cogtarget=()

    def Serialize(self):
        L=len(self.tsrlist)
        cmd='DoGeneralIK exec nummanips {}'.format(L)
        for k in self.tsrlist:
            cmd=cmd+' '
            #Assumes that Bw encloses T0_e
            if self.sample_bw:
                #TODO: save the sampled goal for future use
                cmd=cmd+'maniptm {} {}'.format(k.manipindex,_Trans.SerializeTransform(k.sample()))
            else:
                T0_e=k.T0_w*k.Tw_e
                cmd=cmd+'maniptm {} {}'.format(k.manipindex,_Trans.SerializeTransform(T0_e))
            #NOTE: "planning" robot treats torso pose as a manipulator, which
            #clashes a bit with the idea that the torso should be specified by torsotm
        if len(self.supportlinks)>0:
            cmd=cmd+' supportlinks {} {}'.format(len(self.supportlinks),' '.join(self.supportlinks))
        if len(self.cogtarget)>0:
            cmd=cmd+' movecog {} {} {}'.format(self.cogtarget[0],self.cogtarget[1],self.cogtarget[2])
        return cmd
    def appendTSR(self,tsr):
        self.tsrlist.append(tsr)

    def activate(self,extra=[]):
        if len(self.activedofs)==0:
            manips=self.robot.GetManipulators()
            #print manips
            for m in self.tsrlist:
                #print m
                #print robot
                #print manips[m.manipindex].GetArmIndices()
                #print activedofs
                self.activedofs.extend(manips[m.manipindex].GetArmIndices())
            for m in extra:
                self.activedofs.extend(manips[m].GetArmIndices())
        #print self.activedofs
        self.robot.SetActiveDOFs(self.activedofs)

    def run(self,auto=False,extra=[]):

        if auto:
            self.activate(extra)
        response=self.problem.SendCommand(self.Serialize())

        if len(response)>0:
            collisions=_rave.CollisionReport()
            if self.robot.CheckSelfCollision(collisions):
                print "Self-collision between links {} and {}!".format(collisions.plink1,collisions.plink2)
                return False
            if self.robot.GetEnv().CheckCollision(self.robot,collisions):
                print "Environment collision between links {} and {}!".format(collisions.plink1,collisions.plink2)
                return False
            self.soln=[float(x) for x in response[:-1].split(' ')]
            return True

    def goto(self):
        self.activate()
        self.robot.SetDOFValues(self.soln,self.activedofs)
        self.robot.WaitForController(.2)

    def solved(self):
        return len(self.soln)>0

    def findSolution(self,itrs=10,auto=False,extra=[]):
        #Enable TSR sampling
        self.sample_bw=True
        for k in range(itrs):
            if self.solved():
                break
            #rezero to prevent getting stuck...at major speed penalty
            self.robot.SetDOFValues(self.zero)
            self.run(auto,extra)
            _time.sleep(.1)
        return self.solved()

    def continousSolve(self,itrs=1000,auto=False,extra=[]):
        #Enable TSR sampling
        self.sample_bw=True
        for k in range(itrs):
            if not(self.robot.GetEnv().CheckCollision(self.robot)) and not(self.robot.CheckSelfCollision()) and self.solved():
                print "Valid solution found!"
                #TODO: log solutuon + affine DOF

            #rezero to prevent getting stuck...at major speed penalty
            self.robot.SetDOFValues(self.zero)
            self.run(auto,extra)
            _time.sleep(.1)
        return self.solved()

    def resetZero(self):
        self.zero=self.robot.GetDOFValues()

