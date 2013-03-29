"""Planning helper functions for openhubo.

Mostly out-of-date but still useful functions for working with openrave planners.
"""
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openravepy as _rave
import generalik as _ik
import cbirrt as _rrt
import TransformMatrix as _trans
import TSR as _tsr

from numpy.linalg import inv
from rodrigues import rodrigues
from openhubo import plot_projected_com, TIMESTEP
from openhubo.deprecated import CloseLeftHand,CloseRightHand
import time as _time

#TODO: rename functions to fit new style

def MakeInPlaceConstraint(robot,manipname):
    manips=robot.GetManipulators()
    manip=robot.GetManipulator(manipname)
    link=manip.GetEndEffector()
    T0_w=mat(link.GetTransform())
    Tw_e=mat(eye(4))
    for i in range(len(manips)):
        if manips[i].GetName()==manipname:
            manipindex=i
            break
    print manipindex

    tsr=_tsr.TSR(T0_w,Tw_e,mat(zeros(12)),manipindex)
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

    ctrl=PlayTrajWithPhysics(robot,traj,autoinit)

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
        oldctrl=robot.GetController()
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
            print "Real time {}, sim time {}".format(t,ctrl.GetTime())
            #Only approximate time here
            t=t+.1
            _time.sleep(.1)
            handle=plot_projected_com(robot)

    if resetafter:
        env.StopSimulation()
        ctrl.Reset()

    return ctrl

def plotTransforms(env,transforms,color=array([0,1,0])):
    #For now just plot points, eventually plot csys
    #TODO: lump points into a single vector instead of individual handles like this
    handles=[]
    for t in transforms:
        handles.append(env.plot3(points=t[:-1,3].T,pointsize=10.0,colors=color))
    return handles
 
def showGoal(env,T):     
    #TODO: Potential memory leak here?
    with env:
        dummy=_rave.RaveCreateKinBody(env,'')
        dummy.SetName('dummy_1')
        dummy.InitFromBoxes(numpy.array([[0,0,0,0.01,0.03,0.06]]),True)
        env.Add(dummy,True)
        dummy.SetTransform(array(T))
    _time.sleep(5)
    env.Remove(dummy)
    return 0
    
def supportTorsoPose(supports):
    #Calculate torso pose based on grip manips (i.e. best guess based on some parameters
    # Update the passed in reference with the Torso Manip
    t=mat([0,0,0]).T
    #Define ideal vertical offsets from each end effector
    numS=len(supports)
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
        t=t+(T[:3,-1]+mat([-.1,0,L[s]]).T)*w[s]
        wsum+=w[s]
        
    affineTSR=TSR(_trans.MakeTransform(mat(eye(3)),t/wsum))
    affineTSR.Bw=mat([-.05,.05,-.05,.05,-.1,.15,0,0,0,0,-pi/16,pi/16])
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
    return array([Cx,Cy])

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
            response=problem.SendCommand('CheckSupport supportlinks {} {}'.format(len(supportlinks),' '.join(supportlinks)))
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
            
        #init.setdefault('affineManip',supportTorsoPose(trans))
        #final.setdefault('affineManip',supportTorsoPose(trans))       

    init_ik=GeneralIK(robot,problem,init.values())
    planner=Cbirrt(problem)
    #Eliminate support links since CoG projection doesn't work on a ladder
    #init_ik.supportlinks=supportlinks
    #init_ik.cogtarget=(0.0,0,0)
    
    print init_ik.Serialize()
    init_ik.activate()
    init_ik.findSolution(50)
    
    if init_ik.solved():
        init_ik.goto()
        print "Found first transition solution"
        problem.SendCommand('CheckSupport supportlinks {} {}'.format(len(supportlinks),' '.join(supportlinks)))
    else:
        return False

    if len(final)>0:
        #final.setdefault('affineManip',supportTorsoPose(final))
        #final['affineManip'].Bw=mat([-.01,.01,-.01,.01,-.05,.1,0,0,0,0,0,0])
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
