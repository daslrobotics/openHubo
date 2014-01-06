"""Planning helper functions for openhubo.

Mostly out-of-date but still useful functions for working with openrave planners.
"""
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'



import openravepy as _rave
import numpy as _np
import comps as _comps
from openhubo import mapping

from numpy import pi

from openhubo import plot_projected_com
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

    tsr=_comps.TSR(T0_w,Tw_e,_np.mat(_np.zeros(12)),manipindex)
    print tsr.manipindex
    chain=_comps.TSRChain(0,1,1)
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
    #TODO eliminate adding controller here, potential issues switching back
    #Lock the environment, halt simulation in preparation

    env=robot.GetEnv()

    with env:
        if robot.GetController().GetXMLId()!='trajectorycontroller':
            ctrl=_rave.RaveCreateController(env,'trajectorycontroller')
            robot.SetController(ctrl)
        else:
            ctrl=robot.GetController()
        ctrl.SetPath(traj)
        #Resets initial pose, so now we need to set any DOF not in trajectory
        if autoinit:
            setInitialPose(robot)
        ctrl.SetDesired(robot.GetDOFValues())
        #Eventuall make this variable? might not matter if .0005 is good

    _time.sleep(1)
    ctrl.SendCommand('start')

    if waitdone:
        t0=_time.time()
        while not(ctrl.IsDone()):
            print "Real time {}, sim time {}".format(_time.time()-t0,ctrl.GetTime())
            #Only approximate time here
            _time.sleep(.1)
            handle=plot_projected_com(robot)

    if resetafter:
        with env:
            ctrl.Reset()

    return ctrl

def plotTransforms(env,transforms,color=None):
    #For now just plot points, eventually plot csys
    #TODO: lump points into a single vector instead of individual handles like this
    if color is None:
        color=_np.array([0,1,0])

    handle=[]
    points = _np.array([_np.squeeze(_np.array(T[0:3,3])) for T in transforms])
    handle=env.plot3(points,pointsize=10.0,colors=color)
    return handle

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

    affineTSR=_comps.TSR.TSR(_comps.make_transform(rot=None,trans=t/wsum))
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
    #Hack for DRCHubo
    fingers = mapping.get_fingers(robot);
    for f in fingers:
        robot.SetDOFValues([-.6],[f.GetDOFIndex()])


def findPoseIntersection(robot,problem,init,final):
    trans={}
    for k in init.keys():
        for f in final.keys():
            if k==f and init[k]==final[f]:
                # Identical manipulator name and goal means overlap
                trans.setdefault(k,init[k])
    return trans

def solveWholeBodyPose(robot,problem,tsrs,reps=1000):
    """Useful GeneralIK wrapper to check if a goal combination is reasonable"""
    supportlinks=[]
    #find all supporting end effectors from manips
    for k in tsrs.keys():
        supportlinks.append(robot.GetManipulator(k).GetEndEffector().GetName())

    ik=_comps.GeneralIK(robot,problem,tsrs.values())

    #TODO: make this run even if solution is found?
    ik.continuousSolve(reps,True,[3])

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


    init_ik=_comps.GeneralIK(robot,problem,init.values())

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
        #Look for transition pose AS SUBSET of init
        final_ik=_comps.GeneralIK(robot,problem,final.values())
        final_ik.supportlinks=supportlinks
        print final_ik.Serialize()
        final_ik.findSolution(50)
        if final_ik.solved() and init_ik.solved():
            print "Found second transition solution"
            final_ik.goto()
        else:
            return False

    return True


