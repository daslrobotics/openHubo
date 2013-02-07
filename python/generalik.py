#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'
from openravepy import CollisionReport
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time


class GeneralIK:
    def __init__(self,robot,problem,tsrlist=[],sample_bw=False):
        self.robot=robot
        self.problem=problem
        self.tsrlist=tsrlist
        self.sample_bw=sample_bw
        self.soln=[]
        self.zero=robot.GetDOFValues()    
        self.supportlinks=[]
        self.cogtarget=()
        self.return_closest=False
        self.gettime=False
    
    def Serialize(self,auto=False):
        L=len(self.tsrlist)
        cmd='DoGeneralIK nummanips {}'.format(L)
        for k in self.tsrlist:
            cmd=cmd+' '
            #Assumes that Bw encloses T0_e 
            if self.sample_bw:
                #TODO: save the sampled goal for future use
                cmd=cmd+'maniptm {} {}'.format(k.manipindex,SerializeTransform(k.sample()))
            else:
                T0_e=k.T0_w*k.Tw_e
                cmd=cmd+'maniptm {} {}'.format(k.manipindex,SerializeTransform(T0_e))
            #NOTE: "planning" robot treats torso pose as a manipulator, which 
            #clashes a bit with the idea that the torso should be specified by torsotm
        if len(self.supportlinks)>0:
            cmd=cmd+' supportlinks {} {}'.format(len(self.supportlinks),' '.join(self.supportlinks))
        if len(self.cogtarget)>0:
            cmd=cmd+' movecog {} {} {}'.format(self.cogtarget[0],self.cogtarget[1],self.cogtarget[2])
        if auto:
            cmd+=' exec'
        if self.return_closest:
            cmd+=' returnclosest'
        if self.gettime:
            cmd+=' gettime'

        cmd+=' returnsolved'
        return cmd
    def appendTSR(self,tsr):
        self.tsrlist.append(tsr)
    
    def activate(self,extra=[],extradof=[]):
        activedofs=[]
        manips=self.robot.GetManipulators()
        #Temporary copy 
        for m in self.tsrlist:
            #print m
            #print robot
            #print manips[m.manipindex].GetArmIndices()
            #print activedofs
            activedofs.extend(manips[m.manipindex].GetArmIndices())

        for m in extra:
            activedofs.extend(manips[m].GetArmIndices())
        for d in extradof:
            activedofs.append(d)
        #print self.activedofs
        self.robot.SetActiveDOFs(unique(activedofs))
        
    def run(self,auto=False,extra=[]):
        
        if auto:
            self.activate(extra)

        self.response=self.problem.SendCommand(self.Serialize(auto))
        datalist=self.response[:-3].split(' ')
        
        if len(datalist)>1:
            collisions=CollisionReport()
            self.soln=[float(x) for x in datalist]
            if self.response[-2]=='T':
                self.solvedflag=True
            else:
                self.solvedflag=False

            return True
            
    def goto(self):
        self.activate()
        self.robot.SetDOFValues(self.soln,robot.GetActiveDOFIndices())
        self.robot.WaitForController(.2)
    
    def solved(self):
        return len(self.soln)>0 and self.solvedflag
    
    def findSolution(self,itrs=10,auto=False,extra=[]):
        #Enable TSR sampling
        self.sample_bw=True
        for k in range(itrs):
            if self.solved():
                break
            #rezero to prevent getting stuck...at major speed penalty
            self.robot.SetDOFValues(self.zero)
            self.run(auto,extra)
            time.sleep(.1)
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
            time.sleep(.1)
        return self.solved()
    
    def resetZero(self):
        self.zero=self.robot.GetDOFValues()
    
if __name__ == '__main__':
    import openhubo
    from openravepy import *
    
    env=Environment()
    rtuple=openhubo.load(env,'rlhuboplus.robot.xml')
    robot=rtuple[0]
    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,robot.GetName())
    juiceTSR = TSR()
    juiceTSR.Tw_e = MakeTransform(rodrigues([pi/8, 0, 0]),mat([.1, 0.22, -05]).T)
    juiceTSR.Bw = mat([0, 0,   0, 0,   -0.02, 0.02,   0, 0,   0, 0,   -pi, pi])
    juiceTSR.manipindex = 0
    
    test=GeneralIK(robot,probs_cbirrt,[juiceTSR],False)
    test.return_closest=True

    print test.Serialize()
    test.run()
