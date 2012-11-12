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
            collisions=CollisionReport()
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
    
    juiceTSR = TSR()
    juiceTSR.Tw_e = MakeTransform(rodrigues([pi/2, 0, 0]),mat([0, 0.22, 0.1]).T)
    juiceTSR.Bw = mat([0, 0,   0, 0,   -0.02, 0.02,   0, 0,   0, 0,   -pi, pi])
    juiceTSR.manipindex = 0
    
    test=GeneralIK('','',[juiceTSR],False)
    test.supportlinks=['leftFootBase']
    test.cogtarget=(.1,.2,.3)
    print test.Serialize()
