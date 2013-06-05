# -*- coding: utf-8 -*-
""" Repacked COnstrained Manipulation Planning Suite python functions into new classes for use with openhubo.
"""

import numpy as _np
import openravepy as _rave
from openravepy import matrixSerialization,KinBody
import copy as _copy
import time as _time
from openhubo import Pose

def Serialize1DMatrix(arr):
    return ' '.join([str(x) for x in arr])

def SerializeTransform(tm):
    if isinstance(tm,Transform):
        return tm.serialize()
    return matrixSerialization(_np.array(tm))

class Transform:
    """Equivalent to RaveTransform class in OpenRAVE"""

    def __init__(self,rot=None,trans=None,check=True):
        if isinstance(rot,Transform):
            self.tm=rot.tm
        elif _np.size(rot) == 16:
            #TODO: additional checks
            self.tm=_np.mat(rot)
        #TODO: init from 3x4 matrix
        else:
            self.tm=Transform.make_transform(rot,trans,check)

    def rot(self):
        return self.tm[0:3,0:3]

    def trans(self):
        return self.tm[0:3,3]

    def serialize(self):
        return matrixSerialization(_np.array(self.tm))

    def inv(self):
        """Slow hack way to get Transform inverse"""
        Tnew=Transform()
        Tnew.tm[0:3,0:3]=self.rot().T
        Tnew.tm[0:3,3]=-self.trans()
        return Tnew

    def __mul__(self,other):
        #TODO: make this return a Transform?
        """Overload multiplication to provide proper matrix multiplication of 2
        Transforms, or a Transform and an equivalent matrix.
        Note that this function returns a matrix, NOT a Transform."""
        if type(other) == type(self):
            return self.tm * other.tm
        #Hope that the user sets this up right...
        if type(other) == _np.matrixlib.defmatrix.matrix:
            return self.tm * other
        if type(other) == _np.ndarray:
            return self.tm * _np.mat(other)

    def __rmul__(self,other):
        #TODO: make this return a Transform?
        """Overload multiplication to provide proper matrix multiplication of 2
        Transforms, or a Transform and an equivalent matrix.
        Note that this function returns a matrix, NOT a Transform."""
        if type(other) == type(self):
            return other.tm * self.tm
        #Hope that the user sets this up right...
        if type(other) == _np.matrixlib.defmatrix.matrix:
            return other * self.tm
        if type(other) == _np.ndarray:
            return _np.mat(other) * self.tm

    @staticmethod
    def make_transform(rot=None,trans=None,check=False):
        """Build a transformation matrix out of a rotation and translation of multiple possible shapes.
            Assumes that a rotation matrix is arranged by array index:
                [0  3   6;
                1  4   7;
                2  5   8]
            Therefore, if an array is passed in as a row, it must be transposed after being reshaped:
                [0,1,2,3,4,5,6,7,8,9]
        """
        T = _np.mat(_np.eye(4))

        if rot is not None:
            if  _np.size(rot) == 9 :
                #correct overall dimensions, check for shape and act accordingly
                #Assume that a rotation matrix
                R = _np.mat(rot).reshape(3,3)
                if _np.size(rot,1)==9:
                    R=R.transpose()

                if check:
                    T[0:3,0:3], __ = _np.linalg.qr(R)
                else:
                    T[0:3,0:3], __ = R
            elif _np.size(rot) == 3:
                #assume we want rodrigues
                T[0:3,0:3] = rodrigues(rot)
            else:
                print rot
                print('No rotation matrix specified, assuming eye(3)')

        if trans is not None and _np.size(trans)==3:
            T[0:3,3] = _np.mat(trans).reshape(3,1)

        return T


def rodrigues(r):
    """Rodrigues formula
    Input: 1x3 array of rotations about x, y, and z
    Output: 3x3 rotation matrix"""
    def S(n):
        Sn = _np.array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    theta = _np.linalg.norm(r)
    if theta > 1e-30:
        n = r/theta
        Sn = S(n)
        R = _np.eye(3) + _np.sin(theta)*Sn + (1-_np.cos(theta))*_np.dot(Sn,Sn)
    else:
        Sr = S(r)
        theta2 = theta**2
        R = _np.eye(3) + (1-theta2/6.)*Sr + (.5-theta2/24.)*_np.dot(Sr,Sr)
    return _np.mat(R)


class Cbirrt:
    """ Convenient class to work with CBiRRT problems in python. This class
    stores properties of a problem such as the filename, time limits, TSR's,
    and has methods to run and return results, while allowing the user to avoid
    dealing directly with string commands."""
    def __init__(self, problem,tsr_chains=[],filename='cmovetraj.txt',timelimit=30,smoothing=10):
        self.problem=problem
        self.tsr_chains=[]
        try:
            self.tsr_chains.extend(tsr_chains)
        except TypeError:
            self.tsr_chains.append(tsr_chains)

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
        """Insert a defined tsr chain into the list of tsr chains. Simply appends to the internal list ."""
        self.tsr_chains.append(_copy.deepcopy(chain))

    def Serialize(self):
        """ Convert CBiRRT instance to a command string ready to send to the OpenRAVE Problem."""
        cmd=['RunCBiRRT']
        cmd.append('filename {}'.format(self.filename))
        cmd.append('timelimit {}'.format(self.timelimit))
        cmd.append('smoothingitrs {}'.format(self.smoothing))
        goalSampling=False

        if len(self.jointgoals)>0:
            cmd.append('jointgoals {}'.format(len(self.jointgoals)))
            cmd.append(Serialize1DMatrix(self.jointgoals))
        if len(self.jointstarts)>0:
            cmd.append('jointstarts {}'.format(len(self.jointstarts)))
            cmd.append(Serialize1DMatrix(self.jointstarts))
        for chain in self.tsr_chains:
            cmd.append('{}'.format(chain.Serialize()))
            if chain.bSampleStartFromChain or chain.bSampleGoalFromChain:
                goalSampling=True
        if goalSampling:
            cmd.append('psample {}'.format(self.psample))
        if len(self.supportlinks)>0:
            cmd.append('supportlinks {} {}'.format(len(self.supportlinks),' '.join(self.supportlinks)))
            cmd.append('exactsupport {}'.format(int(self.exactsupport)))

        #print ' '.join(cmd)
        return ' '.join(cmd)

    def run(self):
        """Execute a problem and store the solution status internally."""
        stat = self.problem.SendCommand(self.Serialize())
        if stat == '1':
            self.solved=True
            return True
        return False

    def playback(self,force=False):
        """Re-run a solved trajectory, optionally forcing playback of a failed trajectory."""
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

    def quicksetup(self,T0_w, Tw_e, Bw, manipIndex):
        """A simple 1-manipulator setup method that bypasses explicit creation
        of TSR's and chains. Not meant to be a general method, just a quick
        shortcut."""
        tsr = TSR(T0_w,Tw_e,Bw,manipIndex)
        chain = TSRChain(0,1,0)
        chain.insertTSR(tsr)
        self.insertTSRChain(chain)


class GeneralIK:
    """ Wrapper class to hold solution settings for an IK problem. This class
    can use TSR's to define IK problems, and sample IK solutions from the
    TSR's."""
    def __init__(self,robot,problem,tsrlist=[],sample_bw=True,bcollisions=True):
        self.robot=robot
        self.problem=problem
        self.sample_bw=sample_bw
        self.soln=[]
        self.activedofs=[]
        self.tsrlist=[]
        self.zero=robot.GetDOFValues()
        self.supportlinks=[]
        self.cogtarget=()
        self.appendTSR(tsrlist)
        self.bcollisions=bcollisions

    def Serialize(self):
        #TODO: reformat as list then join at end
        L=len(self.tsrlist)
        cmd=['DoGeneralIK exec nummanips',str(format(L))]
        for k in self.tsrlist:
            #Assumes that Bw encloses T0_e
            if self.sample_bw:
                T0_e=k.sample()
            else:
                T0_e=k.endPose()
            cmd.extend(['maniptm',str(k.manipindex),SerializeTransform(T0_e)])

        if len(self.supportlinks)>0:
            cmd.extend(['supportlinks',str(len(self.supportlinks)),' '.join(self.supportlinks)])
        if len(self.cogtarget)>0:
            cmd.append('movecog {} {} {}'.format(self.cogtarget[0],self.cogtarget[1],self.cogtarget[2]))
        return ' '.join(cmd)

    def appendTSR(self,tsr):
        """Add a TSR (not a TSR Chain!) to the GeneralIK instance. Choose
        non-overlapping combinations of TSR's to avoid over-constrained
        problems."""
        try:
            self.tsrlist.extend(tsr)
        except TypeError:
            self.tsrlist.append(tsr)

    def activate(self,extra=[]):
        """Activate necessary DOF on the robot to allow the IK problem to run.
        Optionally include extra DOF's to activate that are not part of the
        manipulator."""
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
        """Move the robot to the current solved position (direct move, not controller command!)"""
        self.activate()
        self.robot.SetDOFValues(self.soln,self.activedofs)
        self.robot.WaitForController(.2)

    def solved(self):
        return len(self.soln)>0

    def findSolution(self,itrs=10,auto=False,extra=[]):
        return self.continuousSolve(itrs,auto,extra,False,True)

    def continuousSolve(self,itrs=1000,auto=False,extra=[],show=True,quitonsolve=False):
        """ Continously solve in realtime, allowing a user to reposition
         the robot, to quickly get an intuitive idea of what works
        and doesn't. Note that TSR's based on object poses are not recalculated
        if an object is moved."""
        report=_rave.CollisionReport()
        env=self.robot.GetEnv()
        pose=Pose(self.robot)
        for k in xrange(itrs):
            pose.send()
            with self.robot:
                self.run(auto,extra)
                colcheck = env.CheckCollision(self.robot,report=report) and self.robot.CheckSelfCollision() if self.bcollisions else False

            if show:
                self.goto()
                _time.sleep(.1)
            if self.solved and not colcheck and quitonsolve:
                break

        return self.solved()

    def resetZero(self):
        self.zero=self.robot.GetDOFValues()

#Dumped below out of laziness...
#
# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

from numpy import pi,array,mat,zeros,ones,eye,size,asarray,random
import copy

#these are standalone functions for serialization but you should really use the classes
def SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw):
    """
    SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

Input:
    manipindex (int): the 0-indexed index of the robot's manipulator
    bodyandlink (str): body and link which is used as the 0 frame. Format 'body_name link_name'. To use world frame, specify 'NULL'
    T0_w (double 4x4): transform matrix of the TSR's reference frame relative to the 0 frame
    Tw_e (double 4x4): transform matrix of the TSR's offset frame relative the w frame
    Bw (double 1x12): bounds in x y z roll pitch yaw. Format: [x_min x_max y_min y_max...]

Output:
    outstring (str): string to use for SerializeTSRChain function
    """

    return '%d %s %s %s %s'%(manipindex, bodyandlink, T0_w.serialize(), Tw_e.serialize(), Serialize1DMatrix(Bw))

def SerializeTSRChain(bSampleStartFromChain,bSampleGoalFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints):
    """
    SerializeTSRChain(bSampleFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)

Input:
    bSampleStartFromChain (0/1): 1: Use this chain for sampling start configurations   0:Ignore for sampling starts
    bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configurations   0:Ignore for sampling goals
    bConstrainToChain (0/1): 1: Use this chain for constraining configurations   0:Ignore for constraining
    numTSRs (int): Number of TSRs in this chain (must be > 0)
    allTSRstring (str): string of concetenated TSRs generated using SerializeTSR. Should be like [TSRstring 1 ' ' TSRstring2 ...]
    mimicbodyname (str): name of associated mimicbody for this chain (NULL if none associated)
    mimicbodyjoints (int [1xn]): 0-indexed indices of the mimicbody's joints that are mimiced (MUST BE INCREASING AND CONSECUTIVE [FOR NOW])

Output:
    outstring (str): string to include in call to cbirrt planner"""

    outstring = ' TSRChain %d %d %d %d %s %s'%(bSampleStartFromChain, bSampleGoalFromChain, bConstrainToChain, numTSRs, allTSRstring, mimicbodyname)
    if size(mimicbodyjoints) != 0:
        outstring += ' %d %s '%(size(mimicbodyjoints),Serialize1DMatrix(mimicbodyjoints))

    return outstring

class TSR:
    @staticmethod
    def buildT(w):
        print w
        return Transform(w[3:],w[0:3])

    def __init__(self, T0_w_in = None, Tw_e_in = None, Bw_in = zeros(12), manipindex_in = None, link_in = None):
        self.T0_w = Transform(T0_w_in)
        self.Tw_e = Transform(Tw_e_in)
        self.Bw = copy.deepcopy(array(Bw_in))
        self.manipindex = manipindex_in
        self.link = link_in
        self.Bw.reshape((12,))

    def __eq__(self,other):
        tests=[self.T0_w==other.T0_w,self.Tw_e==other.Tw_e,self.Bw == other.Bw,self.link == other.link]
        #all must be true, ignore manipindex
        return bool(_np.prod(tests))

    def Serialize(self):
        #hack to serialize link
        if self.link is None:
            bodyandlink="NULL"
        elif isinstance(self.link,KinBody.Link):
            bodyandlink=self.link.GetParent().GetName()+' '+self.link.GetName()
        elif isinstance(self.link,KinBody):
            bodyandlink=self.link.GetName()+' '+self.link.GetLinks()[0].GetName()
        else:
            raise TypeError('Need a kinbody or link definition')

        manipindex= self.manipindex if self.manipindex else -1

        cmd=[str(manipindex), str(bodyandlink), self.T0_w.serialize(), self.Tw_e.serialize(), Serialize1DMatrix(self.Bw)]
        return ' '.join(cmd)

    def endPose(self):
        T0=Transform(self.link.GetTransform())
        return T0*self.T0_w*self.Tw_e

    def sample(self):
        b_range=self.Bw[1::2]-self.Bw[0::2]
        b_center=(self.Bw[1::2]+self.Bw[0::2])/2
        w=array(random.rand(6))*asarray(b_range)+asarray(b_center)
        return self.endPose()*TSR.buildT(w)

class TSRChain:
    def __init__(self, bSampleStartFromChain_in=0, bSampleGoalFromChain_in=0, bConstrainToChain_in=0, mimicbodyname_in="NULL", mimicbodyjoints_in = []):
        self.bSampleStartFromChain = bSampleStartFromChain_in
        self.bSampleGoalFromChain = bSampleGoalFromChain_in
        self.bConstrainToChain = bConstrainToChain_in
        self.mimicbodyname = mimicbodyname_in
        self.mimicbodyjoints = mimicbodyjoints_in
        self.TSRs = []

    def insertTSR(self, tsr_in):
        self.TSRs.append(copy.deepcopy(tsr_in))

    def Serialize(self):
        print self.TSRs[0]
        allTSRstring = ' '.join([tsr.Serialize() for tsr in self.TSRs])
        numTSRs = len(self.TSRs)
        outstring = ' TSRChain %d %d %d %d %s %s'%(self.bSampleStartFromChain, self.bSampleGoalFromChain, self.bConstrainToChain, numTSRs, allTSRstring, self.mimicbodyname)
        if size(self.mimicbodyjoints) != 0:
            outstring += ' %d %s '%(size(self.mimicbodyjoints),Serialize1DMatrix(self.mimicbodyjoints))

        return outstring

    def SetFirstT0_w(self,T0_w_in):
        self.TSRs[0].T0_w = Transform(copy.deepcopy(T0_w_in))

