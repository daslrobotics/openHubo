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
'''Functions for Serializing TSRs and TSR Chains

SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

Input:
manipindex (int): the 0-indexed index of the robot's manipulator
bodyandlink (str): body and link which is used as the 0 frame. Format 'body_name link_name'. To use world frame, specify 'NULL'
T0_w (double 4x4): transform matrix of the TSR's reference frame relative to the 0 frame
Tw_e (double 4x4): transform matrix of the TSR's offset frame relative the w frame
Bw (double 1x12): bounds in x y z roll pitch yaw. Format: [x_min x_max y_min y_max...]

Output:
outstring (str): string to use for SerializeTSRChain function


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
outstring (str): string to include in call to cbirrt planner'''
from numpy import *
from rodrigues import *
from TransformMatrix import *
import copy


#these are standalone functions for serialization but you should really use the classes
def SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw):
   
    return '%d %s %s %s %s'%(manipindex, bodyandlink, SerializeTransform(T0_w), SerializeTransform(Tw_e), Serialize1DMatrix(Bw))

def SerializeTSRChain(bSampleStartFromChain,bSampleGoalFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints):
    outstring = ' TSRChain %d %d %d %d %s %s'%(bSampleStartFromChain, bSampleGoalFromChain, bConstrainToChain, numTSRs, allTSRstring, mimicbodyname)
    if size(mimicbodyjoints) != 0:
        outstring += ' %d %s '%(size(mimicbodyjoints),Serialize1DMatrix(mimicbodyjoints))

    return outstring

class TSR():
    @staticmethod
    def buildT(w):
        S=rodrigues(squeeze(asarray(w[:,3:])))
        return MakeTransform(S,w[:,0:3].T)
    
    def __init__(self, T0_w_in = mat(eye(4)), Tw_e_in = mat(eye(4)), Bw_in = mat(mat(zeros([1,12]))), manipindex_in = -1, bodyandlink_in = 'NULL'):
      self.T0_w = copy.deepcopy(T0_w_in)
      self.Tw_e = copy.deepcopy(Tw_e_in)
      self.Bw = copy.deepcopy(Bw_in)
      self.manipindex = manipindex_in
      self.bodyandlink = bodyandlink_in

    def __eq__(self,other):
        return self.T0_w==other.T0_w and self.Tw_e==other.Tw_e and self.Bw == other.Bw
        
    def Serialize(self):
        return '%d %s %s %s %s'%(self.manipindex, self.bodyandlink, SerializeTransform(self.T0_w), SerializeTransform(self.Tw_e), Serialize1DMatrix(self.Bw))
    
    def endPose(self):
        return self.T0_w*self.Tw_e
    
    def sample(self):
        #print self.Bw
        b_range=self.Bw[:,1::2]-self.Bw[:,0::2]
        b_center=(self.Bw[:,1::2]+self.Bw[:,0::2])/2
        #print b_range
        #print b_center
        w=array(random.rand(1,6))*asarray(b_range)+asarray(b_center)
        return self.endPose()*TSR.buildT(w)

class TSRChain():
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
        allTSRstring = '%s'%(' '.join(' %s'%(tsr.Serialize()) for tsr in self.TSRs))
        numTSRs = len(self.TSRs)
        outstring = ' TSRChain %d %d %d %d %s %s'%(self.bSampleStartFromChain, self.bSampleGoalFromChain, self.bConstrainToChain, numTSRs, allTSRstring, self.mimicbodyname)
        if size(self.mimicbodyjoints) != 0:
            outstring += ' %d %s '%(size(self.mimicbodyjoints),Serialize1DMatrix(self.mimicbodyjoints))
  
        return outstring

    def SetFirstT0_w(self,T0_w_in):
        self.TSRs[0].T0_w = copy.deepcopy(T0_w_in)

if __name__ == '__main__':

    juiceTSR = TSR()
    juiceTSR.Tw_e = MakeTransform(rodrigues([pi/2, 0, 0]),mat([0, 0.22, 0.1]).T)
    juiceTSR.Bw = mat([0, 0,   0, 0,   -0.02, 0.02,   0, 0,   0, 0,   -pi, pi])
    juiceTSR.manipindex = 0


    juiceTSRChain1 = TSRChain(1,0)
    juiceTSRChain1.insertTSR(juiceTSR)
    
    

    juiceTSR.Tw_e = MakeTransform(rodrigues([0, pi, 0])*rodrigues([pi/2, 0, 0]),mat([0, 0.22, 0.1]).T)

    juiceTSRChain2 = TSRChain(1,0)
    juiceTSRChain2.insertTSR(juiceTSR)
    print juiceTSRChain1.Serialize()
    print juiceTSRChain2.Serialize()
    try:
        import readline
    except ImportError:
        print "Module readline not available."
    else:
        import rlcompleter
        readline.parse_and_bind("tab: complete")
        print "Tab completion enabled via readline module."
    
    juiceTSR.sample()

