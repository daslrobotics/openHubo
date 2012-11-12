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

from numpy import *
from rodrigues import *
from TransformMatrix import *
import copy
from TSR import *

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
    
    def insertTSRChain(self,chain):
        #TODO: Is it better to pass in by reference to make it easy to change?
        self.tsr_chains.append(copy.deepcopy(chain))

    def Serialize(self):
        cmd='RunCBiRRT' 
        cmd = cmd + ' filename {} timelimit {} smoothingitrs {}'.format(self.filename,self.timelimit,self.smoothing)
        goalSampling=False
        if len(self.jointgoals)>0:
            cmd=cmd+' jointgoals {} {}'.format(len(self.jointgoals),Serialize1DMatrix(mat(self.jointgoals)))
        if len(self.jointstarts)>0:
            cmd=cmd+' jointstarts {} {}'.format(len(self.jointstarts),Serialize1DMatrix(mat(self.jointstarts)))
        for chain in self.tsr_chains:
            cmd=cmd+' {}'.format(chain.Serialize())
            if chain.bSampleStartFromChain or chain.bSampleGoalFromChain:
                goalSampling=True
        
        if goalSampling:
            cmd=cmd+' psample {}'.format(self.psample)
        if len(self.supportlinks)>0:    
            cmd=cmd+' supportlinks {} {}'.format(len(self.supportlinks),' '.join(self.supportlinks))
        
        #print cmd
        return cmd
    def run(self):
        stat = self.problem.SendCommand(self.Serialize())
        if stat == '1':
            self.solved=True
            
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
        
#TODO: test functions?
