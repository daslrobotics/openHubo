#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

from openravepy import *
from numpy import *
from numpy.linalg import pinv,inv
import time
import datetime
import sys
import tab
import spacenav as sp
import openhubo
from TSR import *
from generalik import *
import copy

if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(4)
    env.SetViewer('qtcoin')
    time.sleep(.25)

    [robot,ctrl,ind,ref_robot,recorder]=openhubo.load(env,'rlhuboplus.robot.xml','floor.env.xml',True)
    
    pose=zeros(robot.GetDOF())
    with env:
        env.StartSimulation(timestep=0.0005)

    time.sleep(1)

    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,robot.GetName())

    robot.SetDOFValues([-1],[ind('LEP')])
    ik=GeneralIK(robot,probs_cbirrt)

    lh=robot.GetLink('leftPalm')
    Tstart=lh.GetTransform()
    la=robot.GetManipulator('leftArm')
    joints=la.GetArmIndices()
    lhgoal=TSR()
    lhgoal.manipindex=0
    ik.activate([ind('HPY')])
    ik.tsrlist.append(lhgoal)
    ik.gettime=False
    ik.return_closest=True
    ik.auto=False

    gains=array([0.005,0.005,0.005,pi/180,pi/180,pi/180])/25.
    deadzone=ones(6)*20
    spnav = sp.SpaceNav(env,deadzone,gains)

    #TODO: add returnclosest feature to generalik
    #   Make "backing out" of colliding final pose. If closest solution is in
    #   collision, linearly bisect between current (non-colliding) pose and
    #   returned colliding pose to find the closest non-colliding version,
    #   limited by some tolerance.
    #   The robot should be able to "slide" it's hand over itself in simulation
    #   without triggering a collision, perhaps with some safety margin.

    dofs=robot.GetActiveDOFIndices()
    while True:
        time.sleep(.05)
        #Current EE pose
        T=mat(lh.GetTransform())
        #Differential transform
        Ts=mat(spnav.get_transform())
        #Get differential rotation only
        dR=Ts[0:3,0:3]
        dt=Ts[0:3,3]
        #Global rotation update to current pose
        Rnew=dR*T[0:3,0:3]
        Tnew=T
        Tnew[0:3,0:3]=Rnew
        Tnew[0:3,3]+=dt
        ik.tsrlist[0].Tw_e=array(copy.deepcopy(Tnew))
        ik.run(True)
        pose[dofs]=ik.soln
        ctrl.SetDesired(pose)


