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
import time
import sys
from servo import *
import openhubo 
import matplotlib.pyplot as plt

def make_DC_motor(joint,Ks,R,N):
    robot=joint.GetParent()
    env=robot.GetEnv()
    physics=env.GetPhysicsEngine()
    axis=joint.GetAxis(0)
    Kv=60./Ks/2/pi
    Tf=0.0 #Loss torque, assuming zero for now
    def maxon_motor_model():
        """ Ks in rpm/V
            Kt in mNm/A
            R in ohm
            L in mH
            Calculate motor current from a torque and speed, assuming no losses due
            to friction."""
        [force,torque]=physics.GetJointForceTorque(joint)
        T=torque.dot(axis)
        w=joint.GetVelocities()[0]
        wm=N*w
        Tm=T/N
        i=(Tm+Tf)/Kv
        V=R*i+Kv*wm
        return [V,i,T,w]
    return maxon_motor_model

if __name__=='__main__':

    env = Environment()
    env.SetDebugLevel(4)
    env.SetViewer('qtcoin')
    time.sleep(.25)
    #-- Set the robot controller and start the simulation
    with env:
        #NOTE: Make sure to use this sequence of commands WITHIN a "with env:"
        #block to ensure that the model loads correctly.
        env.StopSimulation()
        env.Load('floor.env.xml')
        env.Load('rlhuboplus.robot.xml')
        robot = env.GetRobots()[0]

        pose=zeros(robot.GetDOF())
        #Very important to make sure the initial pose is not colliding
        robot.SetDOFValues(pose)

        robot.SetController(RaveCreateController(env,'servocontroller'))
        collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        if not collisionChecker:
            print "Using ODE since PQP is not available..."
            collisionChecker = RaveCreateCollisionChecker(env,'ode')

        env.SetCollisionChecker(collisionChecker)

        ctrl=robot.GetController()
        ctrl.SendCommand('setgains 100 0 8')
        #Note that you can specify the input format as either degrees or
        #radians, but the internal format is radians
        ctrl.SendCommand('set radians')
        #Use .0005 timestep for non-realtime simulation with ODE to reduce jitter.
        
    ind = openhubo.makeNameToIndexConverter(robot)
    pose = robot.GetDOFValues()

    #pose[ind('RKP')]=.5
    #pose[ind('LKP')]=.5
    #pose[ind('LHP')]=-.25
    #pose[ind('RHP')]=-.25
    #pose[ind('LAP')]=-.25
    #pose[ind('RAP')]=-.25
    ctrl.SetDesired(pose)
    p=env.GetPhysicsEngine()
    data=zeros((10000,12))
    j1 = robot.GetJoint('RKP')
    j2 = robot.GetJoint('LKP')
    with env:
        env.StopSimulation()
    time.sleep(.1)

    Ks=346
    R=.346
    N=160

    data=[]
    motor=make_DC_motor(j1,Ks,R,N)
    for k in range(10000):
        env.StepSimulation(0.0005)
        data.append(motor())

