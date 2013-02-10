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
try:
    import matplotlib.pyplot as plt
except ImportError:
    print "matplotlib is not available. Install the python-matplotlib package to enable plotting"


class MotorModel:

    def __init__(self,joint,Ks=1,R=1,N=1):
        robot=joint.GetParent()
        env=robot.GetEnv()
        physics=env.GetPhysicsEngine() 
        #Closure for force / torque
        def get_ft():
            return physics.GetJointForceTorque(joint)
        self.get_ft=get_ft
        def get_vel():
            return joint.GetVelocities()[0]
        self.get_vel=get_vel
        self.axis=joint.GetAxis
        self.Kv=60./Ks/2/pi
        self.R=R
        self.N=N

    def get_state(self):
        [force,torque]=self.get_ft()
        T=torque.dot(self.axis())
        w=self.get_vel()
        wm=N*w
        Tm=T/self.N
        i=(Tm)/self.Kv
        V=self.R*i+self.Kv*wm
        return [V,i,T,w]


def make_DC_motor(joint,Ks,R,N):
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
    j1 = robot.GetJoint('RSP')
    with env:
        env.StopSimulation()
    time.sleep(.1)

    Ks=346
    R=.346
    N=160

    data=[]
    t0=time.time()
    motor=MotorModel(j1,Ks,R,N)

    for k in range(10000):
        pose[ind('RSP')]=.25*sin(k*2*pi/2000)
        ctrl.SetDesired(pose)
        env.StepSimulation(0.0005)
        data.append(motor.get_state())

    t1=time.time()
    print t1-t0
    
    if 'plt' in globals():
        plt.plot(array(data))
        plt.legend(('Voltage,V','Current, A','Torque, Nm','Speed, rad/s'))
        plt.show()
    
