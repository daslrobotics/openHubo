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

__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openravepy as _rave
import matplotlib.pyplot as _plt

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

