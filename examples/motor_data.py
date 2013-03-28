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

from openhubo import *
from openhubo.motor import *
from openrave import *
from numpy import *
import matplotlib.pyplot as plt

(env,options)=setup('qtcoin')
env.SetDebugLevel(4)
time.sleep(.25)

env.Load('physics.xml')

[robot,ctrl,ind,ref_robot,recorder]=load_scene(env,options.robotfile,'floor.env.xml',True)

# Build pose from current DOF values, specify a test pose, and update the desired pose
pose=zeros(robot.GetDOF())
robot.SetDOFValues(pose)
pose[ind('RKP')]=.5
pose[ind('LKP')]=.5
pose[ind('LHP')]=-.25
pose[ind('RHP')]=-.25
pose[ind('LAP')]=-.25
pose[ind('RAP')]=-.25
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

for k in range(5000):
    pose[ind('RSP')]=.2*sin(k*2*pi*TIMESTEP)
    pose[ind('LSP')]=.2*sin(k*2*pi*TIMESTEP)
    ctrl.SetDesired(pose)
    env.StepSimulation(TIMESTEP)
    data.append(motor.get_state())

t1=time.time()
print t1-t0

plt.plot(array(data))
plt.legend(('Voltage,V','Current, A','Torque, Nm','Speed, rad/s'))
plt.show()

