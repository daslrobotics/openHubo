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

__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

from openravepy import *
from numpy import *
from openhubo import *
from openhubo import kbhit

(env,options)=setup('qtcoin',True)
env.SetDebugLevel(4)
sleep(.25)

options.physicsfile=True
[robot,ctrl,ind,ref,recorder]=load_scene(env,options)

with env:
    #Lock environment to override the servo controller with a controller that does nothing
    robot.SetController(RaveCreateController(env,'idealcontroller'))

#Initialize our pose array as the robot's current pose
pose=Pose(robot)

#Copy the pose array to make an easy fixed set point.
desiredpose=Pose(robot)
#Set a simple goal pose
desiredpose['RSP']=-.4

timestep=0.0005
#Set PD control gains (probably will need a lot of tuning, and these values are not gauranteed to work!)
Kp=10
#Normalize Kd wrt timestep
Kd=Kp*0.1*timestep

error=zeros(robot.GetDOF())

#Collect masses of bodies attached at joint as a very crude scale factor for gains
masses=array([j.GetHierarchyParentLink().GetMass() + j.GetHierarchyChildLink().GetMass() for j in robot.GetJoints()])

#TODO: better way might be to figure out how much weight a joint has to bear? or use something better than PD :-)A

print "Beginning torque control loop, hit a key to stop..."
#TODO: getch-based loop interrupt
while not kbhit.kbhit():
    #Calculate error in pose
    with env:
        pose=robot.GetDOFValues()
        #Difference between current error and last value
        derror=((desiredpose-pose)-error)/timestep
        error=desiredpose-pose

        cmd=(Kp*error+Kd*derror)*masses

        """Finally, set all DOF to have their new torque value. The "True"
            argument is important (it specifies that torques should be added
            rather than overwritten!)."""
        robot.SetDOFTorques(cmd,True)
    env.StepSimulation(timestep)

