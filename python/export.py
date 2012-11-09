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
import sys
import csv

def makeJointDict(robot):
    env = robot.GetEnv()

    jointMap={}

    for k in robot.GetJoints():
        jointMap[k.GetName()]=k.GetDOFIndex()

    return jointMap

if __name__=='__main__':

    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'hubo2/hubo2.robot.xml'

    env = Environment()
    env.Load(file_env)

    with env:
        robot = env.GetRobots()[0]

    jointMap={}

    #Strip off the xml suffix to create a file prefix
    file_prefix=file_env[:-4]

    #Open a "standard" file name to store the joint mapping
    with open('{}.joints.csv'.format(file_prefix),'wb') as f:
        writer=csv.writer(f,delimiter=' ') 
        for k in robot.GetJoints():
            # The line below builds a dictionary of joint names and DOF indices 
            jointMap[k.GetName()]=k.GetDOFIndex()
            # Print the same data to the screen with a formatted string
            print "{}:{}".format(k.GetName(),k.GetDOFIndex())
            # Use the CSV writer to export the data by row
            writer.writerow([k.GetName(),jointMap[k.GetName()]])

    with open('{}.joints.m'.format(file_prefix),'w') as f:
        for k in robot.GetJoints():
            f.write('{}={};\n'.format(k.GetName(),jointMap[k.GetName()]))

