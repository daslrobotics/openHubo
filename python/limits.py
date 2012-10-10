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
import tab

def checkCollision(robot,bodies):
    #Check the first body against the rest
    #Note that this IGNORES adjacent bodies if invoked this way. Therefore, we
    # can use it to directly inspect the model for joint body collisions.
    env=robot.GetEnv()

    if len(bodies)>1:
        for k in range(0,len(bodies)-1):
            #print "Checking collisions for",bodies[k]
            for body in bodies[k+1:]:
                #print body
                res=env.CheckCollision(robot.GetLink(body),robot.GetLink(bodies[k]))
                if res==True:
                    #print "Collision between {} and {}".format(body,bodies[k])
                    return True
    return False


def checklimits(robot,joint,checkbodies):
    env=robot.GetEnv()
    testAngles=array(range(-180,180))
    N=len(testAngles)
    results=array([True for x in range(N)])
    regions=array(zeros(N))
    for k in range(0,N):
        #Go to a check position
        robot.SetDOFValues([float(testAngles[k])*pi/180],[joint.GetDOFIndex()])
        test=checkCollision(robot,checkbodies)
        #print "Joint angle {} colliding: {}".format(testAngles[k],test)
        results[k]=test
        if k>0 and results[k] != results[k-1]:
            regions[k]=regions[k-1]+1
        else:
            regions[k]=regions[k-1]
        #print "Region {}".format(regions[k])


    robot.SetDOFValues([0],[joint.GetDOFIndex()])
    results=testAngles[where(results==False)]
    print joint
    if len(results)>0:
        print min(results),max(results)
    else:
        print "Joint always in collision (check geometry)"
    return results

if __name__=='__main__':
    #-- Read the name of the xml file passed as an argument
    #-- or use the default name
    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'huboplus/huboplus.robot.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(1)
    env.Load(file_env)

    with env:
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    time.sleep(1)
    resultsLHR=checklimits(robot,robot.GetJoint('LHR'),['Body_LHY','Body_LHP',])
    resultsLHP=checklimits(robot,robot.GetJoint('LHP'),['Body_LHY','Body_LHP'])
    resultsLKP=checklimits(robot,robot.GetJoint('LKP'),['Body_LHP','Body_LKP',])
    resultsLAP=checklimits(robot,robot.GetJoint('LAP'),['Body_LAR','Body_LKP','Body_LAP'])
    resultsLAR=checklimits(robot,robot.GetJoint('LAR'),['Body_LAR','Body_LKP','Body_LAP'])
    resultsLSR=checklimits(robot,robot.GetJoint('LSR'),['Body_LSR','Body_LSP'])
    resultsLEP=checklimits(robot,robot.GetJoint('LEP'),['Body_LSY','Body_LEP'])
    resultsLWP=checklimits(robot,robot.GetJoint('LWP'),['Body_LWP','Body_LWY'])
    resultsLWR=checklimits(robot,robot.GetJoint('LWR'),['Body_LWP','Body_LWY'])

