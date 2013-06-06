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

from numpy import pi,array,zeros
import numpy as np
import unittest
import openhubo
from openravepy import raveLogDebug,raveLogInfo

#Get the global environment for simulation

def constraints_3dof(anchors,x=None,y=None,z=None):
    """Pick from a set of joint anchors and check if constraints are met."""

    if x:
        anchor_x=anchors[x]
    if y:
        anchor_y=anchors[y]
    if z:
        anchor_z=anchors[z]

    #Calculate distances between each pair of orthogonal planes
    if x and y:
        diff0=anchor_x[2]-anchor_y[2]
    else:
        diff0=0.0
    if x and z:
        diff1=anchor_x[1]-anchor_z[1]
    else:
        diff1=0.0
    if y and z:
        diff2=anchor_z[0]-anchor_y[0]
    else:
        diff2=0.0
    print '{} errors are {}'.format((x,y,z),(diff2,diff1,diff0))
    return [diff2,diff1,diff0]

def check_all_constraints(pose):
    """For a given robot pose, check anchors and report errors (only works for hubo-like robots)."""
    pose.send()
    anchors={j.GetName():j.GetAnchor() for j in pose.robot.GetJoints()}
    LSerr=constraints_3dof(anchors,'LSR','LSP','LSY')
    RSerr=constraints_3dof(anchors,'RSR','RSP','RSY')

    if anchors.has_key('LWR'):
        LWerr=constraints_3dof(anchors,z='LWY',y='LWP',x='LWR')
        RWerr=constraints_3dof(anchors,z='RWY',y='RWP',x='RWR')
    elif anchors.has_key('LWP'):
        LWerr=constraints_3dof(anchors,z='LWY',y='LWP')
        RWerr=constraints_3dof(anchors,z='RWY',y='RWP')
    else:
        LWerr=zeros(3)
        RWerr=zeros(3)

    LHerr=constraints_3dof(anchors,'LHR','LHP','LHY')
    RHerr=constraints_3dof(anchors,'RHR','RHP','RHY')

    LAerr=constraints_3dof(anchors,x='LAR',y='LAP')
    RAerr=constraints_3dof(anchors,x='RAR',y='RAP')

    return array([LSerr,RSerr,LWerr,RWerr,LHerr,RHerr,LAerr,RAerr])

def errmax(values):
    """Find the infinity norm of the values passed in."""
    return np.max(array(abs(values)))

def model_test_factory(filename=None):
    class TestAnchors(unittest.TestCase):
        def setUp(self):
            (self.env,options)=openhubo.setup()
            options.robotfile=filename
            options.physicsfile=None
            self.env.SetDebugLevel(1)
            [self.robot,ctrl,self.ind,ref,recorder]=openhubo.load_scene(self.env,options)

        def tearDown(self):
            self.env.Destroy()

        def test_anchors(self):
            raveLogInfo('Loaded {}'.format(filename))

            pose=openhubo.Pose(self.robot)

            errors_home=check_all_constraints(pose)

            pose['RSR']=15.*pi/180
            pose['LSR']=-15.*pi/180

            errors_bent_SR=check_all_constraints(pose)

            pose['REP']=10.*pi/180
            pose['LEP']=10.*pi/180

            errors_bent_EP=check_all_constraints(pose)

            #raveLogDebug( "Error sums for {}:".format(self.robot.GetName()))

            self.assertLess(min([errmax(errors_home),errmax(errors_bent_SR),errmax(errors_bent_EP)]),1e-10)

    return TestAnchors

test_anchor1=model_test_factory('huboplus.robot.xml')
test_anchor2=model_test_factory('rlhuboplus.robot.xml')
test_anchor3=model_test_factory('rlhuboplus.noshell.robot.xml')
test_anchor4=model_test_factory('rlhuboplus.fingerless.robot.xml')
test_anchor5=model_test_factory('hubo2.robot.xml')
test_anchor6=model_test_factory('rlhubo2.robot.xml')
test_anchor7=model_test_factory('rlhuboplus.cushionhands.robot.xml')

if __name__=='__main__':
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())

