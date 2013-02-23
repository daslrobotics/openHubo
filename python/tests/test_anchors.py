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
import datetime
import sys
import unittest
import openhubo

#Get the global environment for simulation

def model_test_factory(filename=None):
    class TestAnchors(unittest.TestCase):
        def setUp(self):
            (self.env,options)=openhubo.setup()
            self.env.SetDebugLevel(2)
            [self.robot,ctrl,ind,ref,recorder]=openhubo.load(self.env,filename,None,True,False)

        def tearDown(self):
            self.env.Destroy()

        def test_anchors(self):
            raveLogWarn('Loaded {}'.format(filename))

            anchor={}
            axes={}
            for j in self.robot.GetJoints():
                anchor.setdefault(j.GetName(),j.GetAnchor())
                axes.setdefault(j.GetName(),j.GetAxis())

            LSerr=constraints_3dof(anchor['LSR'],anchor['LSP'],anchor['LSY'])
            RSerr=constraints_3dof(anchor['RSR'],anchor['RSP'],anchor['RSY'])
            print     LSerr
            print     RSerr

            if anchor.has_key('LWR'):
                LWerr=constraints_3dof(z=anchor['LWY'],y=anchor['LWP'],x=anchor['LWR'])
                RWerr=constraints_3dof(z=anchor['RWY'],y=anchor['RWP'],x=anchor['RWR'])
            elif anchor.has_key('LWP'):
                LWerr=constraints_3dof(z=anchor['LWY'],y=anchor['LWP'])
                RWerr=constraints_3dof(z=anchor['RWY'],y=anchor['RWP'])
            else:
                LWerr=zeros(3)
                RWerr=zeros(3)
            print     LWerr
            print     RWerr

            LHerr=constraints_3dof(anchor['LHR'],anchor['LHP'],anchor['LHY'])
            RHerr=constraints_3dof(anchor['RHR'],anchor['RHP'],anchor['RHY'])

            LAerr=constraints_3dof(x=anchor['LAR'],y=anchor['LAP'])
            RAerr=constraints_3dof(x=anchor['RAR'],y=anchor['RAP'])

            print     LHerr
            print     RHerr
            print     LAerr
            print     RAerr

            self.assertLess(sum(abs(array(LSerr))),1e-15)
            self.assertLess(sum(abs(array(RSerr))),1e-15)
            self.assertLess(sum(abs(array(LWerr))),1e-15)
            self.assertLess(sum(abs(array(RWerr))),1e-15)
            self.assertLess(sum(abs(array(LHerr))),1e-15)
            self.assertLess(sum(abs(array(RHerr))),1e-15)
            self.assertLess(sum(abs(array(LAerr))),1e-15)
            self.assertLess(sum(abs(array(RAerr))),1e-15)

    return TestAnchors

def constraints_3dof(x=array([]),y=array([]),z=array([])):

    if x.any() and y.any():
        diff0=x[2]-y[2]
    else:
        diff0=0.0
    if x.any() and z.any():
        diff1=x[1]-z[1]
    else:
        diff1=0.0
    if y.any() and z.any():
        diff2=z[0]-y[0]
    else:
        diff2=0.0
    return [diff2,diff1,diff0]

if __name__=='__main__':
    test1=model_test_factory('huboplus.robot.xml')
    test2=model_test_factory('rlhuboplus.robot.xml')
    test3=model_test_factory('rlhuboplus.noshell.robot.xml')
    test4=model_test_factory('rlhuboplus.fingerless.robot.xml')
    test5=model_test_factory('hubo2.robot.xml')
    test6=model_test_factory('rlhubo2.robot.xml')
    test7=model_test_factory('rlhuboplus.cushionhands.robot.xml')
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())

