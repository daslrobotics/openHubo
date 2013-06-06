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
from openhubo import hands
from openravepy import raveLogDebug,raveLogInfo

class TestHands(unittest.TestCase):
    def setUp(self):
        (self.env,options)=openhubo.setup()
        self.env.SetDebugLevel(1)
        [self.robot,ctrl,self.ind,ref,recorder]=openhubo.load_scene(self.env,options)
        self.manip=self.robot.GetManipulator('rightArm')
        self.fingers=self.manip.GetChildJoints()

    def tearDown(self):
        self.env.Destroy()

    def test_close_fingers(self):
        hands.close_finger(self.fingers[0],.1)

    def test_close_hand(self):
        hands.close_huboplus_hand(self.robot,0,.3)
        hands.close_huboplus_hand(self.robot,0,.3,[1,.5,.5,1,.5,.5,1,.5,.5,1,.5,.5,1,.5,.5])
        hands.open_huboplus_hand(self.robot,0,0)

if __name__=='__main__':
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())


