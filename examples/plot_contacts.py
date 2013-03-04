#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from optparse import OptionParser
import time
import openravepy
import openhubo
import kbhit 
if not __openravepy_build_doc__:
    from numpy import *
    from openravepy import *

if __name__ == "__main__":
    """Modified version of contact display from openrave examples"""
    env=Environment()
    openhubo.load(env,'rlhuboplus.robot.xml','ladder.env.xml',False,False)
    env.SetViewer('qtcoin')
    robot=env.GetRobots()[0]
    env.SetCollisionChecker(RaveCreateCollisionChecker(env,'pqp'))
    raw_input('press key to show at least one contact point')
    stop=False
    while ~stop:
        if kbhit.kbhit():
            stop=True
        with env:
            # move both arms to collision
            lindex = robot.GetJoint('LSP').GetDOFIndex()
            rindex = robot.GetJoint('RSP').GetDOFIndex()
            robot.SetDOFValues([0.226,-1.058],[lindex,rindex])
        
            # setup the collision checker to return contacts
            env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)

            # get first collision
            report = CollisionReport()
            collision=env.CheckCollision(robot,report=report)
            print '%d contacts'%len(report.contacts)
            positions = [c.pos for c in report.contacts]

        if len(positions):
            h1=env.plot3(array(positions),10,[.7,.1,0])
        else:
            h1=[]

        time.sleep(0.05)
    raw_input('press any key to exit')


