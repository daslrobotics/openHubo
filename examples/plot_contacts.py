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
from openhubo import kbhit

if __name__ == "__main__":
    """Modified version of contact display from openrave examples"""

    (env,options)=openhubo.setup('qtcoin',True)
    env.SetDebugLevel(2)
    time.sleep(.25)

    [robot,ctrl,ind,ref,recorder]=openhubo.load_scene(env,options)

    stop=False

    # Set the floor and other bodies to be slightly transparent to better visualize interpenetrations
    for b in env.GetBodies():
        if not b == robot:
            openhubo.set_robot_color(b,trans=.4)

    while ~stop:
        if kbhit.kbhit():
            stop=True

        handles=openhubo.plot_contacts(robot)

        openhubo.sleep(0.02)

