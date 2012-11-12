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
from servo import *
import openhubo 
import matplotlib.pyplot as plt

class ServoTest:

    def __init__(self,filename):
        self.jointdata={}
        self.import_servo_data(filename)

    def import_servo_data(self,filename):

        with open(filename,'r') as f:
            gainstring=f.readline().rstrip()
            names=f.readline().rstrip().split(' ')
            servostrings=f.readlines()

        for l in servostrings:
            data=l.rstrip().split(' ')
            #Store a dictionary of lists?
            self.jointdata.setdefault(data[0],[float(x) for x in data[1:]])

    def plot(self,servolist=['LEP']):
        for s in servolist:
            REF='{}_REF'.format(s)
            plt.plot(self.jointdata[REF],'+',hold=True)
            plt.plot(self.jointdata[s],hold=True)


if __name__=='__main__':
    test=ServoTest('exampleservodata.txt')
    servos=['LEP','LWP'] 
    test.plot(servos)
    plt.show()
