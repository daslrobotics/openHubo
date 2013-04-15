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

import openhubo as oh
from numpy import pi

(env,options)=oh.setup('qtcoin',True)
env.SetDebugLevel(4)

#Note that the load function now directly parses the option structure
options.physicsfile=True
[robot,ctrl,ind,ref,recorder]=oh.load_scene(env,options)
env.StartSimulation(oh.TIMESTEP)

#Change the pose to lift the elbows and send
filename='gaintest.txt'
with robot:
    ctrl.SendCommand('set radians ')
    ctrl.SendCommand('set gains 150 0 .9 '.format(ind('RSP')))
    ctrl.SendCommand('record_on {} '.format(filename))
#0.7.1 Syntax change: Note the new "Pose" class:
pose=oh.Pose(robot,ctrl)
pose['RSP']=.4
pose.send()
oh.sleep(3)
ctrl.SendCommand('record_off {0} {1} {1} '.format(filename,ind('RSP')))

rspdata=oh.ServoPlotter(filename)
#rspdata.plot(['RSP'])
print 'rsp = [ '+' '.join(['{}'.format(x) for x in rspdata.jointdata['RSP']])
