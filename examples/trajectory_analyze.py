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

from openhubo import trajectory
import openhubo
import openravepy as rave
import time

if __name__=='__main__':
    (env,options)=openhubo.setup('qtcoin')
    [robot,ind,__,__,__]=openhubo.load_scene(env,options)

    #Read in trajectory from matlab source
    traj=trajectory.read_text_traj('trajectories/ingress_test.traj.txt',robot,0.01,1.0)
    config=traj.GetConfigurationSpecification()

    Getvals=trajectory.makeJointValueExtractor(robot,traj,config)
    Gettrans=trajectory.makeTransformExtractor(robot,traj,config)

    probs_cbirrt = rave.RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,'rlhuboplus')
    i=0
    t0=time.time()
    for i in range(traj.GetNumWaypoints()):
        robot.SetTransformWithDOFValues(Gettrans(i),Getvals(i))
        #print openhubo.find_com(robot)
        datastring=probs_cbirrt.SendCommand('CheckSupport supportlinks 2 rightFoot leftFoot exact 1 ')
        datalist=datastring.split(' ')
        check=float(datalist[0])
        mass=float(datalist[1])
        com=[float(x) for x in datalist[2:4]]
        pointdata=[float(x) for x in datalist[5:-1]]
        #print Gettrans(i)
        time.sleep(.01)
        #print robot.GetJoint('LAP').GetValues()

    t1=time.time()
    print 'Elapsed analysis time is {}'.format(t1-t0)
