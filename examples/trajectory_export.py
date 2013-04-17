""" Simple test script to run some of the functions above. """

# boilerplate openhubo imports, avoid "import *" to allow pylint to check for
# undefined functions
import openhubo
from numpy import pi
from openravepy import planningutils

#example-specific imports
import openhubo.trajectory as tr

(env,options)=openhubo.setup()
env.SetDebugLevel(3)

timestep=openhubo.TIMESTEP

[robot,controller,ind,ref,recorder]=openhubo.load_scene(env,options)

pose0=openhubo.Pose(robot,controller)
pose0.update()
pose0.send()

env.StartSimulation(timestep=timestep)

pose1=openhubo.Pose(robot,controller)

pose1['LAP']=-pi/8
pose1['RAP']=-pi/8

pose1['LKP']=pi/4
pose1['RKP']=pi/4

pose1['LHP']=-pi/8
pose1['RHP']=-pi/8

[traj,config]=tr.create_trajectory(robot)

#Note the new waypoint-building syntax
traj.Insert(0,pose0.to_waypt(dt=0.0))
traj.Insert(1,pose1.to_waypt(dt=1.0))
traj.Insert(2,pose0.to_waypt(dt=1.0))

#Need to do this to add timing information for interpolating
planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

print 'Dump all DOFs to youngbum format'
tr.write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum.traj')

print 'Only use a selection of DOF\'s instead of everything'
tr.write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum2.traj',dofs=range(28))

print 'Write to hubo-read-trajectory compatible format'
tr.write_hubo_traj(traj,robot,0.025,'traj_example_hubo.traj')

print 'Test reading of trajectories'
traj_in_yb=tr.read_youngbum_traj('traj_example_youngbum2.traj',robot,0.005)
traj_in_text=tr.read_text_traj('traj_example_youngbum2.traj',robot,0.005)

print 'Test IU trajectory import'
iutraj = tr.IUTrajectory(robot)
iutraj.load_from_file('70_0.20.iutraj','iumapping.txt',openhubo.get_root_dir())
traj2=iutraj.to_openrave()


tr.write_hubo_traj(traj2,robot,0.025,'traj_example_iu.traj')
