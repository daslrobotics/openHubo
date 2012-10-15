#!/usr/bin/env python
from openravepy import *
from servo import *
from numpy import pi

def read_youngbum_traj(filename,robot):
    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    config=robot.GetConfigurationSpecification()
    config.AddDeltaTimeGroup()
    traj.Init(config)
    ind=makeNameToIndexConverter(robot)

    f=open(filename,'r')
    header=f.readline().rstrip()
    indices=[]
    print header.split()

    for h in header.split(' '):
        indices.append(ind(h))

    signlist=f.readline().rstrip()

    signs=[]
    print signlist.split(' ')
    for s in signlist.split(' '):
        if s == '+':
            signs.append(1)
        else:
            signs.append(-1)

    k=0
    affinedof=zeros(7)
    #TODO: read out of traj?
    dt=0.01
    while True: 
        string=f.readline().rstrip()
        if len(string)==0:
            break
        jointvals=[float(x) for x in string.split(' ')]
        data=zeros(robot.GetDOF())

        for i in range(len(jointvals)):
            data[indices[i]]=jointvals[i]*pi/180.0*signs[i]*.7

        waypt=list(data)
        waypt.extend(affinedof)
        waypt.append(dt)
        traj.Insert(k,waypt)
        k=k+1

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)
    return traj

if __name__=='__main__':

    try:
        file_env = sys.argv[1]
    except IndexError:
        file_env = 'scenes/simpleFloor.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    timestep=0.0005

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot = env.GetRobots()[0]
        #Create a "shortcut" function to translate joint names to indices
        ind = makeNameToIndexConverter(robot)

        #initialize the servo controller
        controller=RaveCreateController(env,'trajectorycontroller')
        robot.SetController(controller)

        #Set an initial pose before the simulation starts
        robot.SetDOFValues([pi/8,-pi/8],[ind('LSR'),ind('RSR')])
        controller.SendCommand('set gains 50 0 8')
        time.sleep(1)

        #Use the new SetDesired command to set a whole pose at once.
        pose=array(zeros(60))

        #Manually align the goal pose and the initial pose so the thumbs clear
        pose[ind('RSR')]=-pi/8
        pose[ind('LSR')]=pi/8

        controller.SetDesired(pose)

        env.StartSimulation(timestep=timestep)

    #The name-to-index closure makes it easy to index by name 
    # (though a bit more expensive)
    traj=read_youngbum_traj('traj_pump.txt',robot)

    controller.SetPath(traj)
    controller.SendCommand('start')
    while not(controller.IsDone()):
        time.sleep(.1)
        print controller.GetTime()




