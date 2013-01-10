#!/usr/bin/env python
# 08-16-12 OpenRave Tutorial Test Code 
# Hubo

from openravepy import *
import time
import scipy
import tab
from numpy import *
from numpy.linalg import *
import sys
from servo import *

initial_offset=1;
base_config=6
number_of_degrees=57

if __name__=='__main__':
    try:
        file_env = sys.argv[1]
    except IndexError:
	#file_env = 'scenes/ladderclimb.env.xml'
        file_env = 'scenes/simpleFloor_nophy.env.xml'

    env = Environment()
    env.SetViewer('qtcoin')
    env.SetDebugLevel(4)

    #-- Set the robot controller and start the simulation
    with env:
        env.StopSimulation()
        env.Load(file_env)
        robot = env.GetRobots()[0]
        collisionChecker = RaveCreateCollisionChecker(env,'ode')
        env.SetCollisionChecker(collisionChecker)
        robot.SetController(RaveCreateController(env,'idealcontroller'))
        #robot.GetController().SendCommand('set degrees')
	robot.SetDOFValues(zeros(robot.GetDOF()))
        env.StartSimulation(timestep=0.0005)
	ind=openhubo.makeNameToIndexConverter(robot)

    ## ROBOT VARIABLES
    joints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,56]

    init_trans = robot.GetTransform();
    x_off=init_trans[0][3]
    y_off=init_trans[1][3]
    z_off=init_trans[2][3]

    print "test random"
    #for i in range(1,58):    
    #     print str(robot.GetJoint(i).GetLimits())
    #     print "\n" 
    print str(robot.GetJoint(LSP).GetLimits())
    print "\n"
    print str(robot.GetJoint(RSP).GetLimits())
    print "\n"
    print "test over"
    time.sleep(1)

    #Read the file to obtain time steps and the total time
    file_read = open("q_path.txt")  # IU

    #line 1
    line = file_read .readline()
    if not line:
    	print "Version Missing"

    #line 2 has the total time
    total_time=0
    line = file_read .readline()
    if not line:
    	print "Total Time Missing"
    else:
	    total_time=float(line)

    #line 3 has the step size
    line = file_read .readline()
    if not line:
    	print "Time Step is Missing"
    else:
    	step_time=float(line)
    	print step_time

    number_of_steps=0
    if (total_time>0) & (step_time>0):
	number_of_steps = (int)(total_time/step_time)

    pos=[-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    off =[0]
    base = [0,0,0,0,0,0]
    theta=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    velocity=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    for line_counter in range (1,number_of_steps+1):
	print "----------------"
	print "line counter="
	print line_counter
	
        line = file_read .readline()
	#print line        
        if not line:
	    print "Configuration is Missing"
            break
        length =len(line)

        count=0

        for parser in range(0, length):
	    if line[parser]==',':
	    	count=count+1
	    	pos[count]=parser


	#print "initial offset" 
        for i in range(1,initial_offset): ## IU trajectory
	#for i in range(1,number_of_degrees): ## Purdue trajectory
	     off[i-1]=float(line[pos[i-1]+1:pos[i]])

	#print "Calculating BaseConfig" 
        for i in range(1+initial_offset,1+base_config+initial_offset): ## IU trajectory
	#for i in range(1,number_of_degrees): ## Purdue trajectory
	     base[i-1-initial_offset]=float(line[pos[i-1]+1:pos[i]])
	#print str(base)+"\n"

	psi_cal=base[3]
	theta_cal=base[4]
	phi_cal=base[5]
	trans_mat=[[cos(theta_cal)*cos(psi_cal)	,-cos(phi_cal)*sin(psi_cal)+sin(phi_cal)*sin(theta_cal)*cos(psi_cal),sin(phi_cal)*sin(psi_cal)+cos(phi_cal)*sin(theta_cal)*cos(psi_cal),base[0]+x_off],	
		   [cos(theta_cal)*sin(psi_cal)	,cos(phi_cal)*sin(psi_cal)+sin(phi_cal)*sin(theta_cal)*sin(psi_cal) ,-sin(phi_cal)*cos(psi_cal)+cos(phi_cal)*sin(theta_cal)*sin(psi_cal),base[1]+y_off],			
		   [-sin(theta_cal)		,sin(phi_cal)*cos(theta_cal),cos(phi_cal)*cos(theta_cal),base[2]+z_off],			
		   [	  0		,0,	0,	1]]			

	robot.SetTransform(trans_mat)

	#print "Calculating theta" 
        for i in range(1+initial_offset+base_config,number_of_degrees+initial_offset+base_config+1): ## IU trajectory
	#for i in range(1,number_of_degrees): ## Purdue trajectory
	     theta[i-1-initial_offset-base_config]=float(line[pos[i-1]+1:pos[i]])
 
        #for i in range(1,number_of_degrees-1):
	#    velocity[i-1]=float(line[pos[i-1+number_of_degrees]+1:pos[i+number_of_degrees]])
        #velocity[number_of_degrees-1] =  float(line[pos[number_of_degrees-1+number_of_degrees]+1:])   

        
        theta_Purdue=[theta[0]*pi/180, theta[1]*pi/180, theta[2]*pi/180, theta[3]*pi/180, theta[4]*pi/180, theta[5]*pi/180, theta[6]*pi/180, theta[7]*pi/180, theta[8]*pi/180, theta[9]*pi/180, theta[10]*pi/180, theta[11]*pi/180, theta[12]*pi/180, theta[13]*pi/180, theta[14]*pi/180, theta[15]*pi/180, theta[16]*pi/180, theta[17]*pi/180, theta[18]*pi/180, theta[19]*pi/180, theta[20]*pi/180, theta[21]*pi/180, theta[22]*pi/180, theta[23]*pi/180, theta[24]*pi/180, theta[25]*pi/180, theta[26]*pi/180, theta[27]*pi/180, theta[28]*pi/180, theta[29]*pi/180, theta[30]*pi/180, theta[31]*pi/180, theta[32]*pi/180, theta[33]*pi/180, theta[34]*pi/180, theta[35]*pi/180, theta[36]*pi/180, theta[37]*pi/180, theta[38]*pi/180, theta[39]*pi/180, theta[40]*pi/180, theta[41]*pi/180, theta[42]*pi/180, theta[43]*pi/180, theta[44]*pi/180, theta[45]*pi/180, theta[46]*pi/180, theta[47]*pi/180, theta[48]*pi/180, theta[49]*pi/180, theta[50]*pi/180, theta[51]*pi/180, theta[52]*pi/180, theta[53]*pi/180, theta[54]*pi/180, theta[55]*pi/180, theta[56]*pi/180]
	
	theta_IU=[theta[44], #1-1
		  theta[51], #2-1
		  theta[45], #3-1
		  theta[52], #4-1
		  theta[46], #5-1
		  theta[53], #6-1
		  theta[47], #7-1
		  theta[54], #8-1
		  theta[48], #9-1
		  theta[55], #10-1
		  theta[49], #11-1
		  theta[56], #12-1
		  theta[50], #13-1
		  theta[23], #14-1
		  theta[2],  #15-1
		  theta[24], #16-1
		  theta[3],  #17-1
		  theta[25], #18-1
		  theta[4], #19-1
		  theta[26], #20-1
		  theta[5], #21-1
		  theta[27], #22-1
		  theta[6], #23-1
		  theta[28], #24-1
		  theta[7], #25-1
		  theta[0], #26-1
		  theta[1], #27-1
		  theta[42], #28-1
		  theta[43], #29-1
		  theta[41], #30-1
		  theta[39], #31-1
		  theta[40], #32-1
		  theta[38], #33-1
		  theta[36], #34-1
		  theta[37], #35-1
		  theta[35], #36-1
		  theta[33], #37-1
		  theta[34], #38-1
		  theta[32], #39-1
		  theta[30], #40-1
		  theta[31], #41-1
		  theta[29], #42-1
		  theta[21], #43-1
		  theta[22], #44-1
		  theta[20], #45-1
		  theta[18], #46-1
		  theta[19], #47-1
		  theta[17], #48-1
		  theta[15], #49-1
		  theta[16], #50-1
		  theta[14], #51-1
		  theta[12], #52-1
		  theta[13], #53-1
		  theta[11], #54-1
		  theta[9], #55-1
		  theta[10], #56-1
		  theta[8]   ] #57-1
	theta=theta_IU
	        
	#theta=theta_Purdue	        
	#print theta


        ######################## THE SIMULATION CODE #############	
	## theta setting
        #sendSparseServoCommand(robot,{'HPY':theta[0], 'RHY':theta[1], 'LHY':theta[2],  'RHR':theta[3],  'LHR':theta[4],  'RHP':theta[5],  'LHP':theta[6],  'RKP':theta[7],  'LKP':theta[8],  'RAP':theta[9],  'LAP':theta[10],  'RAR':theta[11],  'LAR':theta[12],  'RSP':theta[13],  'LSP':theta[14],  'RSR':theta[15],  'LSR':theta[16],  'RSY':theta[17],  'LSY':theta[18],  'REP':theta[19], 'LEP':theta[20],  'RWY':theta[21],  'LWY':theta[22], 'RWP':theta[23],  'LWP':theta[24],  'HNR':theta[25], 'HNP':theta[26],  'rightIndexKnuckle2':theta[27], 'rightIndexKnuckle3':theta[28],  'rightIndexKnuckle1':theta[29],  'rightMiddleKnuckle2':theta[30], 'rightMiddleKnuckle3':theta[31],  'rightMiddleKnuckle1':theta[32],  'rightRingKnuckle2':theta[33],  'rightRingKnuckle3':theta[34],  'rightRingKnuckle1':theta[35],  'rightPinkyKnuckle2':theta[36],  'rightPinkyKnuckle3':theta[37],  'rightPinkyKnuckle1':theta[38],  'rightThumbKnuckle2':theta[39],  'rightThumbKnuckle3':theta[40],  'rightThumbKnuckle1':theta[41],  'leftIndexKnuckle2':theta[42], 'leftIndexKnuckle3':theta[43],  'leftIndexKnuckle1':theta[44],  'leftMiddleKnuckle2':theta[45],  'leftMiddleKnuckle3':theta[46],  'leftMiddleKnuckle1':theta[47],  'leftRingKnuckle2':theta[48],  'leftRingKnuckle3':theta[49], 'leftRingKnuckle1':theta[50],  'leftPinkyKnuckle2':theta[51],  'leftPinkyKnuckle3':theta[52],  'leftPinkyKnuckle1':theta[53],  'leftThumbKnuckle2':theta[54],  'leftThumbKnuckle3':theta[55],  'leftThumbKnuckle1':theta[56] })
	robot.GetController().SetDesired(theta)

	#velocity
	#robot.SetDOFVelocities(velocity) # 
        time.sleep(step_time)
    file_read .close()


