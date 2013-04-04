if ~exist('BOXLIFT_LOADED')
    close all;

    %load the robot into the environment
    orEnvLoadScene('jaemiHubo.planning.robot.xml',1);
    robotid = orEnvGetBody('jaemiHubo');

    %Load any miscellaneous models
    objid = orEnvCreateKinBody('liftbox','kinbody/box.kinbody.xml');

    %set printing and display options
    orEnvSetOptions('debug 3')
    orEnvSetOptions('collision ode')

    manips = orRobotGetManipulators(robotid);
    jointdofs = 0:orRobotGetActiveDOF(robotid);
    %This should be left and right arms...check with robot def.
    activedofs = [manips{1}.armjoints,manips{2}.armjoints];

    %set initial configuration

    BOXLIFT_LOADED=1;
    disp('Box lifting environment with Jaemi Hubo loaded')

else
    disp('Environment already loaded, clear BOXLIFT_LOADED to reload')
end

%% Load the environment and setup id's and such. The envload script should
%bypass this if the envrionment is already loaded.
orEnvSetOptions('debug 3')

row4=[0 0 0 1]; %Dummy last row of 4x4 transformation matrices

Tedge=[eye(3),[0;.0675;0];row4];
Tbox=[eye(3),[.29;0;-0.1];row4];

%Define initial lift positions of arm end effectors. These should be using the
%palm dummy models with conveniently aligned coordinate systems.
TInitRight=Tedge^-1*Tbox
TInitLeft=Tedge*Tbox

%Define the lift motion relative to current pose
Tlift=[eye(3),[0.0;0.0;0.15];row4];
%f r u

orBodySetTransform(objid,Tbox(1:3,4));

%Create problem instances
probs.manip = orEnvCreateProblem('Manipulation','jaemiHubo');
probs.cbirrt = orEnvCreateProblem('CBiRRT','jaemiHubo');

%get the descriptions of the robot's manipulators
manips = orRobotGetManipulators(robotid);

%This should be left and right arms
activedofs = [manips{1}.armjoints,manips{2}.armjoints];

%set initial configurations for IK solver and graping pose. The initial IK pose
%is chosen primarily to avoid singularities (i.e. gimbal lock on
%shoulders/elbows/knees). The actual initial pose can be chosen somewhat
%arbitrarily, as the CBiRRT planner will accept any valid initial condition.
initIKPose = [-pi/8  pi/4 0.000 -0.5000 0.0000 0.0000 0.0000 -pi/8 -pi/4 0 -0.5 0 0 0];  
initDOFValues = [0.000 0.500 0.500 -2*pi/3 0.0000 0.0000 0.0000 0.000 -0.500 -0.50 -2*pi/3 0 0 0];  
orRobotSetDOFValues(robotid,initIKPose,activedofs);

disp('Initial Pose Set...');
pause(1)

%get the ik solutions for both arms for the box in the start pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
startik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(TInitLeft),GetTrans(TInitLeft)])],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
startik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(TInitRight),GetTrans(TInitRight)])],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
startik = [startik0 ' ' startik1];
orRobotSetDOFValues(robotid,str2num(startik));

disp('Solved first grasping pose...');
pause(1)

%% Define initial and goal positions

TGoalLeft=Tlift*TInitLeft;
TGoalRight=Tlift*TInitRight;

%define the target transform of the goal
TBoxGoal=Tlift*Tbox;
orBodySetTransform(objid,TBoxGoal);

%get the ik solutions for both arms for the box in the goal pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
goalik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(TGoalLeft),GetTrans(TGoalLeft)]) ],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
goalik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(TGoalRight),GetTrans(TGoalRight)]) ],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
goalik = [goalik0 ' ' goalik1];
orRobotSetDOFValues(robotid,str2num(goalik));

disp('Solved final grasping pose...');
%pause

%reset the box to the start pose
orBodySetTransform(objid,Tbox(1:3,4));

%plan a reaching motion to grab the box
orRobotSetDOFValues(robotid,initDOFValues,activedofs);
pause(1);

orProblemSendCommand(['RunCBiRRT filename reaching.traj.txt jointgoals '  num2str(numel(str2num(startik))) ' ' num2str(startik) ],probs.cbirrt);
%execute the planned trajectory
orProblemSendCommand(['traj reaching.traj.txt'],probs.cbirrt);
orEnvWait(robotid);

disp('Arms in initial pose');
pause(1)

Tw_e0=Tedge;
Tw_e1=Tedge^-1;

%specify a TSR for manipulator 0 to keep the box from tilting during the motion
TSRstring0 = SerializeTSR(0,'NULL',Tbox,Tw_e0,[-10 10  -10 10  -10 10  0 0 0 0 -pi/2 pi/2])

%specify a TSR for manipulator 1 to keep its end-effector fixed relative to the box
TSRstring1 = SerializeTSR(1,'liftbox body',eye(4),Tw_e1,zeros(1,12))

%pack the TSRs into a string, specifying them as constraint TSR's
TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring0,'NULL',[]) ' ' SerializeTSRChain(0,0,1,1,TSRstring1,'NULL',[])]

%grab the box with manipulator 0
orProblemSendCommand(['setactivemanip index 0'],probs.manip)
orProblemSendCommand(['GrabBody name liftbox'],probs.manip)

disp('Box Grabbed')
[collision,colbodyid,contacts]=orEnvCheckCollision(robotid)

%% Plan the lifting motion
goals_in = [str2num(goalik)];
orProblemSendCommand(['RunCBiRRT filename lift.traj.txt timelimit 20 smoothingitrs 100 jointgoals '  num2str(numel(goals_in)) ' ' num2str(goals_in) ' ' TSRChainString],probs.cbirrt)
[collision,colbodyid,contacts]=orEnvCheckCollision(robotid)

%execute the planned trajectory
orProblemSendCommand(['traj lift.traj.txt'],probs.cbirrt)
orEnvWait(robotid);

orProblemSendCommand(['setactivemanip index 0'],probs.manip)
orProblemSendCommand(['ReleaseAll'],probs.manip)
disp('Box released')


