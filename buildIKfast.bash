#!/bin/bash

[[ -d ikfast ]] || mkdir ikfast

#TODO: sanitize arguments to avoid messy errors
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftArmManip $1
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightArmManip $1
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftFootManip --freejoint='TY' $1
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightFootManip $1
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=affineManip $1

#Find cached file names from previous generation 

#TODO: error checks here
LA_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftArmManip --getfilename` 
RA_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightArmManip --getfilename`
LL_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftFootManip --getfilename` 
RL_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightFootManip --getfilename`
AFF_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=affineManip --getfilename`

#Not sure if there's a better way to do this, since the plugins are always hashed, but at least this names things conveniently
ln -s $LA_IK_FILE ./ikfast/leftArm.ikfast.so
ln -s $RA_IK_FILE ./ikfast/rightArm.ikfast.so
ln -s $LL_IK_FILE ./ikfast/leftFoot.ikfast.so
ln -s $RL_IK_FILE ./ikfast/rightFoot.ikfast.so
ln -s $AFF_IK_FILE ./ikfast/affine.ikfast.so
