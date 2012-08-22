#!/bin/bash

[[ -d ikfast ]] || mkdir ikfast

openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftArmManip
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightArmManip
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftFootManip --freejoint='TY'
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightFootManip
openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=affineManip

#Find cached file names from previous generation 
#TODO: error checks here
LA_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftArmManip --getfilename` 
RA_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightArmManip --getfilename`
LL_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=leftFootManip --getfilename` 
RL_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=rightFootManip --getfilename`
AFF_IK_FILE=`openrave.py --database inversekinematics --robot=./jaemiHubo.planning.robot.xml --manipname=affineManip --getfilename`

#Not sure if there's a better way to do this, since the plugins are always hashed, but at least this names things conveniently
cp $LA_IK_FILE ./ikfast/leftArmManip.ikfast.so
cp $RA_IK_FILE ./ikfast/rightArmManip.ikfast.so
cp $LL_IK_FILE ./ikfast/leftFootManip.ikfast.so
cp $RL_IK_FILE ./ikfast/rightFootManip.ikfast.so
cp $AFF_IK_FILE ./ikfast/affineManip.ikfast.so
