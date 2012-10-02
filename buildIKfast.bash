#!/bin/bash

[[ -d ikfast ]] || mkdir ikfast

#TODO: sanitize arguments to avoid messy errors
openrave.py --database inversekinematics --robot=./$1 --manipname=leftArmManip $2
openrave.py --database inversekinematics --robot=./$1 --manipname=rightArmManip $2
openrave.py --database inversekinematics --robot=./$1 --manipname=leftFootManip $2
openrave.py --database inversekinematics --robot=./$1 --manipname=rightFootManip $2

#Find cached file names from previous generation 

#TODO: error checks here
LA_IK_FILE=`openrave.py --database inversekinematics --robot=./$1 --manipname=leftArmManip --getfilename` 
RA_IK_FILE=`openrave.py --database inversekinematics --robot=./$1 --manipname=rightArmManip --getfilename`
LL_IK_FILE=`openrave.py --database inversekinematics --robot=./$1 --manipname=leftFootManip --getfilename` 
RL_IK_FILE=`openrave.py --database inversekinematics --robot=./$1 --manipname=rightFootManip --getfilename`

#Not sure if there's a better way to do this, since the plugins are always hashed, but at least this names things conveniently
ln -s $LA_IK_FILE ./ikfast/leftArm.ikfast.so
ln -s $RA_IK_FILE ./ikfast/rightArm.ikfast.so
ln -s $LL_IK_FILE ./ikfast/leftFoot.ikfast.so
ln -s $RL_IK_FILE ./ikfast/rightFoot.ikfast.so
