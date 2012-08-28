#!/bin/bash

[[ -d ikfast ]] || mkdir ikfast

#TODO: sanitize arguments to avoid messy errors
openrave.py --database inversekinematics --robot=./leftarm.robot.xml --freejoint='LSY' $1
openrave.py --database inversekinematics --robot=./leftleg.robot.xml $1
openrave.py --database inversekinematics --robot=./rightarm.robot.xml  --freejoint='RSY' $1
openrave.py --database inversekinematics --robot=./rightleg.robot.xml $1

#Find cached file names from previous generation 

#TODO: error checks here
LA_IK_FILE=`openrave.py --database inversekinematics --robot=leftarm.robot.xml --getfilename`
LL_IK_FILE=`openrave.py --database inversekinematics --robot=leftleg.robot.xml --getfilename`
RA_IK_FILE=`openrave.py --database inversekinematics --robot=rightarm.robot.xml --getfilename`
RL_IK_FILE=`openrave.py --database inversekinematics --robot=rightleg.robot.xml --getfilename`

#This is a rather ugly way to load ik plugins, since it bypasses the systematic
#naming scheme, but until the GetURI bug is fixed or the model is moved to
#Collada, this is easier.
echo $LL_IK_FILE
echo $RL_IK_FILE

ln -sf "$LA_IK_FILE" ./ikfast/leftArmReverse.ikfast.so
ln -sf "$RA_IK_FILE" ./ikfast/rightArmReverse.ikfast.so
ln -sf "$LL_IK_FILE" ./ikfast/leftFootReverse.ikfast.so
ln -sf "$RL_IK_FILE" ./ikfast/rightFootReverse.ikfast.so
