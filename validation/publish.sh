#!/bin/bash
DEST_FOLDER=~/Dropbox/research/hubo_logs/mass_validation
rm *.traj
openhubo test_bodytilt.py --robot ../robots/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml --no-interact
rm $DEST_FOLDER/*.traj
cp *.traj $DEST_FOLDER/
