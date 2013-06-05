#!/bin/bash

#Make sure to run within the git directory
#TODO: error if not run within git dir?
BASE_DIR=`git rev-parse --show-toplevel`

cd $BASE_DIR
for l in `find plugins -name lib*.so`
do
    rm $l
done

git submodule deinit comps-plugins
git submodule deinit openmr

if [ -f openhubo ]
then
    echo -e "\nRemoving local openhubo symlink from master\n"
    unlink openhubo
fi

if [ -d python ]
then
    echo -e "\nUPGRADE WARNING: Renaming old python folder to python.bak, please move any custom python code out of this folder!\n"
    mv python python.bak
fi

for pkg in comps-plugins/generalik comps-plugins/cbirrt2 comps-plugins/manipulation2 servocontroller forceSensor
do
    cd $BASE_DIR/$pkg
    echo "Removing build folder in "`pwd`
    [ -d build ] && rm -rf build
done
