#!/bin/bash

#Make sure to run within the git directory
#TODO: error if not run within git dir?
BASE_DIR=`git rev-parse --show-toplevel`

cd $BASE_DIR
rm plugins/lib*.so

for pkg in comps-plugins/generalik comps-plugins/cbirrt2 comps-plugins/manipulation2 openmr forceSensor
do
    cd $BASE_DIR/$pkg
    echo "Removing build folder in "`pwd`
    [ -d build ] && rm -rf build
done

