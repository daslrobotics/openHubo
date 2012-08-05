#!/bin/bash

BASE_DIR=`git rev-parse --show-toplevel`

git submodule update --init
cd $BASE_DIR/openmr/
[ -d build ] || mkdir build
cd build
cmake ../src/
make
sudo make install

cd $BASE_DIR/forceSensor
cmake .
make
sudo make install

for pkg in generalik cbirrt2 manipulation2
do
   cd $BASE_DIR/comps-plugins/$pkg
   [ -d build ] || mkdir build
   cd build
   cmake ../
   make
   sudo make install
done

cd $BASE_DIR

SOURCED_SEARCH=`grep OPENRAVE_PLUGINS ~/.bashrc`

if [[ ${#SOURCED_SEARCH} == 0 ]]
then
    echo 'OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:'$BASE_DIR'/plugins' >> ~/.bashrc
fi

export OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:$BASE_DIR/plugins
