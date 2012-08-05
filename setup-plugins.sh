#!/bin/bash

BASE_DIR=`git rev-parse --show-toplevel`

git submodule update --init
cd $BASE_DIR/openmr/
[ -d build ] || mkdir build
cd build
cmake ../src/
make
make install

cd $BASE_DIR/forceSensor
cmake .
make
make install

for pkg in generalik cbirrt2 manipulation2
do
    cd $BASE_DIR/comps-plugins/$pkg
    [ -d build ] || mkdir build
    cd build
    cmake ../
    make
    make install
done

cd $BASE_DIR

SOURCED_SEARCH=`grep $BASE_DIR/env.sh ~/.bashrc`

#Setup base folder in env script
#sed "s,\$BASE_DIR,$BASE_DIR,g" <"./misc/plugins_folder" >env.sh

if [[ ${#SOURCED_SEARCH} == 0 ]]
then
    echo "source $BASE_DIR/env.sh" >> ~/.bashrc
fi

source env.sh

for f in `ls plugins/*.so`
do
    PLUGIN_CHECK=`openrave --listplugins | grep $f`
    if [[ ${#PLUGIN_CHECK} == 0 ]]
    then
        echo "$f NOT found by OpenRAVE, please check that the plugins dir is properly sourced."
    else
        echo "$f found by OpenRAVE"
    fi
done
