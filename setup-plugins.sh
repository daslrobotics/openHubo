#!/bin/bash

function verify-dep()
{
    local RESULT=`dpkg-query -l $1 | grep "ii"`
    local INSTALL_MISSING_DEP=''
    if [[ ${#RESULT} == 0 ]]
    then
        read -p "Install missing dependency $1 [Y/n]?" INSTALL_MISSING_DEP
        if [[ $INSTALL_MISSING_DEP == 'n' || $INSTALL_MISSING_DEP == 'N' ]]
        then
            echo "WARNING: skipping dependency $1..."
        else
            sudo apt-get install $1
        fi
    else
        echo "Dependency $1 is installed..."
    fi
}

BASE_DIR=`git rev-parse --show-toplevel`

#git submodule update --init
cd $BASE_DIR/openmr/
[ -d build ] || mkdir build
cd build
cmake ../src/
make
make install

cd $BASE_DIR/forceSensor
[ -d build ] || mkdir build
cd build
cmake ../
make
make install

#CoMPS prereqs and build steps

for dep in libqhull-dev libqhull5 libnewmat10-dev libnewmat10ldbl
do
    verify-dep $dep
done

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
 
#Strip out base folder definition and update with current
sed -i "s,\( OPENHUBO_DIR=\).*,\1," env.sh
sed -i "s,\( OPENHUBO_DIR=\),\1$BASE_DIR," env.sh
echo "OpenHubo base folder is $BASE_DIR"
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
