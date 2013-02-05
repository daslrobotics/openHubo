#!/bin/bash

#Useful functions for setup script
function verify-dep()
{
    local RESULT=`dpkg-query -l $1 | grep "ii"`
    local INSTALL_MISSING_DEP=''
    if [[ ${#RESULT} == 0 ]]
    then
        read -p "Install missing dependent package/library $1 [Y/n]?" INSTALL_MISSING_DEP
        if [[ $INSTALL_MISSING_DEP == 'n' || $INSTALL_MISSING_DEP == 'N' ]]
        then
            echo "WARNING: skipping dependent package/library $1..."
        else
            sudo apt-get install $1
        fi
    else
        echo "package/library $1 is installed"
    fi
}

function check_submodules()
{
    #Check if submodules have changed and ask user to decide whether to checkout
    #specified version in parent repository.
    git submodule init

    OLD_IFS=$IFS
    IFS=$'\n'

    for stat in `git submodule status`
    do
        local PLUGIN_STATUS=`echo $stat | cut -c 1`
        local PLUGIN_NAME=`echo $stat | awk '{print $2}'`

        if [[ $PLUGIN_STATUS == '-' ]]
        then
            git submodule update $PLUGIN_NAME
        fi
        if [[ $PLUGIN_STATUS == '+' ]]
        then
            echo "-----------------------------------------------------"
            echo "Submodule $PLUGIN_NAME has been changed from the default commit specified in this repository."
            echo "Would you like to revert from the current state to the default commit?"
            echo "Choose No (n) if you are making your own changes and want to build the plugins as-is"
            read -p "Enter your choice [Y/n]?" SUBMODULE_UPDATE
            if [[ $SUBMODULE_UPDATE == 'n' || $SUBMODULE_UPDATE == 'N' ]]
            then
                echo "Submodule update skipped..."
            else
                echo "Updating plugin $PLUGIN_NAME to match the current openHubo commit..."
                git submodule update $PLUGIN_NAME
            fi
        fi
    done
    IFS=$OLD_IFS
}

#Install script starts here

BASE_DIR=`git rev-parse --show-toplevel`

check_submodules

echo ""
echo "Building OpenMR Servo Controller Plugin..."
echo ""

cd $BASE_DIR/openmr/
[ -d build ] || mkdir build
cd build
cmake ../src/
make
make install

echo ""
echo "Building OpenGRASP ForceSensor plugin..."
echo ""

cd $BASE_DIR/forceSensor
[ -d build ] || mkdir build
cd build
cmake ../
make
make install

echo ""
echo "Building CoMPS plugins..."
echo ""

#CoMPS prereqs and build steps

for dep in libqhull-dev libqhull5 libnewmat10-dev libnewmat10ldbl libboost-regex-dev
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

ENV_SOURCED_SEARCH=`grep $BASE_DIR/env.sh ~/.bashrc`

if [[ ${#ENV_SOURCED_SEARCH} == 0 ]]
then
    echo "source $BASE_DIR/env.sh" >> ~/.bashrc
fi
 
#Strip out base folder definition and update with current
sed "s,\( OPENHUBO_DIR=\),\1$BASE_DIR," .env.template > env.sh
echo ""
echo "OpenHubo base folder is $BASE_DIR"
source env.sh

PLUGINS_LIST=`openrave.py --listplugins`

for f in `ls plugins/*.so`
do
    PLUGIN_CHECK=`echo $PLUGINS_LIST | grep $f`
    if [[ ${#PLUGIN_CHECK} == 0 ]]
    then
        echo "$f NOT found by OpenRAVE, please check that the plugins dir is properly sourced."
    else
        echo "$f found by OpenRAVE"
    fi
done
