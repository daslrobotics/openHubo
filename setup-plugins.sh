#!/bin/bash

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

BASE_DIR=`git rev-parse --show-toplevel`


#Check if submodules have changed and ask user to decide whether to checkout
#specified version in parent repository.
FOUND_PLUGIN_CHANGES=$(git submodule foreach --quiet 'git diff')

if [ ${#FOUND_PLUGIN_CHANGES} -gt 0 ]
then 
    echo "-----------------------------------------------------"
    echo "Would you like to update dependent submodules to the "
    echo "version specified by this openHubo version?"
    echo ""
    echo "Choose Yes (y) if this is the first time you're building the"
    echo "   plugins, or if you have recently pulled updates to openHubo from github"
    echo "Choose No (n) if you are making your own changes and want to build the plugins as-is"
    read -p "Enter your choice [Y/n]?" SUBMODULE_UPDATE
    if [[ $SUBMODULE_UPDATE == 'n' || $SUBMODULE_UPDATE == 'N' ]]
    then
        echo "Submodule update skipped, Using present versions of dependent plugins..."
    else
        echo "Updating plugin submodules to match the current openHubo commit..."
        git submodule update --init
    fi
fi

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

ENV_SOURCED_SEARCH=`grep $BASE_DIR/env.sh ~/.bashrc`

if [[ ${#ENV_SOURCED_SEARCH} == 0 ]]
then
    echo "source $BASE_DIR/env.sh" >> ~/.bashrc
fi
 
#Strip out base folder definition and update with current
sed -i "s,\( OPENHUBO_DIR=\).*,\1," env.sh
sed -i "s,\( OPENHUBO_DIR=\),\1$BASE_DIR," env.sh
echo ""
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
