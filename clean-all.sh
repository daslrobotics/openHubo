#!/bin/bash

BASE_DIR=`git rev-parse --show-toplevel`

cd $BASE_DIR/openmr/
echo `pwd`
[ ! -d build ] || rm -rf build

cd $BASE_DIR/forceSensor
echo `pwd`
[ ! -d build ] || rm -rf build

for pkg in generalik cbirrt2 manipulation2
do
    cd $BASE_DIR/comps-plugins/$pkg
echo `pwd`
    [ ! -d build ] || rm -rf build
done

