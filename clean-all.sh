#!/bin/bash

BASE_DIR=`git rev-parse --show-toplevel`

cd $BASE_DIR/openmr/
[ -d build ] || rm -rf build

cd $BASE_DIR/forceSensor
[ -d build ] || rm -rf build


for pkg in generalik cbirrt2 manipulation2
do
    cd $BASE_DIR/comps-plugins/$pkg
    [ -d build ] || rm -rf build
done

