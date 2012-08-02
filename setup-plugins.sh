#!/bin/bash

git submodule update --init
cd openmr/plugin
cmake .
make
sudo make install
cd ../../forceSensor
cmake .
make
sudo make install
