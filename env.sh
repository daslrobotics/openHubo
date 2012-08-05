#!/bin/bash

if [[ ${#OPENRAVE_PLUGINS} == 0 ]]
then
    export OPENRAVE_PLUGINS=~/openHubo/plugins
else

    export OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:~/openHubo/plugins
fi
