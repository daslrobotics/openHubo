#!/bin/bash

export OPENHUBO_DIR=/home/robert/openHubo

if [[ ${#OPENRAVE_PLUGINS} == 0 ]]
then
    export OPENRAVE_PLUGINS=$OPENHUBO_DIR/plugins
else

    export OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:$OPENHUBO_DIR/plugins
fi
