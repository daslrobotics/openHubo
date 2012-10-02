#!/bin/bash

export OPENHUBO_DIR=/home/robert/openHubo

if [[ ${#OPENRAVE_PLUGINS} == 0 ]]
then
    export OPENRAVE_PLUGINS=$OPENHUBO_DIR/plugins
else

    export OPENRAVE_PLUGINS=$OPENRAVE_PLUGINS:$OPENHUBO_DIR/plugins
fi

source `openrave-config --share-dir`/openrave_completion.bash
complete -F "_complete_openravepy" -o filenames -o plusdirs "openrave"

export PYTHONPATH=$PYTHONPATH:$OPENHUBO_DIR/comps-plugins/python:$OPENHUBO_DIR/python
export MATLABPATH=`echo $MATLABPATH: | sed 's,^:,,g'`$OPENHUBO_DIR/comps-plugins/matlab:$OPENHUBO_DIR/matlab
