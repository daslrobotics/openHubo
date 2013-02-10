#!/bin/bash
gnome-terminal --tab --working-directory=$OPENHUBO_DIR/$1/src -t Source --tab --working-directory=$OPENHUBO_DIR/$1/build -t Build --profile=Build
