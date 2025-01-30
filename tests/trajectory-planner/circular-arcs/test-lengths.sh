#!/bin/bash
set -o monitor
./build-debug.sh
cp position.blank position.txt
linuxcnc "$1" > test.log &
./machine_setup.py "$2"
fg
./save_lengths.sh test.log
#if [ -a length_data.log ] 
#then
    #octave/plot_movement.m
#fi
