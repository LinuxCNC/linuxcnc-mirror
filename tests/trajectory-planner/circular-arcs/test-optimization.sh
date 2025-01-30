#!/bin/bash

set -e

function say_done {
    espeak "done" 2> /dev/null
}
set -o monitorA

./build-debug.sh
cp position.blank position.txt
linuxcnc -r circular_arcs.ini > test.log &
./machine_setup.py "$1" && say_done
fg
./save_activate.sh test.log
