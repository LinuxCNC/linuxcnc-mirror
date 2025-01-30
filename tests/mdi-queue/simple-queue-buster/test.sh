#!/bin/bash

rm -f gcode-output

if ! command -v nc ; then
    echo "E: Binary 'nc' not in PATH or not installed."
    exit 1
fi

if ! command -v linuxcnc ; then
    echo "E: Binary 'linuxcnc' not in PATH or not installed."
    exit 1
fi

if nc -z localhost 5007; then
    echo "E: Process already listening on port 5007. Exiting"
    exit 1
fi

linuxcnc -r linuxcncrsh-test.ini &


# let linuxcnc come up
TOGO=80
while [  $TOGO -gt 0 ]; do
    echo "I: trying to connect to linuxcncrsh TOGO=$TOGO"
    if nc -z localhost 5007; then
        break
    fi
    sleep 0.25
    TOGO=$((TOGO - 1))
done
if [  $TOGO -eq 0 ]; then
    echo connection to linuxcncrsh timed out
    exit 1
fi

# switch back and forth between tool 1 and tool 2 every few MDI calls
rm -f expected-gcode-output lots-of-gcode
echo "P is -1.000000" >> expected-gcode-output
NUM_MDIS=1
NUM_MDIS_LEFT=$NUM_MDIS
TOOL=1
for i in $(seq 0 1000); do
    NUM_MDIS_LEFT=$((NUM_MDIS_LEFT - 1))
    if [ $NUM_MDIS_LEFT -eq 0 ]; then
        echo "set mdi t$TOOL m6" >> lots-of-gcode
        if [ $TOOL -eq 1 ]; then
            TOOL=2
        else
            TOOL=1
        fi

        NUM_MDIS=$((NUM_MDIS + 1))
        if [ $NUM_MDIS -gt 10 ]; then
            NUM_MDIS=1
        fi

        NUM_MDIS_LEFT=$NUM_MDIS
    fi
    echo "set mdi m100 p$i" >> lots-of-gcode
    echo "P is $i.000000" >> expected-gcode-output
done
echo "P is -2.000000" >> expected-gcode-output

(
    echo "hello EMC mt 1.0"
    echo "set enable EMCTOO"

    # ask linuxcncrsh to not read the next command until it's done running
    # the current one
    #echo "set set_wait done"

    echo "set mode manual"
    echo "set estop off"
    echo "set machine on"

    echo "set mode mdi"
    echo "set mdi m100 p-1"
    echo "set wait done"

    # here comes a big blob
    dd bs=4096 if=lots-of-gcode

    echo "set mdi m100 p-2"
    echo "set wait done"

    echo "shutdown"
) | nc localhost 5007


# wait for linuxcnc to finish
wait

exit 0

