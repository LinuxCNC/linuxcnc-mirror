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
    echo "E: connection to linuxcncrsh timed out"
    exit 1
fi

# switch back and forth between tool 1 and tool 2 every few MDI calls
rm -f expected-gcode-output lots-of-gcode
echo "P is -100.000000" >> expected-gcode-output
NUM_MDIS=1
NUM_MDIS_LEFT=$NUM_MDIS
TOOL=1
for i in $(seq 0 1000); do
    NUM_MDIS_LEFT=$((NUM_MDIS_LEFT - 1))
    if [ $NUM_MDIS_LEFT -eq 0 ]; then
        echo "set mdi o<queue-buster> call [$TOOL]" >> lots-of-gcode
        {
        echo "P is 12345.000000"
        echo "P is $((-1 * TOOL)).000000"
        echo "P is 54321.000000"
        }  >> expected-gcode-output

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
echo "P is -200.000000" >> expected-gcode-output

(
    echo "hello EMC mt 1.0"
    echo "set enable EMCTOO"

    echo "set mode manual"
    echo "set estop off"
    echo "set machine on"

    echo "set mode auto"
    echo "set open dummy.ngc"

    echo "set mode mdi"
    echo "set mdi m100 p-100"
    echo "set wait done"

    # here comes a big blob
    dd bs=4096 if=lots-of-gcode

    echo "set mdi m100 p-200"
    echo "set wait done"

    echo shutdown
) | nc localhost 5007


# wait for linuxcnc to finish
wait

exit 0

