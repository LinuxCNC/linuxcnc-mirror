#!/bin/bash
set -x

rm -f sim.var

# reset the tool table to a known starting configuration
rm -f simpockets.tbl
cp ../../simpockets.tbl.original simpockets.tbl

rm -f gcode-output

if nc -z localhost 5007; then
    echo "E: Process already listening on port 5007. Exiting"
    exit 1
fi

linuxcnc -r sim.ini &


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
    echo "I: connection to linuxcncrsh timed out"
    exit 1
fi


(
    function introspect() {
        SEQUENCE_NUMBER=$1
        echo "set mdi m100 P6 Q$SEQUENCE_NUMBER"  # sequence number
        echo "set mdi m100 P0 Q#5420"             # X
        echo "set mdi m100 P1 Q#5421"             # Y
        echo "set mdi m100 P2 Q#5422"             # Z
        echo "set mdi m100 P3 Q#5400"             # toolno
        echo "set mdi m100 P4 Q#5403"             # TLO z
        echo "set mdi m100 P5"                    # blank line
    }

    echo "hello EMC mt 1.0"
    echo "set enable EMCTOO"

    echo "set estop off"
    echo "set machine on"
    echo "set mode mdi"

    introspect 0

    echo "set mdi t1 m6"
    introspect 1

    echo "set mdi g43"
    introspect 2

    echo "set mdi g10 l10 p1 z.1"
    introspect 3

    echo "set mdi g43"
    introspect 4

    echo "set mdi g10 l10 p10 z.15"
    introspect 5

    echo "set mdi g43"
    introspect 6

    echo "set mdi g10 l10 p99999 z.2"
    introspect 7

    echo "set mdi g43"
    introspect 8


    echo "set mdi t10 m6"
    introspect 9

    echo "set mdi g43"
    introspect 10

    echo "set mdi g10 l10 p1 z.103"
    introspect 11

    echo "set mdi g43"
    introspect 12

    echo "set mdi g10 l10 p10 z.1035"
    introspect 13

    echo "set mdi g43"
    introspect 14

    echo "set mdi g10 l10 p99999 z.104"
    introspect 15

    echo "set mdi g43"
    introspect 16


    echo "set mdi t99999 m6"
    introspect 17

    echo "set mdi g43"
    introspect 18

    echo "set mdi g10 l10 p1 z.3"
    introspect 19

    echo "set mdi g43"
    introspect 20

    echo "set mdi g10 l10 p10 z.35"
    introspect 21

    echo "set mdi g43"
    introspect 22

    echo "set mdi g10 l10 p99999 z.4"
    introspect 23

    echo "set mdi g43"
    introspect 24


    echo "set mdi t1 m6"
    introspect 25

    echo "set mdi g43"
    introspect 26


    echo "set mdi t10 m6"
    introspect 27

    echo "set mdi g43"
    introspect 28


    echo "set mdi t99999 m6"
    introspect 29

    echo "set mdi g43"
    introspect 30


    # wait for linuxcnc to finish
    echo "set wait done"

    echo "shutdown"
) | nc localhost 5007


# wait for linuxcnc to finish
wait

exit 0

