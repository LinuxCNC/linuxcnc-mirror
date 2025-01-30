#!/bin/bash

set -e

rm -f gcode-output

if nc -z localhost 5007; then
    echo "Process already listening on port 5007. Exiting"
    exit 1
fi

linuxcnc -r linuxcncrsh-test.ini &


# let linuxcnc come up
TOGO=80
while [  $TOGO -gt 0 ]; do
    echo "trying to connect to linuxcncrsh TOGO=$TOGO"
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

# test set command by
# - getting the key's old value before setting
# - setting the new value
# - getting the new value
# - collect possible error from linuxcncsvr
function testSet() {
    # get before value
    echo "get $1"
    # set cmd
    echo "set $*"
    # get after value
    echo "get $1"
    # get error from server (or OK)
    echo "get error"
}

# get command with collecting possible error from linuxcncsvr
function testGet() {
    cmd="$@"
    echo "get $*"
    echo "get error"
}

(
    # initialize
    echo hello EMC mt 1.0
    testSet enable EMCTOO
    testGet debug
    testSet verbose on
    testSet echo off
    # ask linuxcncrsh to not read the next command until it's done running
    # the current one
    # shellcheck disable=SC1010
    testSet wait_mode done
    # test deprecation mode of set_wait -> wait_mode rename
    # shellcheck disable=SC1010
    testSet set_wait done

    # check default global settings
    testGet plat
    testGet update

    # check default global settings
    echo get plat

    # test commands that fail when machine not running
    testSet mode mdi

    # prepare machine
    testSet estop off
    testSet machine on

    # test if probing in manual mode fails
    testSet mode manual
    testGet pos_offset
    testGet probe_tripped
    testGet probe_value
    testGet probe_clear
    testSet probe 0 0 0                    # <x> <y> <z>
    testSet probe_clear

    # test if probing before homing fails
    testSet mode mdi
    testSet probe 0 0 0                    # <x> <y> <z>
    testSet probe_clear

    # do homing
    testSet mode manual
    testSet home 0                         # <Axis No>
    testSet home 1
    testSet home 2
    testSet home 3
    testSet home 4
    testSet home 5

    # test if probing without teleop fails
    testSet mode mdi
    testSet probe 0 0 0                    # <x> <y> <z>
    testSet probe_clear

    # test probing
    testSet mode manual
    testSet teleop_enable on
    testSet mode mdi
    testSet probe 0 0 0                    # <x> <y> <z>
    testSet probe_clear

    # test spindle command
    testSet mode manual
    testSet spindle forward                # turn on all w/o param
    testSet spindle off -1                 # turn off all w/param
    testGet spindle -1                     # check all w/param
    testSet spindle forward 99             # turn on illegal spindle

    # test brake command
    testSet brake on                       # turn on all w/o param
    testSet brake off -1                   # turn off all w/param
    testGet brake -1                       # check all w/param
    testSet brake forward 99               # turn on illegal spindle

    # test pause
    testSet wait_mode received             # otherwise pause will stall
    testSet pause
    # shellcheck disable=SC1010
    testSet wait_mode done
    testSet resume

    # test g-code
    echo set mode mdi
    echo set mdi m100 p-1 q-2
    sleep 1

    # here comes a big blob
    dd bs=4096 if=lots-of-gcode
    echo set mdi m100 p-3 q-4

    # test misc. get commands
    testGet error
    testGet abs_act_pos
    testGet abs_cmd_pos
    testGet angular_unit_conversion
    testGet comm_mode
    testGet comm_prot
    testGet display_angular_units
    testGet display_linear_units
    testGet error
    testGet feed_override
    testGet flood
    testGet joint_fault
    #testGet joint_homed
    testGet joint_limit
    testGet joint_pos
    testGet joint_type
    testGet joint_units
    testGet kinematics_type
    testGet linear_unit_conversion
    testGet mist
    testGet operator_display
    testGet operator_text
    testGet optional_stop
    testGet override_limits
    testGet program
    testGet program_angular_units
    testGet program_codes
    testGet program_line
    testGet program_linear_units
    testGet program_status
    testGet program_units
    testGet rel_act_pos
    testGet rel_cmd_pos
    testGet wait_mode
    testGet teleop_enable
    testGet timeout
    testGet tool
    testGet tool_offset
    testGet update
    testGet user_angular_units
    testGet user_linear_units

    # test misc. set commands
    testSet comm_mode ascii                # <Ascii | Binary>
    testSet comm_prot 1.0
    testSet abort
    testSet angular_unit_conversion deg    # <Deg | Rad | Grad | Auto | Custom>
    testSet feed_override 100              # <Percent>
    testSet flood on                       # <On | Off>
    testSet jog 0 1                        # <Axis No, Speed>
    testSet jog_incr 0 1 1                 # <Axis No, Speed, Distance>
    testSet jog_stop 0                     # <Joint No|Axis letter>
    testSet linear_unit_conversion mm      # <Inch | CM | MM | Auto | Custom>
    #testSet load_tool_table <Table name>
    testSet mist on                        # <On | Off>
    #testSet Open <File path / name>
    testSet optional_stop 1                # <none | 0 | 1>
    testSet override_limits off            # <On | Off>
    #testSet run <Line No>
    testSet step
    testSet task_plan_init
    testSet teleop_enable
    testSet timeout 10
    testSet tool_offset 1
    testSet update auto                    # <None | Auto>
    testSet wait 1                         # <Time>

    echo shutdown
) | nc -v localhost 5007 > telnet-output


# wait for linuxcnc to finish
wait

exit 0
