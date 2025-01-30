#!/bin/bash
set -x

rm -f sim.var

# reset the tool table to a known starting configuration
rm -f tool.tbl
cp tool.tbl.original tool.tbl

rm -f gcode-output

if nc -z localhost 5007; then
    echo "Process already listening on port 5007. Exiting"
    exit 1
fi

linuxcnc -r sim.ini &


# let linuxcnc come up
TOGO=80
while [  $TOGO -gt 0 ]; do
    echo trying to connect to linuxcncrsh TOGO=$TOGO
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


(
    function introspect() {
        SEQUENCE_NUMBER=$1
        echo 'set wait done'
        echo "set mdi m100 P6 Q$SEQUENCE_NUMBER"  # sequence number
        echo 'set mdi m100 P0 Q#5420'             # X
        echo 'set mdi m100 P1 Q#5421'             # Y
        echo 'set mdi m100 P2 Q#5422'             # Z
        echo 'set mdi m100 P3 Q#5400'             # toolno
        echo 'set mdi m100 P4 Q#5403'             # TLO z
        echo 'set mdi m100 P5'                    # blank line
        echo 'set wait done'
    }

    echo "hello EMC mt 1.0"
    echo "set enable EMCTOO"

    echo "set estop off"
    echo "set machine on"
    echo "set mode mdi"


    #
    # first, a bunch of testing with no tool in the spindle
    #
    # On nonrandom the spindle has T0, which is the nonrandom way to
    # specify the special tool that means "no tool".
    #
    # On random the spindle has T-1, which is the random way to specify the
    # special tool that means "no tool".
    #


    # Both random and nonrandom start this test with no tool in the
    # spindle, but they encode this in different ways.
    introspect 0


    #
    # g43
    #

    # Apply the TLO of the current tool.  On both, the spindle has no tool,
    # which has 0 TLO.
    echo "set mdi g43"
    introspect 1

    # Apply the TLO of tool T1.  On both, T1 is a valid tool, so we use its
    # TLO.
    echo "set mdi g43 h1"
    introspect 2

    # Apply the TLO of H0, this one is weird.
    #
    # On nonrandom, this requests the TLO of the tool in the spindle, which
    # is currently T0, which is a special tool with TLO 0.
    #
    # On random, this requests the TLO of T0.  If T0 is not in the tool
    # table, this is an error and the active TLO remains unchanged (so it
    # stays as the TLO of T1 that we just set above).  If T0 is defined, we
    # apply its TLO here.
    echo "set mdi g43 h0"
    introspect 3


    #
    # g10 l1
    #

    # Try to change TLO of the normal tool T7.  This should work on both,
    # but not change the active TLO.
    echo "set mdi g10 l1 p7 z0.1"
    introspect 4

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # both.
    echo "set mdi g43 h7"
    introspect 5

    # Try to change TLO of the strange tool T0.
    #
    # This should fail in every config because you cannot change T0.
    echo "set mdi g10 l1 p0 z0.2"
    introspect 6

    # Apply the TLO of the special H0.
    #
    # On nonrandom, this requests the TLO of the tool in the spindle, which
    # is currently T0, which is a special tool with TLO 0.
    #
    # On random, this requests the TLO of T0.  If T0 is not in the tool
    # table, this is an error and the active TLO remains unchanged (so it
    # stays as the TLO of T7 that we just set above).  If T0 is defined, we
    # apply its TLO here.
    echo "set mdi g43 h0"
    introspect 7


    #
    # g10 l10
    #

    # Try to change TLO of the normal tool T7.  This should work on both.
    echo "set mdi g10 l10 p7 z0.3"
    introspect 8

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # both.
    echo "set mdi g43 h7"
    introspect 9

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on every config because you cannot change T0.
    echo "set mdi g10 l10 p0 z0.4"
    introspect 10

    # Apply the TLO of the special H0.
    #
    # On nonrandom, this requests the TLO of the tool in the spindle, which
    # is currently T0, which is a special tool with TLO 0.
    #
    # On random, this requests the TLO of T0.  If T0 is not in the tool
    # table, this is an error and the active TLO remains unchanged (so it
    # stays as the TLO of T7 that we just set above).  If T0 is defined, we
    # apply its TLO here.
    echo "set mdi g43 h0"
    introspect 11


    #
    # g10 l11
    #

    # First offset the g59.3 coordinate system and make sure that worked,
    # but then switch back to g54.  This sets different g59.3 offsets in
    # different configs, because they have different active Z TLOs at this
    # point.
    #
    # Before this G10 L20, we are at the following Z locations:
    #             nonrandom: 0.0 (T0 TLO active)
    #     random without T0: 0.3 (T7 TLO active)
    #        random with T0: 0.0 (T0 TLO active)
    #
    # Then we set the G59.3 coordinate system offset such that the current
    # Z location is 5, which gives us these G59.3 Z offsets:
    #             nonrandom: -5.0 + 0.0 TLO = -5.0
    #     random without T0: -5.0 + 0.3 TLO = -4.7
    #        random with T0: -5.0 + 0.0 TLO = -5.0
    #
    # All the configs will be at Z=5.0 after switching to G59.3.
    echo "set mdi g10 l20 p9 z5"
    introspect 11.5

    echo "set mdi g59.3"
    introspect 12

    echo "set mdi g54"
    introspect 13

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs.
    #
    # We should end up with the following Z coordinates:
    #             Nonrandom: -5.0 + 0.5 = -4.5
    #     Random without T0: -4.7 + 0.5 = -4.2
    #        Random with T0: -5.0 + 0.5 = -4.5
    #
    # But these offsets are not active until the G43 below.
    echo "set mdi g10 l11 p7 z0.5"
    introspect 14

    # Apply the TLO of T7, which we just tried to set.  This should
    # work on all configs.
    echo "set mdi g43 h7"
    introspect 15

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on every config because you cannot change T0.
    echo "set mdi g10 l11 p0 z0.6"
    introspect 16

    # Apply the TLO of the special H0.
    #
    # On nonrandom, this requests the TLO of the tool in the spindle, which
    # is currently T0, which is a special tool with TLO 0.
    #
    # On random, this requests the TLO of T0.  If T0 is not in the tool
    # table, this is an error and the active TLO remains unchanged (so it
    # stays as the TLO of T7 that we just set above).  If T0 is defined, we
    # apply its TLO here.
    echo "set mdi g43 h0"
    introspect 17


    #
    # That's all the testing we can do of the condition that there's
    # nothing in the spindle.  G41/G42 would be nice to test, but the
    # numbered parameters we use for introspection aren't accessible to
    # gcode while cutter comp is enabled.  :-(
    #
    # So we move on!  Put a valid tool in the spindle and test everything
    # again!
    #


    # Switch to the valid tool T50 but don't apply its TLO yet.
    # This should work on all configs.
    echo "set mdi t50 m6"
    introspect 100


    #
    # g43
    #

    # Apply the TLO of the current tool.  On all configs, the spindle has
    # T50.  This should work on all configs.
    echo "set mdi g43"
    introspect 101

    # change the TLO of the normal tool T7
    # this should work on both, but not change the active TLO
    echo "set mdi g10 l10 p7 z1.1"
    introspect 102

    # apply the TLO of the normal tool T7 (TLO of 0.5 set above)
    # this should work on both
    echo "set mdi g43 h7"
    introspect 103

    # try to change the TLO of T0
    #
    # On nonrandom this should fail because you can't change T0.
    #
    # On random without T0 this should fail because there is no T0.
    #
    # On random with T0 this should fail because you can't change T0.
    echo "set mdi g10 l10 p0 z1.2"
    introspect 104

    # Apply the TLO of the special H0.
    #
    # On nonrandom, this requests the TLO of the tool in the spindle, which
    # is currently T50.
    #
    # On random, this requests the TLO of T0.  If T0 is not in the tool
    # table, this is an error and the active TLO remains unchanged (so it
    # stays as the TLO of T7 that we just set above).  If T0 is defined, we
    # apply its TLO here.
    echo "set mdi g43 h0"
    introspect 105


    #
    # g10 l1
    #

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs, but not change the active TLO.
    echo "set mdi g10 l1 p7 z1.3"
    introspect 106

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # all configs.
    echo "set mdi g43 h7"
    introspect 107

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0.
    #
    # It should fail on random without T0 because it has no T0.
    #
    # It should fail on random with T0 because you cannot change T0.
    echo "set mdi g10 l1 p0 z1.4"
    introspect 108

    # Apply the TLO of the special H0.
    #
    # It should work on nonrandom and give us the TLO of the tool in the
    # spindle, T50 (in this config, G43 H0 means "use current tool").
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with T0 and give us the TLO of T0 (0.0).
    echo "set mdi g43 h0"
    introspect 109


    #
    # g10 l10
    #

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs.
    echo "set mdi g10 l10 p7 z1.5"
    introspect 110

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # all configs.
    echo "set mdi g43 h7"
    introspect 111

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0
    #
    # It should fail on random without T0 because it has no T0.
    #
    # It should fail on random with T0 because you cannot change T0
    echo "set mdi g10 l10 p0 z1.6"
    introspect 112

    # Apply the TLO of the special H0.
    #
    # It should work on nonrandom and give us the TLO of the tool in the
    # spindle, T50 (in this config, G43 H0 means "use current tool").
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with T0 and give us the TLO of T0 (0.0).
    echo "set mdi g43 h0"
    introspect 113

    # Try to change TLO of the currently loaded tool, T50.
    # This should work on all configs but not change the active TLO
    echo "set mdi g10 l10 p50 z1.7"
    introspect 114

    # Apply the TLO of the special H0.
    #
    # It should work on nonrandom and give us the TLO of the tool in the
    # spindle, T50 (in this config, G43 H0 means "use current tool").
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with T0 and give us the TLO of T0 (0.0).
    echo "set mdi g43 h0"
    introspect 115

    # Apply the TLO of T7.
    # This should work on all configs.
    echo "set mdi g43 h7"
    introspect 116

    # Apply the TLO of the current tool
    # This should work on all configs.
    echo "set mdi g43"
    introspect 117


    #
    # g10 l11
    #

    # First offset the G59.3 coordinate system and make sure that worked,
    # but then switch back to G54.  All configs are using the TLO of T50
    # at this point, so they all set the same G59.3 coordinate system
    # offset.
    #
    # Before this G10 L20, all configs are at Z=1.7 (T50 TLO active)
    #
    # Then we set the G59.3 coordinate system offset such that the current
    # Z location is 6, which gives us a G59.3 Z offset of:
    #     -6.0 + 1.7 TLO = -4.3
    #
    # All the configs will be at Z = 6.0 after switching to G59.3.
    echo "set mdi g10 l20 p9 z6"
    introspect 117.5

    echo "set mdi g59.3"
    introspect 118

    echo "set mdi g54"
    introspect 119

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs.
    echo "set mdi g10 l11 p7 z1.8"
    introspect 120

    # Apply the TLO of T7, which we just tried to set.  This should
    # work on all configs.
    echo "set mdi g43 h7"
    introspect 121

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0.
    #
    # It should fail on random without T0 because it has no T0.
    #
    # This should fail on random with T0 because you cannot change T0.
    echo "set mdi g10 l11 p0 z1.3"
    introspect 122

    # Apply the TLO of T0, which we just tried to set.
    #
    # It should work on nonrandom and give us a TLO of the tool in the
    # spindle, T50.
    #
    # It should fail on random and leave the TLO at the value from T7
    # above.
    #
    # It should work on random with T0 and give us the TLO of T0: 0.0
    echo "set mdi g43 h0"
    introspect 123


    #
    # Now load T0 into the spindle and retest everything again.
    #


    # Switch to the special tool T0 but don't apply its TLO yet.
    #
    # On nonrandom, this should unload the spindle.
    #
    # On random without T0 this should fail (there is no T0) and leave
    # T50 in the spindle.
    #
    # On random with T0 this should work and put T0 in the spindle.

    echo "set mdi t0 m6"
    introspect 200


    #
    # g43
    #

    # Apply the TLO of the current tool.
    #
    # On nonrandom the spindle has the special tool T0, which has a 0 TLO.
    #
    # On random without T0 the spindle has T50, which has a TLO of -1.15,
    # set above.
    #
    # On random with T0 the spindle has tool T0, which has a 0 TLO.
    echo "set mdi g43"
    introspect 201

    # change the TLO of the normal tool T7
    # this should work on all configs, but not change the active TLO
    echo "set mdi g10 l10 p7 z2.0"
    introspect 202

    # apply the TLO of the normal tool T7 (TLO set above)
    # this should work on all configs
    echo "set mdi g43 h7"
    introspect 203

    # try to change the TLO of T0
    #
    # On nonrandom this should fail because you can't change T0.
    #
    # On random without T0 this should fail because there is no T0.
    #
    # On random with T0 this should fail because you can't change T0.
    echo "set mdi g10 l10 p0 z2.1"
    introspect 204

    # try to apply the TLO of H0
    #
    # On nonrandom, this should apply the TLO of the current tool (special
    # handling of G43 H0).  The current tool is T0, which has 0 TLO.
    #
    # On random without T0 this should be an error (G43 H0 is nothing
    # special, but there is no T0) and the active TLO should remain
    # unchanged.
    #
    # On random with T0 this should apply the TLO of T0 (0.0).
    echo "set mdi g43 h0"
    introspect 205


    #
    # g10 l1
    #

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs, but not change the active TLO.
    echo "set mdi g10 l1 p7 z2.2"
    introspect 206

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # all configs.
    echo "set mdi g43 h7"
    introspect 207

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0.
    #
    # It should fail on random without T0 because it has no T0.
    #
    # This should fail on random with T0 because you cannot change T0.
    echo "set mdi g10 l1 p0 z2.3"
    introspect 208

    # Apply the TLO of H0.
    #
    # It should work on nonrandom and give us the TLO of the tool in the
    # spindle, T50.
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with TLO and give us the TLO of T0.
    echo "set mdi g43 h0"
    introspect 209


    #
    # g10 l10
    #

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs.
    echo "set mdi g10 l10 p7 z2.4"
    introspect 210

    # Apply the TLO of T7, which we just tried to set.  This should work on
    # all configs.
    echo "set mdi g43 h7"
    introspect 211

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0
    #
    # It should fail on random without T0 because it has no T0.
    #
    # It should fail on random with T0 because you cannot change T0
    echo "set mdi g10 l10 p0 z2.5"
    introspect 212

    # Apply the TLO of the special H0.
    #
    # It should work on nonrandom and give us the TLO of the tool in the
    # spindle, T0 (in this config, G43 H0 means "use current tool").
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with T0 and give us the TLO of T0 (0.0).
    echo "set mdi g43 h0"
    introspect 213


    #
    # g10 l11
    #

    # First offset the G59.3 coordinate system and make sure that worked,
    # but then switch back to G54.  This sets different G59.3 offsets in
    # different configs, because they have different active Z TLOs at
    # this point.
    #
    # Before this G10 L20, we are at the following Z locations:
    #             nonrandom: 0.0 (T0 TLO active)
    #     random without T0: 2.4 (T7 TLO active)
    #        random with T0: 0.0 (T0 TLO active)
    #
    # Then we set the G59.3 coordinate system offset such that the current
    # Z location is 7, which gives us these G59.3 Z offsets:
    #             nonrandom: -7.0 + 0.0 TLO = -7.0
    #     random without T0: -7.0 + 2.4 TLO = -4.6
    #        random with T0: -7.0 + 0.0 TLO = -7.0
    #
    # All the configs will be at Z=7.0 after switching to G59.3.
    echo "set mdi g10 l20 p9 z7"
    introspect 213.5

    echo "set mdi g59.3"
    introspect 214

    echo "set mdi g54"
    introspect 215

    # Try to change TLO of the normal tool T7.  This should work on all
    # configs.
    #
    # We should end up with the following Z coordinates:
    #             Nonrandom: -7.0 + 2.6 = -4.4
    #     Random without T0: -4.6 + 2.6 = -2.0
    #        Random with T0: -7.0 + 2.6 = -4.4
    #
    # But these offsets are not active until the G43 below.
    echo "set mdi g10 l11 p7 z2.6"
    introspect 216

    # Apply the TLO of T7, which we just tried to set.  This should
    # work on all configs.
    echo "set mdi g43 h7"
    introspect 217

    # Try to change TLO of the strange tool T0.
    #
    # This should fail on nonrandom because you cannot change T0.
    #
    # It should fail on random without T0 because it has no T0.
    #
    # It should work on random with T0 because you cannot change T0.
    echo "set mdi g10 l11 p0 z2.7"
    introspect 218

    # Apply the TLO of H0.
    #
    # It should work on nonrandom and give us a TLO of the tool in the
    # spindle, T0
    #
    # It should fail on random without T0 and leave the TLO at the value
    # from T7 above.
    #
    # It should work on random with T0 and give us a TLO of the tool in the
    # spindle, T0
    echo "set mdi g43 h0"
    introspect 219


    # give linuxcnc a second to finish
    echo 'set wait done'

    echo shutdown
) | nc localhost 5007


# wait for linuxcnc to finish
wait

exit 0

