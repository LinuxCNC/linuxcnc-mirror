#!/bin/sh
#
# Test setup based on configs/sim/axis/classicladder/
#

# Disable eatmydata, as it do not seem to work with classicladder
if [ "libeatmydata.so" = "$LD_PRELOAD" ] ; then
    unset LD_PRELOAD
fi
exec linuxcnc "$(dirname "$0")/classicladder-estop.ini"
