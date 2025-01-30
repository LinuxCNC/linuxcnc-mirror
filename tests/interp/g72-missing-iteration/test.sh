#!/bin/bash
#
# Demonstrate G71 problem reported in
# https://github.com/LinuxCNC/linuxcnc/issues/707.  When Z limit is
# 39.999, everything work, when it is 40, the facing is done without
# iterations.

rs274 -g g72-iterations-missing.ngc | awk '{$1=""; print}' > result &
pid=$!

# Give it 10 seconds to complete
count=10
while [ 0 -lt $count ] && kill -0 $pid > /dev/null 2>&1 ; do
    sleep 1
    count=$((count - 1))
done

if kill -0 $pid > /dev/null 2>&1; then
    kill -9 $pid
    echo "E: g71-iterations-missing.ngc program seem to be stuck, killing"
    exit 1
fi

exit 0
