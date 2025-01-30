#!/bin/bash
#
# Test working G71, derived from problem reported in
# https://github.com/LinuxCNC/linuxcnc/issues/707.

rs274 -g g72-iterations-present.ngc | awk '{$1=""; print}' > result &
pid=$!

# Give it 10 seconds to complete
count=10
while [ 0 -lt $count ] && kill -0 $pid > /dev/null 2>&1 ; do
    sleep 1
    count=$((count - 1))
done

if kill -0 $pid > /dev/null 2>&1; then
    kill -9 $pid
    echo "E: g71-iterations-present.ngc program seem to be stuck, killing"
    exit 1
fi

exit 0
