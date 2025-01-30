#!/bin/bash
#
# Demonstrate endless loop when encountering incorrect parameters in
# G71 call.  Note, the expected output is not verified.  No idea what
# it should look like on a successful termination of this endless
# loop.

rs274 -g g71-endless-loop.ngc | awk '{$1=""; print}' > result &
pid=$!

# Give it 10 seconds to complete
count=10
while [ 0 -lt $count ] && kill -0 $pid > /dev/null 2>&1 ; do
    sleep 1
    count=$((count - 1))
done

if kill -0 $pid > /dev/null 2>&1; then
    kill -9 $pid
    echo "E: g71-endless-loop.ngc program seem to be stuck, killing"
    exit 1
fi

exit 0
