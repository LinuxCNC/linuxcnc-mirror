#!/bin/bash
#
# Copy of 'nc_files/lathe_g70_71_demo.ngc'. Uses both G70 and G71.

rs274 -g g71-with-g70.ngc | awk '{$1=""; print}' > result &
pid=$!

# Give it 5 seconds to complete
count=5
while [ 0 -lt $count ] && kill -0 $pid > /dev/null 2>&1 ; do
    sleep 1
    count=$((count - 1))
done

if kill -0 $pid > /dev/null 2>&1; then
    kill -9 $pid
    echo "E: g71_and_g70.ngc program seem to be stuck, killing"
    exit 1
fi

exit 0
