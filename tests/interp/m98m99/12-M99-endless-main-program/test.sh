#!/bin/bash -e

rm -f out.motion-logger

# Run test file in task:  should loop through program three times
linuxcnc -r motion-logger.ini

# Run test file in stand-alone interpreter:  should run program once
rs274 -g test.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
