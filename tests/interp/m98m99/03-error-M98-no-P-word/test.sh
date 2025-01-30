#!/bin/bash
if ! rs274 -g test.ngc 2>&1; then
    # expected to fail
    exit 0
fi

exit 1
