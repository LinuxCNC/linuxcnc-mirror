#!/bin/bash
export PYTHONUNBUFFERED=1
if ! rs274 -i test.ini -g test.ngc 2>&1; then
    # expected to fail
    exit 0
fi

exit 1
