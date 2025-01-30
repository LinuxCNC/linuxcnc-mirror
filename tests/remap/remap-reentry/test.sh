#!/bin/bash -e
rs274 -g -i test.ini test.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
