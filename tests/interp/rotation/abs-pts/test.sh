#!/bin/bash
rs274 -g test.ngc -t test.tbl | awk '/MESSAGE/ {$1=""; print}' | sed 's/-0\.0000/0.0000/g'
exit "${PIPESTATUS[0]}"
