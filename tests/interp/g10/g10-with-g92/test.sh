#!/bin/bash
rs274 -g test.ngc -t test.tbl | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
