#!/bin/bash
rs274 -t test.tbl -g test.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
