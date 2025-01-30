#!/bin/bash
rs274 -t test.tbl -g g76only.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
