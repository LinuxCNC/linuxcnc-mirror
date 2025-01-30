#!/bin/bash
rs274 -t test.tbl -g cam.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
