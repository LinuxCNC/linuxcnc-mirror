#!/bin/bash
rs274 -g g17-g98-g81.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
