#!/bin/bash
rs274 -g g33.1.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
