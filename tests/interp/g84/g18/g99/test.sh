#!/bin/bash
rs274 -g g18-g99-g84.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
