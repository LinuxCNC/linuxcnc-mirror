#!/bin/bash
rs274 -g inverse.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
