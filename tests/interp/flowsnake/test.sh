#!/bin/bash
rs274 -g flowsnake.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
