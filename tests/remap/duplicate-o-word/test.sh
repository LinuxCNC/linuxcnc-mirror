#!/bin/bash
export INI_FILE_NAME=test.ini
rs274 -i "$INI_FILE_NAME" -g test.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
