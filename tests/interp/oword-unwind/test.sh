#!/bin/bash
rs274 -n 0 -i test.ini -g test.ngc | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
