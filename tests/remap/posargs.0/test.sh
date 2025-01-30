#!/bin/bash
rs274 -i test.ini -n 0 -g test.ngc| awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
