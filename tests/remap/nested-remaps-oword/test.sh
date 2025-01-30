#!/bin/bash
rs274 -i test.ini -g test.ngc 2>&1 | sed 's/^ *[0-9]* //'
exit "${PIPESTATUS[0]}"
