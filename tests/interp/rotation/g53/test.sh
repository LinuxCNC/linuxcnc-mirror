#!/bin/bash
rs274 -g g53.ngc | sed 's/-0\.0000/0.0000/g'
exit "${PIPESTATUS[0]}"
