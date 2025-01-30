#!/bin/bash
rs274 -p ./canterp.so -g canon | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
