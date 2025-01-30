#!/bin/bash
rs274 -p "${LIBDIR}"/linuxcnc/canterp.so -g canon | awk '{$1=""; print}'
exit "${PIPESTATUS[0]}"
