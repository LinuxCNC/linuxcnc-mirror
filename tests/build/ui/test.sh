#!/bin/sh
set -x
g++ -I"${HEADERS}" \
    nml-position-logger.cc \
    -L "${LIBDIR}" -lnml -llinuxcnc \
    -o /dev/null
