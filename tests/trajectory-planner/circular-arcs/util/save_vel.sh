#!/bin/bash
set -o nounset                              # Treat unset variables as an error

if [ -a "$1" ]
then
    awk '/tc state/ {print $5,$8,$11}' "$1" > vel_data.log
fi
