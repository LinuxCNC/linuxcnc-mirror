#!/bin/sh
# shellcheck disable=SC2086
gcc -O -I${HEADERS} test.c -o test -DULAPI -std=gnu99 -pthread || exit 1
./test; exitval=$?
rm -f test
exit $exitval
