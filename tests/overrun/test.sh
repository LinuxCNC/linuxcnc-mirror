#!/bin/sh
TMPDIR=$(mktemp -d /tmp/overrun.XXXXXX)
trap 'rm -rf "$TMPDIR"' 0 1 2 3 15

TEST_HAL=$TMPDIR/test.hal
echo loadusr -w echo overrun > "$TMPDIR/test.hal"

! $RUNTESTS "$TMPDIR" 2>&1
