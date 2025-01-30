#!/bin/sh
TMPDIR=$(mktemp -d /tmp/overrun.XXXXXX)
trap "rm -rf '$TMPDIR'" 0 1 2 3 15
# not trapping 9 since it cannot be trapped

TEST_HAL="$TMPDIR"/test.hal
echo loadusr -w echo overrun > "$TMPDIR"/test.hal

! $RUNTESTS "$TMPDIR" 2>&1
