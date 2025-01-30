#!/bin/bash -e

LEGAL_TESTS="
	end-main-with-m2
	end-main-with-m02
	end-main-with-m30
	end-main-with-percent
"

ILLEGAL_TESTS="
	end-main-with-eof
	no-m30-before-osub
	sub-after-percent
"

#
# Run legal tests
#
for t in $LEGAL_TESTS; do
    echo "Legal test ${t}:"
    rs274 -g "test-legal-${t}.ngc" | awk '{$1=""; print}'
    res=${PIPESTATUS[0]}
    if test "$res" = 0; then
	echo "Success: Test '${t}' exited ${res}" >&2
    else
	echo "Error: Test '${t}' exited ${res}" >&2
	exit "$res"
    fi
done

# Run illegal tests
#
for t in $ILLEGAL_TESTS; do
    echo "Illegal test ${t}:"
    res=-1
    rs274 -g "test-illegal-${t}.ngc" | awk '{$1=""; print}'
    res=${PIPESTATUS[0]}
    if test "$res" = 0; then
	echo "Error: Illegal test '${t}' exited ${res}" >&2
	exit "$res"
    else
	echo "Success: Illegal test '${t}' exited ${res} (non-zero=success)" >&2
    fi
done
