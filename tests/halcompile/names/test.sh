#!/bin/bash
set -x

# this one must succeed
rm -f names_match.c
if ! halcompile names_match.comp; then
    echo 'halcompile failed to process names_match.comp'
    exit 1
fi
if [ ! -f names_match.c ]; then
    echo 'halcompile failed to produce names_match.c'
    exit 1
fi

# this one must fail
rm -f names_dont_match.c
if halcompile names_dont_match.comp; then
    echo 'halcompile erroneously accepted names_dont_match.comp'
    exit 1
fi
if [ -f names_dont_match.c ]; then
    echo 'halcompile erroneously produced names_dont_match.c'
    exit 1
fi

