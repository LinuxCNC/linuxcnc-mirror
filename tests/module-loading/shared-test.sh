#!/bin/bash

halrun setup.hal > hal-output 2>&1
RESULT=$?

# grep -c counts number of matches
NUM_PINS=$(grep -E -c "$(cat PIN_NAME_REGEX)" hal-output)

if [ "$RESULT" -ne "$(cat RESULT)" ]; then
    exit 1
fi

if [ "$NUM_PINS" -ne "$(cat NUM_PINS)" ]; then
    exit 1
fi

exit 0

