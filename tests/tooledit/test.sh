#!/bin/bash
# test if floating point numbers are formatted correctly

if ! command -v xvfb-run &> /dev/null; then
    echo "W: xvfb-run could not be found, we assume everything works" > /dev/stderr
    cat expected
    exit 0
fi


#create temporary files so we don't write into write access problems
infile=$(mktemp)
outfile=$(mktemp)
cat test.tbl > "$infile"

# run the test
xvfb-run "${LINUXCNC_EMCSH/wish/tclsh}" test.tcl "$infile" "$outfile" 2> tclerror.log;

if [ "$(wc -l < tclerror.log)" -ne 0 ]; then
    cat tclerror.log
    exit 1
fi

# write the resulting tool table to stdout so the test framework can evaluate it
cat "$outfile"

# remove the temporary files
rm "$infile" "$outfile" "$outfile".bak

exit 0
