#!/bin/bash

thisfile=$(readlink -f "$0")
thisdir=$(dirname "$thisfile")
cd "$thisdir" || { echo "E: Could not change directory to '$thisdir'"; exit 1; }

LAYOUT=${1:-""}

[ -x "$(which "$COLORTERM")" ] && TERM="$COLORTERM"
[ -z "$TERM" ] && TERM=xterm
# when invoked from Application menu and RIP, may have TERM=dumb
[ "$TERM" = "dumb" ] && TERM=xterm

XHC_HB04=$(which xhc-hb04)
HALLIB_DIR=$(linuxcnc_var HALLIB_DIR)

case $LAYOUT in
  layout1) CFG=${HALLIB_DIR}/xhc-hb04-layout1.cfg;;
        *) CFG=${HALLIB_DIR}/xhc-hb04-layout2.cfg;;
esac
dashI="-I $CFG"

if [ -n "$debug" ] ; then
  echo "COLORTERM=$COLORTERM"
  echo "TERM=$TERM"
  echo "XHC_HB04=$XHC_HB04"
  echo "LAYOUT=$LAYOUT"
fi
$TERM -e "$XHC_HB04 -x $dashI"

