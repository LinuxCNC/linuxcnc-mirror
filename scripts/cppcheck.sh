#!/bin/bash

if ! command -v cppcheck; then
  echo "E: Please install the program 'cppcheck' prior to executing this script."
  exit 1
fi

nproc=2
if command -v nproc; then
  nproc=$(nproc)
fi

# See if cppcheck accepts --check-level
EXHAUSTIVE=$(cppcheck --check-level=exhaustive --version > /dev/null 2>&1 && echo "--check-level=exhaustive")

CPPCHKOPT="-j $nproc --force $EXHAUSTIVE"
CPPCHKOPT="$CPPCHKOPT --enable=warning,performance,portability"
CPPCHKOPT="$CPPCHKOPT -I$(realpath "$(dirname "$0")/../include")"

# Even cppcheck 2.3 (debian 11) supports c++17 (undocumented)
CCSTD="--std=c11 --language=c"
CXSTD="--std=c++17 --language=c++"

CPPCHKCC="$CPPCHKOPT $CCSTD"
CPPCHKCX="$CPPCHKOPT $CXSTD"

# Do this from the source directory
cd "$(dirname "$0")/../src" || (echo "E: Could not return to source directory" ; exit 1 )

docheck() {
    files=$(find "$1" -maxdepth 1 \( -name "*.c" -o -name "*.h" \) )
    [ -n "$files" ] && cppcheck -I"$1" "$CPPCHKCC" "$files"

    files=$(find "$1" -maxdepth 1 \( -name "*.cc" -o -name "*.hh" \) )
    [ -n "$files" ] && cppcheck -I"$1" "$CPPCHKCX" "$files"
}

# *** HAL files ***
echo "I (1/4): checking HAL folders with both C and C++ code"
find hal/ -type d -not -name "*__pycache__" | while read -r d
do
    # Don't care about examples
    case "$d" in hal/user_comps/mb2hal/examples*) continue;; esac

    echo "I (1/4): checking $d"
    docheck "$d"
done

# *** EMC files ***
echo "I (2/4): checking EMC folders with both C and C++ code"
find emc/ -type d -not -name "*__pycache__" | while read -r d 
do
    # Will give Tcl problems
    case "$d" in emc/usr_intf/axis/extensions*) continue;; esac

    echo "I (2/4): checking $d"
    docheck "$d"
done

# *** NML files ***
echo "I (3/4): checking LIBNML folders with both C and C++ code"
find libnml/ -type d -not -name "*__pycache__" | while read -r d
do
    echo "I (3/4): checking $d"
    docheck "$d"
done

# *** RTAPI files ***
echo "I (4/4): checking RTAPI folders with both C and C++ code"
find rtapi/ -type d -not -name "*__pycache__" | while read -r d
do
    echo "I (4/4): checking $d"
    docheck "$d"
done

exit 0
