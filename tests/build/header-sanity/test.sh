#!/bin/bash
set -xe 
CPPFLAGS=$(pkg-config --silence-errors --cflags libtirpc)
for i in "$HEADERS"/*.h; do
    case $i in
    */rtapi_app.h) continue ;;
    esac
    gcc ${CPPFLAGS} -DULAPI -I"$HEADERS" -E -x c "$i" > /dev/null
done
for i in "$HEADERS"/*.h "$HEADERS"/*.hh; do
    case $i in
    */rtapi_app.h) continue ;;
    */interp_internal.hh) continue ;;
    esac
    if g++ ${CPPFLAGS} -std=c++11 -S -o /dev/null -xcxx /dev/null > /dev/null 2>&1; then
        ELEVEN=-std=c++11
    else
        ELEVEN=-std=c++0x
    fi
    g++ ${CPPFLAGS} "$ELEVEN" -DULAPI -I"$HEADERS" -E -x c++ "$i" > /dev/null
done
