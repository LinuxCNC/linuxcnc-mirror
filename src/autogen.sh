#!/bin/sh
rm -rf autom4te.cache
AUTOGEN_TARGET=${AUTOGEN_TARGET-configure:config.h.in}
set -e
case :$AUTOGEN_TARGET: in
*:configure:*)
    automake_libdir=$(automake --print-libdir)
    [ -e config.guess ] || cp "$automake_libdir/config.guess" .
    [ -e config.sub ] || cp "$automake_libdir/config.sub" .
    [ -e install-sh ] || cp "$automake_libdir/install-sh" .
    aclocal --force
    autoconf
    # autoconf only updates the timestamp if the output actually changed.
    # The target's timestamp must be updated or make is confused
    touch configure
    ;;
esac
case :$AUTOGEN_TARGET: in
*:config.h.in:*)
    autoheader
    # autoheader only updates the timestamp if the output actually changed.
    # The target's timestamp must be updated or make is confused
    touch config.h.in
    ;;
esac
