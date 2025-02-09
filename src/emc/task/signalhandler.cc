/*    This is a component of LinuxCNC
 *    Copyright 2011 Michael Haberler <git@mah.priv.at>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
// generate a backtrace from a signal handler,
// or alternatively start gdb in a new window
//
// start gdb with command script and have it connect to a gdbserver instance
// then start gdbserver, attach it to our pid and let gdb connect to it

#include <stdio.h>
#include <limits.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

static const char *progname;
static const char *dir_prefix = "/tmp/";
static const char *gdbserver = "gdbserver";
static int port = 2345;

static void call_gdb(int sig, int start_gdb_in_window)
{
    FILE *f;
    char tmp_gdbrc[PATH_MAX];

    snprintf(tmp_gdbrc, sizeof(tmp_gdbrc), EMC2_TMP_DIR "/gdbrc.%d",getpid());

    if ((f = fopen(tmp_gdbrc,"w")) == NULL) {
	perror(tmp_gdbrc);
	abort();
    }
    fprintf(f,"set tcp auto-retry on\nset tcp connect-timeout 3600\n"
	    "file %s\ntarget remote :%d\n", progname, port);

    // this should be configurable
    fprintf(f,start_gdb_in_window ? "backtrace\n" :
	    "backtrace full\ninfo source\nquit\n");
    fclose(f);
    char cmd[PATH_MAX];
    if (start_gdb_in_window) {
	snprintf(cmd, sizeof(cmd), "gnome-terminal --title 'GDB - %s backtrace' -x gdb -x %s",
		progname, tmp_gdbrc);
	fprintf(stderr, "signal_handler: got signal %d, starting debugger window (pid %d)\n",sig, getpid());

    } else {
	snprintf(cmd, sizeof(cmd), "gdb --batch -x %s > %sbacktrace.%d &",
		tmp_gdbrc, dir_prefix, getpid());
	fprintf(stderr, "signal_handler: got signal %d, generating backtrace in %sbacktrace.%d\n",sig,  dir_prefix, getpid());
    }
    int rc = system(cmd);
    if (rc == -1) {
	perror(cmd);
    } else if (rc) {
	fprintf(stderr,"system(%s) returned %d", cmd,  rc);
    }
    snprintf(cmd, sizeof(cmd),"%s --once --attach :%d %d &", gdbserver, port, getpid());
    rc = system(cmd);
    if (rc == -1) {
	perror(cmd);
    } else if (rc) {
	fprintf(stderr,"system(%s) returned %d", cmd,  rc);
    }
    // gdb needs a bit of time to connect and do its thing
    sleep(3);
    unlink(tmp_gdbrc);
    fprintf(stderr, "signal_handler: sig %d -  done\n", sig);
    if (sig == SIGSEGV)
	exit(0);
}


void gdb_backtrace(int sig)
{
    call_gdb(sig, 0);
}


void gdb_in_window(int sig)
{
    call_gdb(sig, 1);
}

void setup_signal_handlers()
{
    struct sigaction backtrace_action, gdb_action;
    char path[PATH_MAX];
    char exe[PATH_MAX];

    // determine pathname of running program for gdb
    snprintf(path, sizeof(path),"/proc/%d/exe", getpid());
    if (readlink(path, exe, sizeof(exe)) < 0) {
	fprintf(stderr, "signal_handler: can\'t readlink(%s): %s\n",path,strerror(errno));
	return;
    }
    progname = strdup(exe);

    sigemptyset( &gdb_action.sa_mask );
    gdb_action.sa_handler = gdb_in_window;
    gdb_action.sa_flags   = 0;

    sigemptyset( &backtrace_action.sa_mask );
    backtrace_action.sa_handler = gdb_backtrace;
    backtrace_action.sa_flags   = 0;

    // trap into gdb in new window on SEGV, USR1
    sigaction( SIGSEGV, &gdb_action, (struct sigaction *) NULL );
    sigaction( SIGUSR1, &gdb_action, (struct sigaction *) NULL );

    // generate a backtrace on USR2 signal
    sigaction( SIGUSR2,  &backtrace_action, (struct sigaction *) NULL );
}


#ifdef TEST

int main(int argc, const char *argv[]) {


    signal_handlers();

    sleep(10);  // during which a SIGUSR2 will generate a backtrace

    void *foo = 0;
    // Fully intentional
    // cppcheck-suppress nullPointer
    memset(foo,0,47); // this segfault would warp us into the gdb window

    return 0;
}
#endif
