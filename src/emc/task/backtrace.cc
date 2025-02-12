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
// based on http://stackoverflow.com/questions/4636456/stack-trace-for-c-using-gcc/4732119#4732119
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include "config.h"

extern volatile int done;
extern int emcOperatorError(const char *fmt, ...);
extern int emcOperatorText(const char *fmt, ...);

void backtrace(int signo)
{
    char pid_buf[30];
    char name_buf[512];
    char filename[512];

    signal(signo, SIG_IGN); // prevent multiple invocations on same signal
    snprintf(pid_buf, sizeof(pid_buf), "%d", getpid());
    snprintf(filename, sizeof(filename), EMC2_TMP_DIR "/backtrace.%s", pid_buf);

    name_buf[readlink("/proc/self/exe", name_buf, 511)]=0;
    int child_pid = fork();
    if (!child_pid) {
	fclose(stderr);
	stderr = freopen(filename, "a", stderr);
        dup2(2,1); // child: redirect output to stderr
        fprintf(stdout,"stack trace for %s pid=%s signal=%d\n",name_buf,pid_buf, signo);
        execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread", "-ex", "bt", name_buf, pid_buf, NULL);
	emcOperatorError("backtrace for %s (pid %s signal %d): gdb failed to start", name_buf, pid_buf, signo);
        abort(); /* If gdb failed to start */
    } else {
	int status;
        waitpid(child_pid, &status,0);
	if (signo == SIGUSR1) {  // continue running after backtrace
	    signal(SIGUSR1, backtrace);
	    emcOperatorText("backtrace for %s stored in %s, continuing", name_buf, filename);
	    fprintf(stderr, "%s: backtrace stored in %s, continuing\n", name_buf, filename);
	} else {
	    // this takes emcmodule.cc:EMC_COMMAND_TIMEOUT seconds to display:
	    if (status == 0) // backtrace succeeded
		emcOperatorError("%s (pid %d) died on signal %d, backtrace stored in %s",
				 name_buf, getpid(), signo, filename);
	    fprintf(stderr, "%s exiting\n", name_buf);
	    done = 1;  // signal task to exit main loop
	}
    }
}
