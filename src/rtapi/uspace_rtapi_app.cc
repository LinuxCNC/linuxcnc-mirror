/* Copyright (C) 2006-2014 Jeff Epler <jepler@unpythonic.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "config.h"
#include "linuxcnc.h"

#ifdef __linux__
#include <sys/fsuid.h>
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <signal.h>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#ifdef HAVE_SYS_IO_H
#include <sys/io.h>
#endif
#include <sys/resource.h>
#include <sys/mman.h>
#ifdef __linux__
#include <malloc.h>
#include <sys/prctl.h>
#endif
#ifdef __FreeBSD__
#include <pthread_np.h>
#endif

#include "rtapi.h"
#include "hal.h"
#include "hal/hal_priv.h"
#include "rtapi_uspace.hh"

#include <boost/lockfree/queue.hpp>

std::atomic<int> WithRoot::level;
static uid_t euid, ruid;

#include "rtapi/uspace_common.h"

WithRoot::WithRoot() {
    if(!level++) {
#ifdef __linux__
        setfsuid(euid);
#endif
    }
}

WithRoot::~WithRoot() {
    if(!--level) {
#ifdef __linux__
        setfsuid(ruid);
#endif
    }
}

namespace
{
RtapiApp &App();

struct message_t {
    msg_level_t level;
    char msg[1024-sizeof(level)];
};

boost::lockfree::queue<message_t, boost::lockfree::capacity<128>>
rtapi_msg_queue;

static void set_namef(const char *fmt, ...) {
    char *buf = NULL;
    va_list ap;

    va_start(ap, fmt);
    if (vasprintf(&buf, fmt, ap) < 0) {
        return;
    }
    va_end(ap);

    int res = pthread_setname_np(pthread_self(), buf);
    if (res) {
        fprintf(stderr, "pthread_setname_np() failed for %s: %d\n", buf, res);
    }
    free(buf);
}

pthread_t queue_thread;
void *queue_function(void * /*arg*/) {
    set_namef("rtapi_app:mesg");
    // note: can't use anything in this function that requires App() to exist
    // but it's OK to use functions that aren't safe for realtime (that's the
    // point of running this in a thread)
    while(1) {
        pthread_testcancel();
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, nullptr);
        rtapi_msg_queue.consume_all([](const message_t &m) {
            fputs(m.msg, m.level == RTAPI_MSG_ALL ? stdout : stderr);
        });
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr);
        struct timespec ts = {0, 10000000};
        rtapi_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL, NULL);
    }
    return nullptr;
}
}

static int sim_rtapi_run_threads(int fd, int (*callback)(int fd));

using namespace std;

template<class T> T DLSYM(void *handle, const string &name) {
	return (T)(dlsym(handle, name.c_str()));
}

template<class T> T DLSYM(void *handle, const char *name) {
	return (T)(dlsym(handle, name));
}

static std::map<string, void*> modules;

static int instance_count = 0;
static int force_exit = 0;

static int do_newinst_cmd(string type, string name, string arg) {
    void *module = modules["hal_lib"];
    if(!module) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: hal_lib is required, but not loaded\n");
        return -1;
    }

    hal_comp_t *(*find_comp_by_name)(char*) =
        DLSYM<hal_comp_t*(*)(char *)>(module, "halpr_find_comp_by_name");
    if(!find_comp_by_name) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: halpr_find_comp_by_name not found\n");
        return -1;
    }

    hal_comp_t *comp = find_comp_by_name((char*)type.c_str());
    if(!comp) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: component %s not found\n", type.c_str());
        return -1;
    }

    return comp->make((char*)name.c_str(), (char*)arg.c_str());
}

static int do_one_item(char item_type_char, const string &param_name, const string &param_value, void *vitem, int idx=0) {
    char *endp;
    switch(item_type_char) {
        case 'l': {
            long *litem = *(long**) vitem;
            litem[idx] = strtol(param_value.c_str(), &endp, 0);
	    if(*endp) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                        "`%s' invalid for parameter `%s'",
                        param_value.c_str(), param_name.c_str());
                return -1;
            }
            return 0;
        }
        case 'i': {
            int *iitem = *(int**) vitem;
            iitem[idx] = strtol(param_value.c_str(), &endp, 0);
	    if(*endp) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                        "`%s' invalid for parameter `%s'",
                        param_value.c_str(), param_name.c_str());
                return -1;
            }
            return 0;
        }
        case 's': {
            char **sitem = *(char***) vitem;
            sitem[idx] = strdup(param_value.c_str());
            return 0;
        }
        default:
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: Invalid type character `%c'\n",
                    param_name.c_str(), item_type_char);
            return -1;
        }
    return 0;
}

void remove_quotes(string &s) {
    s.erase(remove_copy(s.begin(), s.end(), s.begin(), '"'), s.end());
}

static int do_comp_args(void *module, vector<string> args) {
    for(unsigned i=1; i < args.size(); i++) {
        string &s = args[i];
	remove_quotes(s);
        size_t idx = s.find('=');
        if(idx == string::npos) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Invalid parameter `%s'\n",
                    s.c_str());
            return -1;
        }
        string param_name(s, 0, idx);
        string param_value(s, idx+1);
        void *item=DLSYM<void*>(module, "rtapi_info_address_" + param_name);
        if(!item) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unknown parameter `%s'\n", s.c_str());
            return -1;
        }
        char **item_type=DLSYM<char**>(module, "rtapi_info_type_" + param_name);
        if(!item_type || !*item_type) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unknown parameter `%s' (type information missing)\n",
                    s.c_str());
            return -1;
        }

        int*max_size_ptr=DLSYM<int*>(module, "rtapi_info_size_" + param_name);

        char item_type_char = **item_type;
        if(max_size_ptr) {
            int max_size = *max_size_ptr;
            size_t idx = 0;
            int i = 0;
            while(idx != string::npos) {
                if(i == max_size) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                            "%s: can only take %d arguments\n",
                            s.c_str(), max_size);
                    return -1;
                }
                size_t idx1 = param_value.find(",", idx);
                string substr(param_value, idx, idx1 - idx);
                int result = do_one_item(item_type_char, s, substr, item, i);
                if(result != 0) return result;
                i++;
                idx = idx1 == string::npos ? idx1 : idx1 + 1;
            }
        } else {
            int result = do_one_item(item_type_char, s, param_value, item);
            if(result != 0) return result;
        }
    }
    return 0;
}

static int do_load_cmd(string name, vector<string> args) {
    void *w = modules[name];
    if(w == NULL) {
        char what[LINELEN+1];
        snprintf(what, LINELEN, "%s/%s.so", EMC2_RTLIB_DIR, name.c_str());
        void *module = modules[name] = dlopen(what, RTLD_GLOBAL | RTLD_NOW);
        if(!module) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: dlopen: %s\n", name.c_str(), dlerror());
            modules.erase(name);
            return -1;
        }
	/// XXX handle arguments
        int (*start)(void) = DLSYM<int(*)(void)>(module, "rtapi_app_main");
        if(!start) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: dlsym: %s\n", name.c_str(), dlerror());
            dlclose(module);
            modules.erase(name);
            return -1;
        }
        int result;

        result = do_comp_args(module, args);
        if(result < 0) {
            dlclose(module);
            modules.erase(name);
            return -1;
        }

        if ((result=start()) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: rtapi_app_main: %s (%d)\n",
                name.c_str(), strerror(-result), result);
            dlclose(module);
            modules.erase(name);
	    return result;
        } else {
            instance_count ++;
	    return 0;
        }
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: already exists\n", name.c_str());
        return -1;
    }
}

static int do_unload_cmd(string name) {
    void *w = modules[name];
    if(w == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: not loaded\n", name.c_str());
	return -1;
    } else {
        int (*stop)(void) = DLSYM<int(*)(void)>(w, "rtapi_app_exit");
	if(stop) stop();
	modules.erase(modules.find(name));
        dlclose(w);
        instance_count --;
    }
    return 0;
}

struct ReadError : std::exception {};
struct WriteError : std::exception {};

static int read_number(int fd) {
    int r = 0, neg=1;
    char ch;

    while(1) {
        int res = read(fd, &ch, 1);
        if(res != 1) return -1;
        if(ch == '-') neg = -1;
        else if(ch == ' ') return r * neg;
        else r = 10 * r + ch - '0';
    }
}

static string read_string(int fd) {
    int len = read_number(fd);
    if(len < 0)
        throw ReadError();
    if(!len)
        return string();
    string str(len, 0);
    if(read(fd, str.data(), len) != len)
        throw ReadError();
    return str;
}

static vector<string> read_strings(int fd) {
    vector<string> result;
    int count = read_number(fd);
    if(count < 0)
        return result;
    for(int i=0; i<count; i++) {
        result.push_back(read_string(fd));
    }
    return result;
}

static void write_number(string &buf, int num) {
    char numbuf[10];
    snprintf(numbuf, sizeof(numbuf), "%d ", num);
    buf = buf + numbuf;
}

static void write_string(string &buf, string s) {
    write_number(buf, s.size());
    buf += s;
}

static void write_strings(int fd, vector<string> strings) {
    string buf;
    write_number(buf, strings.size());
    for(unsigned int i=0; i<strings.size(); i++) {
        write_string(buf, strings[i]);
    }
    if(write(fd, buf.data(), buf.size()) != (ssize_t)buf.size()) throw WriteError();
}

static int handle_command(vector<string> args) {
    if(args.size() == 0) { return 0; }
    if(args.size() == 1 && args[0] == "exit") {
        force_exit = 1;
        return 0;
    } else if(args.size() >= 2 && args[0] == "load") {
        string name = args[1];
        args.erase(args.begin());
        return do_load_cmd(name, args);
    } else if(args.size() == 2 && args[0] == "unload") {
        return do_unload_cmd(args[1]);
    } else if(args.size() == 3 && args[0] == "newinst") {
        return do_newinst_cmd(args[1], args[2], "");
    } else if(args.size() == 4 && args[0] == "newinst") {
        return do_newinst_cmd(args[1], args[2], args[3]);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Unrecognized command starting with %s\n",
                args[0].c_str());
        return -1;
    }
}

static int slave(int fd, vector<string> args) {
    try {
        write_strings(fd, args);
    }
    catch (WriteError &e) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_app: failed to write to master: %s\n", strerror(errno));
    }

    int result = read_number(fd);
    return result;
}

static int callback(int fd)
{
    struct sockaddr_un client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    socklen_t len = sizeof(client_addr);
    int fd1 = accept(fd, (sockaddr*)&client_addr, &len);
    if(fd1 < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_app: failed to accept connection from slave: %s\n", strerror(errno));
        return -1;
    } else {
        int result;
        try {
            result = handle_command(read_strings(fd1));
        } catch (ReadError &e) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_app: failed to read from slave: %s\n", strerror(errno));
            close(fd1);
            return -1;
        }
        string buf;
        write_number(buf, result);
        if(write(fd1, buf.data(), buf.size()) != (ssize_t)buf.size()) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_app: failed to write to slave: %s\n", strerror(errno));
        };
        close(fd1);
    }
    return !force_exit && instance_count > 0;
}

static pthread_t main_thread{};

static int master(int fd, vector<string> args) {
    main_thread = pthread_self();
    int result;
    if((result = pthread_create(&queue_thread, nullptr, &queue_function, nullptr)) != 0) {
        errno = result;
        perror("pthread_create (queue function)");
        return -1;
    }
    do_load_cmd("hal_lib", vector<string>());
    instance_count = 0;
    App(); // force rtapi_app to be created
    if(args.size()) {
        result = handle_command(args);
        if(result != 0) goto out;
        if(force_exit || instance_count == 0) goto out;
    }
    sim_rtapi_run_threads(fd, callback);
out:
    pthread_cancel(queue_thread);
    pthread_join(queue_thread, nullptr);
    rtapi_msg_queue.consume_all([](const message_t &m) {
        fputs(m.msg, m.level == RTAPI_MSG_ALL ? stdout : stderr);
    });
    return result;
}

static std::string
_get_fifo_path() {
    std::string s;
    if(getenv("RTAPI_FIFO_PATH"))
       s = getenv("RTAPI_FIFO_PATH");
    else if(getenv("HOME"))
       s = std::string(getenv("HOME")) + "/.rtapi_fifo";
    else {
       rtapi_print_msg(RTAPI_MSG_ERR,
           "rtapi_app: RTAPI_FIFO_PATH and HOME are unset.  rtapi fifo creation is unsafe.");
       return NULL;
    }
    if(s.size() + 1 > sizeof(sockaddr_un::sun_path)) {
       rtapi_print_msg(RTAPI_MSG_ERR,
           "rtapi_app: rtapi fifo path is too long (arch limit %zd): %s",
               sizeof(sockaddr_un::sun_path), s.c_str());
       return NULL;
    }
    return s;
}

static const char *
get_fifo_path() {
    static std::string path = _get_fifo_path();
    return path.c_str();
}

static int
get_fifo_path(char *buf, size_t bufsize) {
    const char *s = get_fifo_path();
    if(!s) return -1;
    snprintf(buf, bufsize, "%s", s);
    return 0;
}

int main(int argc, char **argv) {
    if(getuid() == 0) {
        char *fallback_uid_str = getenv("RTAPI_UID");
        int fallback_uid = fallback_uid_str ? atoi(fallback_uid_str) : 0;
        if(fallback_uid == 0)
        {
            fprintf(stderr,
                "Refusing to run as root without fallback UID specified\n"
                "To run under a debugger with I/O, use e.g.,\n"
                "    sudo env RTAPI_UID=`id -u` RTAPI_FIFO_PATH=$HOME/.rtapi_fifo gdb " EMC2_BIN_DIR "/rtapi_app\n");
            exit(1);
        }
        if (setreuid(fallback_uid, 0) != 0) { perror("setreuid"); abort(); }
        fprintf(stderr,
            "Running with fallback_uid.  getuid()=%d geteuid()=%d\n",
            getuid(), geteuid());
    }
    ruid = getuid();
    euid = geteuid();
    if (setresuid(euid, euid, ruid) != 0) { perror("setresuid"); abort(); }
#ifdef __linux__
    setfsuid(ruid);
#endif
    vector<string> args;
    for(int i=1; i<argc; i++) { args.push_back(string(argv[i])); }

become_master:
    int fd = socket(PF_UNIX, SOCK_STREAM, 0);
    if(fd == -1) { perror("socket"); exit(1); }

    int enable = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    if(get_fifo_path(addr.sun_path, sizeof(addr.sun_path)) < 0)
       exit(1);
    int result = ::bind(fd, (sockaddr*)&addr, sizeof(addr));

    if(result == 0) {
        int result = listen(fd, 10);
        if(result != 0) { perror("listen"); exit(1); }
        setsid(); // create a new session if we can...
        result = master(fd, args);
        unlink(get_fifo_path());
        return result;
    } else if(errno == EADDRINUSE) {
        struct timeval t0, t1;
        gettimeofday(&t0, NULL);
        gettimeofday(&t1, NULL);
        for(int i=0; i < 3 || (t1.tv_sec < 3 + t0.tv_sec) ; i++) {
            result = connect(fd, (sockaddr*)&addr, sizeof(addr));
            if(result == 0) break;
            if(i==0) srand48(t0.tv_sec ^ t0.tv_usec);
            usleep(lrand48() % 100000);
            gettimeofday(&t1, NULL);
        }
        if(result < 0 && errno == ECONNREFUSED) {
            unlink(get_fifo_path());
            fprintf(stderr, "Waited 3 seconds for master.  giving up.\n");
            close(fd);
            goto become_master;
        }
        if(result < 0) { fprintf(stderr, "connect %s: %s", addr.sun_path, strerror(errno)); exit(1); }
        return slave(fd, args);
    } else {
        perror("bind"); exit(1);
    }
}


/* These structs hold data associated with objects like tasks, etc. */
/* Task handles are pointers to these structs.                      */

struct rtapi_module {
  int magic;
};

#define MODULE_MAGIC  30812
#define SHMEM_MAGIC   25453

#define MAX_MODULES  64
#define MODULE_OFFSET 32768

rtapi_task::rtapi_task()
    : magic{}, id{}, owner{}, stacksize{}, prio{},
      period{}, nextstart{},
      ratio{}, arg{}, taskcode{}
{}

namespace
{
struct PosixTask : rtapi_task
{
    PosixTask() : rtapi_task{}, thr{}
    {}

    pthread_t thr;                /* thread's context */
};

struct Posix : RtapiApp
{
    Posix(int policy = SCHED_FIFO) : RtapiApp(policy), do_thread_lock(policy != SCHED_FIFO) {
        pthread_once(&key_once, init_key);
        if(do_thread_lock) {
            pthread_once(&lock_once, init_lock);
        }
    }
    int task_delete(int id);
    int task_start(int task_id, unsigned long period_nsec);
    int task_pause(int task_id);
    int task_resume(int task_id);
    int task_self();
    long long task_pll_get_reference(void);
    int task_pll_set_correction(long value);
    void wait();
    struct rtapi_task *do_task_new() {
        return new PosixTask;
    }
    unsigned char do_inb(unsigned int port);
    void do_outb(unsigned char value, unsigned int port);
    int run_threads(int fd, int (*callback)(int fd));
    static void *wrapper(void *arg);
    bool do_thread_lock;

    static pthread_once_t key_once;
    static pthread_key_t key;
    static void init_key(void) {
        pthread_key_create(&key, NULL);
    }

    static pthread_once_t lock_once;
    static pthread_mutex_t thread_lock;
    static void init_lock(void) {
        pthread_mutex_init(&thread_lock, NULL);
    }

    long long do_get_time(void) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }

    void do_delay(long ns);
};

static void signal_handler(int sig, siginfo_t * /*si*/, void * /*uctx*/)
{
    switch (sig) {
    case SIGXCPU:
        // should not happen - must be handled in RTAPI if enabled
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: BUG: SIGXCPU received - exiting\n");
        exit(0);
        break;

    case SIGTERM:
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: SIGTERM - shutting down\n");
        exit(0);
        break;

    default: // pretty bad
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: caught signal %d - dumping core\n", sig);
        sleep(1); // let syslog drain
        signal(sig, SIG_DFL);
        raise(sig);
        break;
    }
    exit(1);
}

const size_t PRE_ALLOC_SIZE = 1024*1024*32;
const static struct rlimit unlimited = {RLIM_INFINITY, RLIM_INFINITY};
static void configure_memory()
{
    int res = setrlimit(RLIMIT_MEMLOCK, &unlimited);
    if(res < 0) perror("setrlimit");

    res = mlockall(MCL_CURRENT | MCL_FUTURE);
    if(res < 0) perror("mlockall");

#ifdef __linux__
    /* Turn off malloc trimming.*/
    if (!mallopt(M_TRIM_THRESHOLD, -1)) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                  "mallopt(M_TRIM_THRESHOLD, -1) failed\n");
    }
    /* Turn off mmap usage. */
    if (!mallopt(M_MMAP_MAX, 0)) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                  "mallopt(M_MMAP_MAX, -1) failed\n");
    }
#endif
    /*
     * The following code seems pointless, but there is a non-observable effect
     * in the allocation and loop.
     *
     * The malloc() is forced to set brk() because mmap() allocation is
     * disabled in a call to mallopt() above. All touched pages become resident
     * and locked in the loop because of above mlockall() call (see notes in
     * mlockall(2)). The mallopt() trim setting prevents the brk() from being
     * reduced after free(), effectively creating an open space for future
     * allocations that will not generate page faults.
     *
     * The qualifier 'volatile' on the buffer pointer is required because newer
     * clang would remove the malloc(), for()-loop and free() completely.
     * Marking 'buf' volatile ensures that the code will remain in place.
     */
    volatile char *buf = static_cast<volatile char *>(malloc(PRE_ALLOC_SIZE));
    if (buf == NULL) {
        rtapi_print_msg(RTAPI_MSG_WARN, "malloc(PRE_ALLOC_SIZE) failed\n");
        return;
    }
    long pagesize = sysconf(_SC_PAGESIZE);
    /* Touch each page in this piece of memory to get it mapped into RAM */
    for (size_t i = 0; i < PRE_ALLOC_SIZE; i += pagesize) {
            /* Each write to this buffer will generate a pagefault.
             * Once the pagefault is handled a page will be locked in
             * memory and never given back to the system. */
            buf[i] = 0;
    }
    free((void *)buf);
}

static int harden_rt()
{
    if(!rtapi_is_realtime()) return -EINVAL;

    WITH_ROOT;
#if defined(__linux__) && (defined(__x86_64__) || defined(__i386__))
    if (iopl(3) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "iopl() failed: %s\n"
                        "cannot gain I/O privileges - "
                        "forgot 'sudo make setuid' or using secure boot? -"
                        "parallel port access is not allowed\n",
                        strerror(errno));
    }
#endif

    struct sigaction sig_act = {};
#ifdef __linux__
    // enable realtime
    if (setrlimit(RLIMIT_RTPRIO, &unlimited) < 0)
    {
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "setrlimit(RTLIMIT_RTPRIO): %s\n",
		  strerror(errno));
        return -errno;
    }

    // enable core dumps
    if (setrlimit(RLIMIT_CORE, &unlimited) < 0)
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "setrlimit: %s - core dumps may be truncated or non-existent\n",
		  strerror(errno));

    // even when setuid root
    if (prctl(PR_SET_DUMPABLE, 1) < 0)
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "prctl(PR_SET_DUMPABLE) failed: no core dumps will be created - %d - %s\n",
		  errno, strerror(errno));
#endif /* __linux__ */

    configure_memory();

    sigemptyset( &sig_act.sa_mask );
    sig_act.sa_handler = SIG_IGN;
    sig_act.sa_sigaction = NULL;

    // prevent stopping of RT threads by ^Z
    sigaction(SIGTSTP, &sig_act, (struct sigaction *) NULL);

    sig_act.sa_sigaction = signal_handler;
    sig_act.sa_flags   = SA_SIGINFO;

    sigaction(SIGSEGV, &sig_act, (struct sigaction *) NULL);
    sigaction(SIGILL,  &sig_act, (struct sigaction *) NULL);
    sigaction(SIGFPE,  &sig_act, (struct sigaction *) NULL);
    sigaction(SIGTERM, &sig_act, (struct sigaction *) NULL);
    sigaction(SIGINT, &sig_act, (struct sigaction *) NULL);

#ifdef __linux__
    int fd = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN, "failed to open /dev/cpu_dma_latency: %s\n", strerror(errno));
    } else {
        int r;
        r = write(fd, "\0\0\0\0", 4);
        if (r != 4) {
            rtapi_print_msg(RTAPI_MSG_WARN, "failed to write to /dev/cpu_dma_latency: %s\n", strerror(errno));
        }
        // deliberately leak fd until program exit
    }
#endif /* __linux__ */
    return 0;
}


static RtapiApp *makeApp()
{
    if(euid != 0 || harden_rt() < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX non-realtime\n");
        return new Posix(SCHED_OTHER);
    }
    WithRoot r;
    void *dll = nullptr;
    if(detect_xenomai()) {
        dll = dlopen(EMC2_HOME "/lib/libuspace-xenomai.so.0", RTLD_NOW);
        if(!dll) fprintf(stderr, "dlopen: %s\n", dlerror());
    } else if(detect_rtai()) {
        dll = dlopen(EMC2_HOME "/lib/libuspace-rtai.so.0", RTLD_NOW);
        if(!dll) fprintf(stderr, "dlopen: %s\n", dlerror());
    }
    if(dll)
    {
        auto fn = reinterpret_cast<RtapiApp*(*)()>(dlsym(dll, "make"));
        if(!fn) fprintf(stderr, "dlopen: %s\n", dlerror());
        auto result = fn ? fn() : nullptr;
        if(result) {
            return result;
        }
    }
    rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX realtime\n");
    return new Posix(SCHED_FIFO);
}
RtapiApp &App()
{
    static RtapiApp *app = makeApp();
    return *app;
}

}
/* data for all tasks */
struct rtapi_task *task_array[MAX_TASKS];

/* Priority functions.  Uspace uses POSIX task priorities. */

int RtapiApp::prio_highest() const
{
    return sched_get_priority_max(policy);
}

int RtapiApp::prio_lowest() const
{
  return sched_get_priority_min(policy);
}

int RtapiApp::prio_higher_delta() const {
    if(rtapi_prio_highest() > rtapi_prio_lowest()) {
        return 1;
    }
    return -1;
}

int RtapiApp::prio_bound(int prio) const {
    if(rtapi_prio_highest() > rtapi_prio_lowest()) {
        if (prio >= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio < rtapi_prio_lowest())
            return rtapi_prio_lowest();
    } else {
        if (prio <= rtapi_prio_highest())
            return rtapi_prio_highest();
        if (prio > rtapi_prio_lowest())
            return rtapi_prio_lowest();
    }
    return prio;
}

int RtapiApp::prio_next_higher(int prio) const
{
    prio = prio_bound(prio);
    if(prio != rtapi_prio_highest())
        return prio + prio_higher_delta();
    return prio;
}

int RtapiApp::prio_next_lower(int prio) const
{
    prio = prio_bound(prio);
    if(prio != rtapi_prio_lowest())
        return prio - prio_higher_delta();
    return prio;
}

int RtapiApp::allocate_task_id()
{
    for(int n=0; n<MAX_TASKS; n++)
    {
        rtapi_task **taskptr = &(task_array[n]);
        if(__sync_bool_compare_and_swap(taskptr, (rtapi_task*)0, TASK_MAGIC_INIT))
            return n;
    }
    return -ENOSPC;
}

int RtapiApp::task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
  /* check requested priority */
  if ((prio > rtapi_prio_highest()) || (prio < rtapi_prio_lowest()))
  {
    return -EINVAL;
  }

  /* label as a valid task structure */
  int n = allocate_task_id();
  if(n < 0) return n;

  struct rtapi_task *task = do_task_new();
  if(stacksize < (1024*1024)) stacksize = (1024*1024);
  task->id = n;
  task->owner = owner;
  task->uses_fp = uses_fp;
  task->arg = arg;
  task->stacksize = stacksize;
  task->taskcode = taskcode;
  task->prio = prio;
  task->magic = TASK_MAGIC;
  task_array[n] = task;

  /* and return handle to the caller */

  return n;
}

rtapi_task *RtapiApp::get_task(int task_id) {
    if(task_id < 0 || task_id >= MAX_TASKS) return NULL;
    /* validate task handle */
    rtapi_task *task = task_array[task_id];
    if(!task || task == TASK_MAGIC_INIT || task->magic != TASK_MAGIC)
        return NULL;

    return task;
}

void RtapiApp::unexpected_realtime_delay(rtapi_task *task, int /*nperiod*/) {
    static int printed = 0;
    if(!printed)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Unexpected realtime delay on task %d with period %ld\n"
                "This Message will only display once per session.\n"
                "Run the Latency Test and resolve before continuing.\n",
                task->id, task->period);
        printed = 1;
    }
}

int Posix::task_delete(int id)
{
  auto task = ::rtapi_get_task<PosixTask>(id);
  if(!task) return -EINVAL;

  pthread_cancel(task->thr);
  pthread_join(task->thr, 0);
  task->magic = 0;
  task_array[id] = 0;
  delete task;
  return 0;
}

static int probe_rt_cpu_number() {
#ifdef __linux__
    cpu_set_t cpuset_orig;
    int r = sched_getaffinity(getpid(), sizeof(cpuset_orig), &cpuset_orig);
    if(r < 0)
        // if getaffinity fails, (it shouldn't be able to), just use CPU#0
        return 0;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    long top_probe = sysconf(_SC_NPROCESSORS_CONF);
    // in old glibc versions, it was an error to pass to sched_setaffinity bits
    // that are higher than an imagined/probed kernel-side CPU mask size.
    // this caused the message
    //     sched_setaffinity: Invalid argument
    // to be printed at startup, and the probed CPU would not take into
    // account CPUs masked from this process by default (whether by
    // isolcpus or taskset).  By only setting bits up to the "number of
    // processes configured", the call is successful on glibc versions such as
    // 2.19 and older.
    for(long i=0; i<top_probe && i<CPU_SETSIZE; i++) CPU_SET(i, &cpuset);

    r = sched_setaffinity(getpid(), sizeof(cpuset), &cpuset);
    if(r < 0)
        // if setaffinity fails, (it shouldn't be able to), go on with
        // whatever the default CPUs were.
        perror("sched_setaffinity");

    r = sched_getaffinity(getpid(), sizeof(cpuset), &cpuset);
    if(r < 0) {
        // if getaffinity fails, (it shouldn't be able to), copy the
        // original affinity list in and use it
        perror("sched_getaffinity");
        CPU_AND(&cpuset, &cpuset_orig, &cpuset);
    }

    int top = -1;
    for(int i=0; i<CPU_SETSIZE; i++) {
        if(CPU_ISSET(i, &cpuset)) top = i;
    }
    return top;
#else
    return (-1);
#endif
}

static int find_rt_cpu_number(int prio) {
    static const int probed_cpu_number = probe_rt_cpu_number();

    char cpu_env[64];
    const char *minus = "";
    if (prio < 0) {
        prio = -prio;
	minus = "m";
    }
    snprintf(cpu_env, sizeof(cpu_env), "RTAPI_CPU_NUMBER_PRIO%s%d", minus, prio);
    if(getenv(cpu_env))
        return atoi(getenv(cpu_env));
    if(getenv("RTAPI_CPU_NUMBER"))
        return atoi(getenv("RTAPI_CPU_NUMBER"));

    return probed_cpu_number;
}

int Posix::task_start(int task_id, unsigned long int period_nsec)
{
  auto task = ::rtapi_get_task<PosixTask>(task_id);
  if(!task) return -EINVAL;

  if(period_nsec < (unsigned long)period) period_nsec = (unsigned long)period;
  task->period = period_nsec;
  task->ratio = period_nsec / period;

  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = task->prio;

  // limit PLL correction values to +/-1% of cycle time
  task->pll_correction_limit = period_nsec / 100;
  task->pll_correction = 0;

  int nprocs = sysconf( _SC_NPROCESSORS_ONLN );

  pthread_attr_t attr;
  int ret;
  if((ret = pthread_attr_init(&attr)) != 0)
      return -ret;
  if((ret = pthread_attr_setstacksize(&attr, task->stacksize)) != 0)
      return -ret;
  if((ret = pthread_attr_setschedpolicy(&attr, policy)) != 0)
      return -ret;
  if((ret = pthread_attr_setschedparam(&attr, &param)) != 0)
      return -ret;
  if((ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0)
      return -ret;
  if(nprocs > 1) {
      int rt_cpu_number = find_rt_cpu_number(task->prio);
      if(rt_cpu_number != -1) {
#ifdef __FreeBSD__
          cpuset_t cpuset;
#else
          cpu_set_t cpuset;
#endif
          CPU_ZERO(&cpuset);
          CPU_SET(rt_cpu_number, &cpuset);
          if((ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset)) != 0)
               return -ret;
      }
  }
  if((ret = pthread_create(&task->thr, &attr, &wrapper, reinterpret_cast<void*>(task))) != 0)
      return -ret;

  return 0;
}

#define RTAPI_CLOCK (CLOCK_MONOTONIC)

pthread_once_t Posix::key_once = PTHREAD_ONCE_INIT;
pthread_once_t Posix::lock_once = PTHREAD_ONCE_INIT;
pthread_key_t Posix::key;
pthread_mutex_t Posix::thread_lock;

void *Posix::wrapper(void *arg)
{
  struct rtapi_task *task;

  /* use the argument to point to the task data */
  task = (struct rtapi_task*)arg;
  long int period = App().period;
  if(task->period < period) task->period = period;
  task->ratio = task->period / period;
  task->period = task->ratio * period;
  rtapi_print_msg(RTAPI_MSG_INFO, "task %p period = %lu ratio=%u\n",
	  task, task->period, task->ratio);

  pthread_setspecific(key, arg);
  set_namef("rtapi_app:T#%d", task->id);

  Posix &papp = reinterpret_cast<Posix&>(App());
  if(papp.do_thread_lock)
      pthread_mutex_lock(&papp.thread_lock);

  struct timespec now;
  clock_gettime(RTAPI_CLOCK, &now);
  rtapi_timespec_advance(task->nextstart, now, task->period + task->pll_correction);

  /* call the task function with the task argument */
  (task->taskcode) (task->arg);

  rtapi_print("ERROR: reached end of wrapper for task %d\n", task->id);
  return NULL;
}

long long Posix::task_pll_get_reference(void) {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return 0;
    return task->nextstart.tv_sec * 1000000000LL + task->nextstart.tv_nsec;
}

int Posix::task_pll_set_correction(long value) {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    if (value > task->pll_correction_limit) value = task->pll_correction_limit;
    if (value < -(task->pll_correction_limit)) value = -(task->pll_correction_limit);
    task->pll_correction = value;
    return 0;
}

int Posix::task_pause(int) {
    return -ENOSYS;
}

int Posix::task_resume(int) {
    return -ENOSYS;
}

int Posix::task_self() {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    return task->id;
}

void Posix::wait() {
    if(do_thread_lock)
        pthread_mutex_unlock(&thread_lock);
    pthread_testcancel();
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    rtapi_timespec_advance(task->nextstart, task->nextstart, task->period + task->pll_correction);
    struct timespec now;
    clock_gettime(RTAPI_CLOCK, &now);
    if(rtapi_timespec_less(task->nextstart, now))
    {
        if(policy == SCHED_FIFO)
            unexpected_realtime_delay(task);
    }
    else
    {
        int res = rtapi_clock_nanosleep(RTAPI_CLOCK, TIMER_ABSTIME, &task->nextstart, nullptr, &now);
        if(res < 0) perror("clock_nanosleep");
    }
    if(do_thread_lock)
        pthread_mutex_lock(&thread_lock);
}

unsigned char Posix::do_inb(unsigned int port)
{
#ifdef HAVE_SYS_IO_H
    return inb(port);
#else
    return 0;
#endif
}

void Posix::do_outb(unsigned char val, unsigned int port)
{
#ifdef HAVE_SYS_IO_H
    return outb(val, port);
#endif
}

void Posix::do_delay(long ns) {
    struct timespec ts = {0, ns};
    rtapi_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL, NULL);
}
int rtapi_prio_highest(void)
{
    return App().prio_highest();
}

int rtapi_prio_lowest(void)
{
    return App().prio_lowest();
}

int rtapi_prio_next_higher(int prio)
{
    return App().prio_next_higher(prio);
}

int rtapi_prio_next_lower(int prio)
{
    return App().prio_next_lower(prio);
}

long rtapi_clock_set_period(long nsecs)
{
    return App().clock_set_period(nsecs);
}

long RtapiApp::clock_set_period(long nsecs)
{
  if(nsecs == 0) return period;
  if(period != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "attempt to set period twice\n");
      return -EINVAL;
  }
  period = nsecs;
  return period;
}


int rtapi_task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
    return App().task_new(taskcode, arg, prio, owner, stacksize, uses_fp);
}

int rtapi_task_delete(int id) {
    return App().task_delete(id);
}

int rtapi_task_start(int task_id, unsigned long period_nsec)
{
    int ret = App().task_start(task_id, period_nsec);
    if(ret != 0) {
        errno = -ret;
        perror("rtapi_task_start()");
    }
    return ret;
}

int rtapi_task_pause(int task_id)
{
    return App().task_pause(task_id);
}

int rtapi_task_resume(int task_id)
{
    return App().task_resume(task_id);
}

int rtapi_task_self()
{
    return App().task_self();
}

long long rtapi_task_pll_get_reference(void)
{
    return App().task_pll_get_reference();
}

int rtapi_task_pll_set_correction(long value)
{
    return App().task_pll_set_correction(value);
}

void rtapi_wait(void)
{
    App().wait();
}

void rtapi_outb(unsigned char byte, unsigned int port)
{
    App().do_outb(byte, port);
}

unsigned char rtapi_inb(unsigned int port)
{
    return App().do_inb(port);
}

long int simple_strtol(const char *nptr, char **endptr, int base) {
  return strtol(nptr, endptr, base);
}

int Posix::run_threads(int fd, int(*callback)(int fd)) {
    while(callback(fd)) { /* nothing */ }
    return 0;
}

int sim_rtapi_run_threads(int fd, int (*callback)(int fd)) {
    return App().run_threads(fd, callback);
}

long long rtapi_get_time() {
    return App().do_get_time();
}

void default_rtapi_msg_handler(msg_level_t level, const char *fmt, va_list ap) {
    if(main_thread && pthread_self() != main_thread) {
        message_t m;
        m.level = level;
        vsnprintf(m.msg, sizeof(m.msg), fmt, ap);
        rtapi_msg_queue.push(m);
    } else {
        vfprintf(level == RTAPI_MSG_ALL ? stdout : stderr, fmt, ap);
    }
}

long int rtapi_delay_max() { return 10000; }

void rtapi_delay(long ns) {
    if(ns > rtapi_delay_max()) ns = rtapi_delay_max();
    App().do_delay(ns);
}

const unsigned long ONE_SEC_IN_NS = 1000000000;
void rtapi_timespec_advance(struct timespec &result, const struct timespec &src, unsigned long nsec)
{
    time_t sec = src.tv_sec;
    while(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    nsec += src.tv_nsec;
    if(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    result.tv_sec = sec;
    result.tv_nsec = nsec;
}

int rtapi_open_as_root(const char *filename, int mode) {
    WITH_ROOT;
    int r = open(filename, mode);
    if(r < 0) return -errno;
    return r;
}

int rtapi_spawn_as_root(pid_t *pid, const char *path,
    const posix_spawn_file_actions_t *file_actions,
    const posix_spawnattr_t *attrp,
    char *const argv[], char *const envp[])
{
    return posix_spawn(pid, path, file_actions, attrp, argv, envp);
}

int rtapi_spawnp_as_root(pid_t *pid, const char *path,
    const posix_spawn_file_actions_t *file_actions,
    const posix_spawnattr_t *attrp,
    char *const argv[], char *const envp[])
{
    return posix_spawnp(pid, path, file_actions, attrp, argv, envp);
}
