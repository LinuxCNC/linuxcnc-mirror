/*    This is a component of LinuxCNC
 *    Copyright 2011, 2012, 2013 Michael Haberler <git@mah.priv.at>,
 *    Sebastian Kuzminsky <seb@highlab.com>
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
#ifndef PYTHON_PLUGIN_HH
#define PYTHON_PLUGIN_HH

#ifndef BOOST_PYTHON_MAX_ARITY
#define BOOST_PYTHON_MAX_ARITY 4
#endif
#include <boost/python/object.hpp>

#include <vector>
#include <string>
#include <sys/types.h>


extern std::string handle_pyerror();


// PY_EXCEPTION: both exception_msg and error_msg are set
// PY_ERROR:  error_msg is set
enum pp_status   {
    PLUGIN_NO_SECTION = -1,
    PLUGIN_NO_INIFILE = -2,
    PLUGIN_BAD_INIFILE = -3,
    PLUGIN_NO_TOPLEVEL = -4,
    PLUGIN_BAD_PATH = -5,
    PLUGIN_STAT_FAILED = -6,
    PLUGIN_INITTAB_FAILED = -7 ,
    PLUGIN_PYTHON_ALREADY_INITIALIZED = -8,
    PLUGIN_EXCEPTION_DURING_PATH_PREPEND = -9,
    PLUGIN_EXCEPTION_DURING_PATH_APPEND = -10,
    PLUGIN_INIT_EXCEPTION = -11,
    PLUGIN_PYTHON_NOT_INITIALIZED = -12,
    PLUGIN_PATH_TOO_LONG = -13,

    // errors < OK make run_string(), is_callable(), and call() fail immediately
    // a reload might clear the error
    PLUGIN_OK = 0,
    PLUGIN_NO_CALLABLE = 1,
    PLUGIN_EXCEPTION = 2
};

class PythonPlugin {
public:
    // factory method
    static PythonPlugin *instantiate(struct _inittab *inittab = NULL);
    int configure(const char *iniFilename = NULL, const char *section = NULL);
    bool is_callable(const char *module, const char *funcname);
    int call(const char *module,const char *callable,
	     boost::python::object tupleargs, boost::python::object kwargs, boost::python::object &retval);
    int run_string(const char *cmd, boost::python::object &retval, bool as_file = false);
    int call_method(boost::python::object method, boost::python::object &retval);

    int plugin_status() { return status; };
    bool usable() { return (status >= PLUGIN_OK); }
    int initialize();
    std::string last_exception() { return exception_msg; };
    std::string last_errmsg() { return error_msg; };
    boost::python::object main_namespace;

private:
    PythonPlugin(struct _inittab *inittab);       // nb: no public constructor
    PythonPlugin(const PythonPlugin &) = delete;        // not copyable
    PythonPlugin & operator=(const PythonPlugin&) = delete;  // not assignable
    ~PythonPlugin() {};

    int reload();
    std::vector<std::string> inittab_entries;
    int status;
    time_t module_mtime;                  // toplevel module - last modification time
    bool reload_on_change;                // auto-reload if toplevel module was changed
    const char *toplevel;          // toplevel script
    //    const char *plugin_dir;               // directory prefix
    const char *abs_path;                 // normalized path to toplevel module
    std::string exception_msg;
    std::string error_msg;
    int log_level;
};

#endif
