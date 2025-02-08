#    This is 'halcompile', a tool to write HAL boilerplate
#    Copyright 2006 Jeff Epler <jepler@unpythonic.net>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

%%
parser Hal:
    ignore: "//.*"
    ignore: "/[*](.|\n)*?[*]/"
    ignore: "[ \t\r\n]+"

    token END: ";;"
    token PARAMDIRECTION: "rw|r"
    token PINDIRECTION: "in|out|io"
    token TYPE: "float|bit|signed|unsigned|u32|s32|u64|s64|port"
    token NAME: "[a-zA-Z_][a-zA-Z0-9_]*"
    token STARREDNAME: "[*]*[a-zA-Z_][a-zA-Z0-9_]*"
    token HALNAME: "[#a-zA-Z_][-#a-zA-Z0-9_.]*"
    token FPNUMBER: "-?([0-9]*\\.[0-9]+|[0-9]+\\.?)([Ee][+-]?[0-9]+)?f?"
    token NUMBER: "0x[0-9a-fA-F]+|[+-]?[0-9]+"
    token STRING: "\"(\\.|[^\\\"])*\""
    token HEADER: "<.*?>"
    token POP: "[-()+*/:?]|&&|\\|\\||personality|==|&|!=|<<|<|<=|>>|>|>="
    token TSTRING: "r?\"\"\"(\\.|\\\n|[^\\\"]|\"(?!\"\")|\n)*\"\"\""

    rule File: ComponentDeclaration Declaration* "$" {{ return True }}
    rule ComponentDeclaration:
        "component" NAME OptString";" {{ comp(NAME, OptString); }}
    rule Declaration:
        "pin" PINDIRECTION TYPE HALNAME OptArray OptSAssign OptPersonality OptString ";"  {{ pin(HALNAME, TYPE, OptArray, PINDIRECTION, OptString, OptSAssign, OptPersonality) }}
      | "param" PARAMDIRECTION TYPE HALNAME OptArray OptSAssign OptPersonality OptString ";" {{ param(HALNAME, TYPE, OptArray, PARAMDIRECTION, OptString, OptSAssign, OptPersonality) }}
      | "function" NAME OptFP OptString ";"       {{ function(NAME, OptFP, OptString) }}
      | "variable" NAME STARREDNAME OptSimpleArray OptAssign ";" {{ variable(NAME, STARREDNAME, OptSimpleArray, OptAssign) }}
      | "option" NAME OptValue ";"   {{ option(NAME, OptValue) }}
      | "see_also" String ";"   {{ see_also(String) }}
      | "notes" String ";"   {{ notes(String) }}
      | "description" String ";"   {{ description(String) }}
      | "examples" String ";"   {{ examples(String) }}
      | "license" String ";"   {{ license(String) }}
      | "author" String ";"   {{ author(String) }}
      | "include" Header ";"   {{ include(Header) }}
      | "modparam" NAME {{ NAME1=NAME; }} NAME OptSAssign OptString ";" {{ modparam(NAME1, NAME, OptSAssign, OptString) }}

    rule Header: STRING {{ return STRING }} | HEADER {{ return HEADER }}

    rule String: TSTRING {{ return eval(TSTRING) }} 
            | STRING {{ return eval(STRING) }}
 
    rule OptPersonality: "if" Personality {{ return Personality }}
            | {{ return None }}
    rule Personality: {{ pp = [] }} (PersonalityPart {{ pp.append(PersonalityPart) }} )* {{ return " ".join(pp) }}
    rule PersonalityPart: NUMBER {{ return NUMBER }}
            | POP {{ return POP }}
    rule OptSimpleArray: "\\[" NUMBER "\\]" {{ return int(NUMBER) }}
            | {{ return 0 }}
    rule OptArray: "\\[" NUMBER OptArrayPersonality "\\]" {{ return OptArrayPersonality and (int(NUMBER), OptArrayPersonality) or int(NUMBER) }}
            | {{ return 0 }}
    rule OptArrayPersonality: ":" Personality {{ return Personality }}
            | {{ return None }} 
    rule OptString: TSTRING {{ return eval(TSTRING) }} 
            | STRING {{ return eval(STRING) }}
            | {{ return '' }}
    rule OptAssign: "=" Value {{ return Value; }}
                | {{ return None }}
    rule OptSAssign: "=" SValue {{ return SValue; }}
                | {{ return None }}
    rule OptFP: "fp" {{ return 1 }} | "nofp" {{ return 0 }} | {{ return 1 }}
    rule Value: "yes" {{ return 1 }} | "no" {{ return 0 }}  
                | "true" {{ return 1 }} | "false" {{ return 0 }}  
                | "TRUE" {{ return 1 }} | "FALSE" {{ return 0 }}  
                | NAME {{ return NAME }}
                | FPNUMBER {{ return float(FPNUMBER.rstrip("f")) }}
                | NUMBER {{ return int(NUMBER,0) }}
    rule SValue: "yes" {{ return "yes" }} | "no" {{ return "no" }}  
                | "true" {{ return "true" }} | "false" {{ return "false" }}  
                | "TRUE" {{ return "TRUE" }} | "FALSE" {{ return "FALSE" }}  
                | NAME {{ return NAME }}
                | FPNUMBER {{ return FPNUMBER }}
                | NUMBER {{ return NUMBER }}
    rule OptValue: Value {{ return Value }}
                | TSTRING {{ return eval(TSTRING) }}
                | STRING {{ return eval(STRING) }}
                | {{ return 1 }}
    rule OptSValue: SValue {{ return SValue }}
                | {{ return 1 }}
%%

import os, sys, tempfile, shutil, getopt, time
BASE = os.path.abspath(os.path.join(os.path.dirname(sys.argv[0]), ".."))
BINDIR = os.getenv('USER_MODULE_DIR', None) or os.path.join(BASE, "bin")
sys.path.insert(0, os.path.join(BASE, "lib", "python"))

MAX_USERSPACE_NAMES = 16 # for userspace (loadusr) components
# NOTE: names are assigned dynamically for realtime components (loadrt)

# Components that use 'personality' features statically allocate
# memory based on MAX_PERSONALITIES (RTAPI_MP_ARRAY_INT)
# The number can be set with the cmdline option -P|--personalities
# Smaller values may be useful since the index of the personality
# exported is computed modulo MAX_PERSONALITIES
MAX_PERSONALITIES = 64

mp_decl_map = {'int': 'RTAPI_MP_INT', 'dummy': None}

# These are symbols that comp puts in the global namespace of the C file it
# creates.  The user is thus not allowed to add any symbols with these
# names.  That includes not only global variables and functions, but also
# HAL pins & parameters, because comp adds #defines with the names of HAL
# pins & params.
reserved_names = [ 'comp_id', 'fperiod', 'rtapi_app_main', 'rtapi_app_exit', 'extra_setup', 'extra_cleanup' ]

def _parse(rule, text, filename=None):
    global P, S
    S = HalScanner(text, filename=filename)
    P = Hal(S)
    return runtime.wrap_error_reporter(P, rule)

def parse(filename):
    initialize()
    f = open(filename).read()
    if '\r' in f:
        if require_unix_line_endings:
            raise SystemExit("%s:0: Error: File contains DOS-style or Mac-style line endings." % filename)
        else:
            print("%s:0: Warning: File contains DOS-style or Mac-style line endings." % filename, file=sys.stderr)
        f = open(filename, "rU").read()
    a, b = f.split("\n;;\n", 1)
    p = _parse('File', a + "\n\n", filename)
    if not p: raise SystemExit(1)
    if require_license:
        if not finddoc('license'):
            raise SystemExit("%s:0: License not specified" % filename)
    return a, b

dirmap = {'r': 'HAL_RO', 'rw': 'HAL_RW', 'in': 'HAL_IN', 'out': 'HAL_OUT', 'io': 'HAL_IO' }
typemap = {'signed': 's32', 'unsigned': 'u32'}
deprmap = {'s32': 'signed', 'u32': 'unsigned'}
deprecated = ['s32', 'u32']

def initialize():
    global functions, params, pins, comp_name, names, docs, variables
    global modparams, includes

    functions = []; params = []; pins = []; options = {}; variables = []
    modparams = []; docs = []; includes = [];
    comp_name = None

    names = {}

def Warn(msg, *args):
    if args:
        msg = msg % args
    print("%s:%d: Warning: %s" % (S.filename, S.line, msg), file=sys.stderr)

def Error(msg, *args):
    if args:
        msg = msg % args
    raise runtime.SyntaxError(S.get_pos(), msg, None)

def comp(name, doc):
    docs.append(('component', name, doc))
    global comp_name
    if comp_name:
        Error("Duplicate specification of component name")
    comp_name = name;

def description(doc):
    docs.append(('descr', doc));

def examples(doc):
    docs.append(('examples', doc));

def license(doc):
    docs.append(('license', doc));

def author(doc):
    docs.append(('author', doc));

def see_also(doc):
    docs.append(('see_also', doc));

def notes(doc):
    docs.append(('notes', doc));

def type2type(type):
    # When we start warning about s32/u32 this is where the warning goes
    return typemap.get(type, type)
    
def checkarray(name, array):
    hashes = len(re.findall("#+", name))
    if array:
        if hashes == 0: Error("Array name contains no #: %r" % name)
        if hashes > 1: Error("Array name contains more than one block of #: %r" % name)
    else:
        if hashes > 0: Error("Non-array name contains #: %r" % name)

def check_name_ok(name):
    if name in reserved_names:
        Error("Variable name %s is reserved" % name)
    if name in names:
        Error("Duplicate item name %s" % name)

def pin(name, type, array, dir, doc, value, personality):
    checkarray(name, array)
    type = type2type(type)
    check_name_ok(name)
    docs.append(('pin', name, type, array, dir, doc, value, personality))
    names[name] = None
    pins.append((name, type, array, dir, value, personality))

def param(name, type, array, dir, doc, value, personality):
    checkarray(name, array)
    type = type2type(type)
    check_name_ok(name)
    docs.append(('param', name, type, array, dir, doc, value, personality))
    names[name] = None
    params.append((name, type, array, dir, value, personality))

def function(name, fp, doc):
    check_name_ok(name)
    docs.append(('funct', name, fp, doc))
    names[name] = None
    functions.append((name, fp))

def option(name, value):
    if name in options:
        Error("Duplicate option name %s" % name)
    options[name] = value

def variable(type, name, array, default):
    check_name_ok(name)
    names[name] = None
    variables.append((type, name, array, default))

def modparam(type, name, default, doc):
    check_name_ok(name)
    names[name] = None
    modparams.append((type, name, default, doc))

def include(value):
    includes.append((value))

def removeprefix(s,p):
    if s.startswith(p): return s[len(p):]
    return s

def to_hal(name):
    name = re.sub("#+", lambda m: "%%0%dd" % len(m.group(0)), name)
    return name.replace("_", "-").rstrip("-").rstrip(".")
def to_c(name):
    name = re.sub("[-._]*#+", "", name)
    name = name.replace("#", "").replace(".", "_").replace("-", "_")
    return re.sub("_+", "_", name)

def prologue(f):
    print("/* Autogenerated by %s on %s -- do not edit */" % (
        sys.argv[0], time.asctime()), file=f)
    print("""\
#include "rtapi.h"
#ifdef RTAPI
#include "rtapi_app.h"
#endif
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "hal.h"
#include "rtapi_math64.h"

static int comp_id;
""", file=f)
    for name in includes:
        print("#include %s" % name, file=f)

    names = {}

    def q(s):
        s = s.replace("\\", "\\\\")
        s = s.replace("\"", "\\\"")
        s = s.replace("\r", "\\r")
        s = s.replace("\n", "\\n")
        s = s.replace("\t", "\\t")
        s = s.replace("\v", "\\v")
        return '"%s"' % s

    print("#ifdef MODULE_INFO", file=f)
    for v in docs:
        if not v: continue
        v = ":".join(map(str, v))
        print("MODULE_INFO(linuxcnc, %s);" % q(v), file=f)
        license = finddoc('license')
    if license and license[1]:
        print("MODULE_LICENSE(\"%s\");" % license[1].split("\n")[0], file=f)
    print("#endif // MODULE_INFO", file=f)
    print("", file=f)


    has_data = options.get("data")

    has_array = False
    has_personality = False
    for name, type, array, dir, value, personality in pins:
        if array: has_array = True
        if isinstance(array, tuple): has_personality = True
        if personality: has_personality = True
    for name, type, array, dir, value, personality in params:
        if array: has_array = True
        if isinstance(array, tuple): has_personality = True
        if personality: has_personality = True
    for type, name, default, doc in modparams:
        decl = mp_decl_map[type]
        if decl:
            print("%s %s" % (type, name), end=' ', file=f)
            if default: print("= %s;" % default, file=f)
            else: print(";", file=f)
            print("%s(%s, %s);" % (decl, name, q(doc)), file=f)
            
    print("", file=f)
    print("struct __comp_state {", file=f)
    print("    struct __comp_state *_next;", file=f)
    if has_personality:
        print("    int _personality;", file=f)

    for name, type, array, dir, value, personality in pins:
        if array:
            if isinstance(array, tuple): array = array[0]
            print("    hal_%s_t *%s_p[%s];" % (type, to_c(name), array), file=f)
        else:
            print("    hal_%s_t *%s_p;" % (type, to_c(name)), file=f)
        names[name] = 1

    for name, type, array, dir, value, personality in params:
        if array:
            if isinstance(array, tuple): array = array[0]
            print("    hal_%s_t %s_p[%s];" % (type, to_c(name), array), file=f)
        else:
            print("    hal_%s_t %s_p;" % (type, to_c(name)), file=f)
        names[name] = 1

    for type, name, array, value in variables:
        if array:
            print("    %s %s_p[%d];\n" % (type, name, array), file=f)
        else:
            print("    %s %s_p;\n" % (type, name), file=f)
    if has_data:
        print("    void *_data;", file=f)

    print("};", file=f)

    if options.get("userspace"):
        print("#include <stdlib.h>", file=f)

    print("struct __comp_state *__comp_first_inst=0, *__comp_last_inst=0;", file=f)
    
    print("", file=f)
    for name, fp in functions:
        if name in names:
            Error("Duplicate item name: %s" % name)
        print("static void %s(struct __comp_state *__comp_inst, long period);" % to_c(name), file=f)
        names[name] = 1

    print("static int __comp_get_data_size(void);", file=f)
    if options.get("extra_setup"):
        print("static int extra_setup(struct __comp_state *__comp_inst, char *prefix, long extra_arg);", file=f)
    if options.get("extra_cleanup"):
        print("static void extra_cleanup(void);", file=f)

    if not options.get("no_convenience_defines"):
        print("#undef TRUE", file=f)
        print("#define TRUE (1)", file=f)
        print("#undef FALSE", file=f)
        print("#define FALSE (0)", file=f)
        print("#undef true", file=f)
        print("#define true (1)", file=f)
        print("#undef false", file=f)
        print("#define false (0)", file=f)

    print("", file=f)
    if has_personality:
        print("static int export(char *prefix, long extra_arg, long personality) {", file=f)
    else:
        print("static int export(char *prefix, long extra_arg) {", file=f)
    print("    (void)extra_arg;", file=f)
    if len(functions) > 0:
        print("    char buf[HAL_NAME_LEN + 1];", file=f)
    print("    int r = 0;", file=f)
    if has_array:
        print("    int j = 0;", file=f)
    print("    int sz = sizeof(struct __comp_state) + __comp_get_data_size();", file=f)
    print("    struct __comp_state *inst = hal_malloc(sz);", file=f)
    print("    memset(inst, 0, sz);", file=f)
    if has_data:
        print("    inst->_data = (char*)inst + sizeof(struct __comp_state);", file=f)
    if has_personality:
        print("    inst->_personality = personality;", file=f)
    if options.get("extra_setup"):
        print("    r = extra_setup(inst, prefix, extra_arg);", file=f)
        print("    if(r != 0) return r;", file=f)
        # the extra_setup() function may have changed the personality
        if has_personality:
            print("    personality = inst->_personality;", file=f)
    for name, type, array, dir, value, personality in pins:
        if personality:
            print("if(%s) {" % personality, file=f)
        if array:
            if isinstance(array, tuple):
                lim, cnt = array
                print("    if((%s) > (%s)) {" % (cnt, lim), file=f)
                print('        rtapi_print_msg(RTAPI_MSG_ERR,' \
                                '"Pin %s: Requested size %%d exceeds max size %%d\\n",'
                                '(int)%s, (int)%s);' % (name, cnt, lim), file=f)
                print("        return -ENOSPC;", file=f)
                print("    }", file=f)
            else: cnt = array
            print("    for(j=0; j < (%s); j++) {" % cnt, file=f)
            print("        r = hal_pin_%s_newf(%s, &(inst->%s_p[j]), comp_id," % (
                type, dirmap[dir], to_c(name)), file=f)
            print("            \"%%s%s\", prefix, j);" % to_hal("." + name), file=f)
            print("        if(r != 0) return r;", file=f)
            if value is not None:
                print("    *(inst->%s_p[j]) = %s;" % (to_c(name), value), file=f)
            print("    }", file=f)
        else:
            print("    r = hal_pin_%s_newf(%s, &(inst->%s_p), comp_id," % (
                type, dirmap[dir], to_c(name)), file=f)
            print("        \"%%s%s\", prefix);" % to_hal("." + name), file=f)
            print("    if(r != 0) return r;", file=f)
            if value is not None:
                print("    *(inst->%s_p) = %s;" % (to_c(name), value), file=f)
        if personality:
            print("}", file=f)

    for name, type, array, dir, value, personality in params:
        if personality:
            print("if(%s) {" % personality, file=f)
        if array:
            if isinstance(array, tuple):
                lim, cnt = array
                print("    if((%s) > (%s)) {" % (cnt, lim), file=f)
                print('        rtapi_print_msg(RTAPI_MSG_ERR,' \
                                '"Parameter %s: Requested size %%d exceeds max size %%d\\n",'
                                '(int)%s, (int)%s);' % (name, cnt, lim), file=f)
                print("        return -ENOSPC;", file=f)
                print("    }", file=f)
            else: cnt = array
            print("    for(j=0; j < (%s); j++) {" % cnt, file=f)
            print("        r = hal_param_%s_newf(%s, &(inst->%s_p[j]), comp_id," % (
                type, dirmap[dir], to_c(name)), file=f)
            print("            \"%%s%s\", prefix, j);" % to_hal("." + name), file=f)
            print("        if(r != 0) return r;", file=f)
            if value is not None:
                print("    inst->%s_p[j] = %s;" % (to_c(name), value), file=f)
            print("    }", file=f)
        else:
            print("    r = hal_param_%s_newf(%s, &(inst->%s_p), comp_id," % (
                type, dirmap[dir], to_c(name)), file=f)
            print("        \"%%s%s\", prefix);" % to_hal("." + name), file=f)
            if value is not None:
                print("    inst->%s_p = %s;" % (to_c(name), value), file=f)
            print("    if(r != 0) return r;", file=f)
        if personality:
            print("}", file=f)

    for type, name, array, value in variables:
        if value is None: continue
        if array:
            print("    for(j=0; j < %s; j++) {" % array, file=f)
            print("        inst->%s_p[j] = %s;" % (name, value), file=f)
            print("    }", file=f)
        else:
            print("    inst->%s_p = %s;" % (name, value), file=f)

    for name, fp in functions:
        print("    rtapi_snprintf(buf, sizeof(buf), \"%%s%s\", prefix);"\
            % to_hal("." + name), file=f)
        print("    r = hal_export_funct(buf, (void(*)(void *inst, long))%s, inst, %s, 0, comp_id);" % (
            to_c(name), int(fp)), file=f)
        print("    if(r != 0) return r;", file=f)
    print("    if(__comp_last_inst) __comp_last_inst->_next = inst;", file=f)
    print("    __comp_last_inst = inst;", file=f)
    print("    if(!__comp_first_inst) __comp_first_inst = inst;", file=f)
    print("    return 0;", file=f)
    print("}", file=f)

    if options.get("count_function"):
        print("static int get_count(void);", file=f)

    if options.get("rtapi_app", 1):
        if options.get("constructable") and not options.get("singleton"):
            print("static int export_1(char *prefix, char *argstr) {", file=f)
            print("    int arg = simple_strtol(argstr, NULL, 0);", file=f)
            print("    return export(prefix, arg);", file=f)
            print("}"   , file=f)
        if not options.get("singleton") and not options.get("count_function") :
            print("static int default_count=%s, count=0;" \
                % options.get("default_count", 1), file=f)
            if options.get("userspace"):
                print("char *names[%d] = {0,};"%(MAX_USERSPACE_NAMES), file=f)
            else:
                print("RTAPI_MP_INT(count, \"number of %s\");" % comp_name, file=f)
                print("char *names = \"\"; // comma separated names", file=f)
                print("RTAPI_MP_STRING(names, \"names of %s\");" % comp_name, file=f)
        if has_personality:
            init1 = str(int(options.get('default_personality', 0)))
            init = ",".join([init1] * MAX_PERSONALITIES)
            print("static int personality[%d] = {%s};" %(MAX_PERSONALITIES,init), file=f)
            print("RTAPI_MP_ARRAY_INT(personality, %d, \"personality of each %s\");" %(MAX_PERSONALITIES,comp_name), file=f)

            # Return personality value.
            # If requested index exceeds MAX_PERSONALITIES, use modulo indexing and give message
            print("""
            static int p_value(char* cname, char *name, int idx) {
                int ans = personality[idx%%%d];
                if (idx >= %d) {
            """%(MAX_PERSONALITIES,MAX_PERSONALITIES), file=f)
            print("""
                    if (name==NULL) {
                        rtapi_print_msg(RTAPI_MSG_ERR,"%s: instance %d assigned personality=%d(=%#0x)\\n",
                                        cname, idx, ans, ans);
                    } else {
                        rtapi_print_msg(RTAPI_MSG_ERR,"%s: name %s assigned personality=%d(=%#0x)\\n",
                                        cname, name, ans, ans);
                    }
                }
                return ans;
            }
            """, file=f)

        print("int rtapi_app_main(void) {", file=f)
        print("    int r = 0;", file=f)
        if not options.get("singleton"):
            print("    int i;", file=f)
        if options.get("count_function"):
            print("    int count = get_count();", file=f)

        print("    comp_id = hal_init(\"%s\");" % comp_name, file=f)
        print("    if(comp_id < 0) return comp_id;", file=f)

        if options.get("singleton"):
            if has_personality:
                print("    r = export(\"%s\", 0, personality[0]);" % \
                        to_hal(removeprefix(comp_name, "hal_")), file=f)
            else:
                print("    r = export(\"%s\", 0);" % \
                        to_hal(removeprefix(comp_name, "hal_")), file=f)
        elif options.get("count_function"):
            print("    for(i=0; i<count; i++) {", file=f)
            print("        char buf[HAL_NAME_LEN + 1];", file=f)
            print("        rtapi_snprintf(buf, sizeof(buf), " \
                                        "\"%s.%%d\", i);" % \
                    to_hal(removeprefix(comp_name, "hal_")), file=f)
            if has_personality:
                print("        r = export(buf, i, p_value(\"%s\", buf, i) );"%comp_name, file=f)
            else:
                print("        r = export(buf, i);", file=f)
            print("    }", file=f)
        else:
            print("    if(count && names[0]) {", file=f)
            print("        rtapi_print_msg(RTAPI_MSG_ERR," \
                            "\"count= and names= are mutually exclusive\\n\");", file=f)
            print("        return -EINVAL;", file=f)
            print("    }", file=f)
            print("    if(!count && !names[0]) count = default_count;", file=f)
            print("    if(count) {", file=f)
            print("        for(i=0; i<count; i++) {", file=f)
            print("            char buf[HAL_NAME_LEN + 1];", file=f)
            print("            rtapi_snprintf(buf, sizeof(buf), " \
                                        "\"%s.%%d\", i);" % \
                    to_hal(removeprefix(comp_name, "hal_")), file=f)
            if has_personality:
                print("            r = export(buf, i, p_value(\"%s\", buf, i) );"%comp_name, file=f)
            else:
                print("            r = export(buf, i);", file=f)
            print("            if(r != 0) break;", file=f)
            print("       }", file=f)
            print("    } else {", file=f)
            if options.get("userspace"):
                print("        int max_names = sizeof(names)/sizeof(names[0]);", file=f)
                print("        for(i=0; (i < max_names) && names[i]; i++) {", file=f)
                print("            if (strlen(names[i]) < 1) {", file=f)
                print("                rtapi_print_msg(RTAPI_MSG_ERR, \"names[%d] is invalid (empty string)\\n\", i);", file=f)
                print("                r = -EINVAL;", file=f)
                print("                break;", file=f)
                print("            }", file=f)
                if has_personality:
                    print("            r = export(names[i], i, p_value(\"%s\", names[i], i) );"%comp_name, file=f)
                else:
                    print("            r = export(names[i], i);", file=f)
                print("            if(r != 0) break;", file=f)
                print("       }", file=f)
                print("    }", file=f)
            else:
                print("        size_t i, j;", file=f)
                print("        int idx;", file=f)
                print("        char buf[HAL_NAME_LEN+1];", file=f)
                print("        const size_t length = strlen(names);", file=f)
                print("        for (i = j = idx = 0; i <= length; i++) {", file=f)
                print("            const char c = buf[j] = names[i];", file=f)
                print("            if ((c == ',') || (c == '\\0')) {", file=f)
                print("                buf[j] = '\\0';", file=f)
                if has_personality:
                    print("                r = export(buf, idx, p_value(\"%s\", buf, idx) );"%comp_name, file=f)
                else:
                    print("                r = export(buf, idx);", file=f)
                print("                if(r != 0) {break;}", file=f)
                print("                idx++;", file=f)
                print("                j = 0;", file=f)
                print("            } else {", file=f)
                print("                if (++j == (sizeof(buf) / sizeof(buf[0]))) {", file=f)
                print("                    buf[j - 1] = '\\0';", file=f)
                print("                    rtapi_print_msg(RTAPI_MSG_ERR,\"names: \\\"%s\\\" too long\\n\", buf);", file=f)
                print("                    r = -EINVAL;", file=f)
                print("                    break;", file=f)
                print("                }", file=f)
                print("            }", file=f)
                print("        }", file=f)
                print("    }", file=f)

        if options.get("constructable") and not options.get("singleton"):
            print("    hal_set_constructor(comp_id, export_1);", file=f)
        print("    if(r) {", file=f)
        if options.get("extra_cleanup"):
            print("    extra_cleanup();", file=f)
        print("        hal_exit(comp_id);", file=f)
        print("    } else {", file=f)
        print("        hal_ready(comp_id);", file=f)
        print("    }", file=f)
        print("    return r;", file=f)
        print("}", file=f)

        print("", file=f)
        print("void rtapi_app_exit(void) {", file=f)
        if options.get("extra_cleanup"):
            print("    extra_cleanup();", file=f)
        print("    hal_exit(comp_id);", file=f)
        print("}", file=f)

    if options.get("userspace"):
        print("static void user_mainloop(void);", file=f)
        if options.get("userinit"):
            print("static void userinit(int argc, char **argv);", file=f)

        if not options.get("singleton"):
            print("""
int __comp_parse_count(int *argc, char **argv) {
    int i;
    for (i = 0; i < *argc; i ++) {
        if (strncmp(argv[i], "count=", 6) == 0) {
            errno = 0;
            count = strtoul(&argv[i][6], NULL, 0);
            for (; i+1 < *argc; i ++) {
                argv[i] = argv[i+1];
            }
            argv[i] = NULL;
            (*argc)--;
            if (errno == 0) {
                return 1;
            }
        }
    }
    return 0;
}
""", file=f)
            print("""
int __comp_parse_names(int *argc, char **argv) {
    int i;
    for (i = 0; i < *argc; i ++) {
        if (strncmp(argv[i], "names=", 6) == 0) {
            char *p = &argv[i][6];
            int j;
            for (; i+1 < *argc; i ++) {
                argv[i] = argv[i+1];
            }
            argv[i] = NULL;
            (*argc)--;
            for (j = 0; j < %d; j ++) {
                names[j] = strtok(p, ",");
                p = NULL;
                if (names[j] == NULL) {
                    return 1;
                }
            }
            return 1;
        }
    }
    return 0;
}
"""%MAX_USERSPACE_NAMES, file=f)
        print("int argc=0; char **argv=0;", file=f)
        print("int main(int argc_, char **argv_) {"    , file=f)
        print("    argc = argc_; argv = argv_;", file=f)
        if not options.get("singleton"):
            print("    int found_count, found_names;", file=f)
            print("    found_count = __comp_parse_count(&argc, argv);", file=f)
            print("    found_names = __comp_parse_names(&argc, argv);", file=f)
            print("    if (found_count && found_names) {", file=f)
            print("        rtapi_print_msg(RTAPI_MSG_ERR, \"count= and names= are mutually exclusive\\n\");", file=f)
            print("        return 1;", file=f)
            print("    }", file=f)
        if options.get("userinit", 0):
            print("    userinit(argc, argv);", file=f)
        print("", file=f)
        print("    if(rtapi_app_main() < 0) return 1;", file=f)
        print("    user_mainloop();", file=f)
        print("    rtapi_app_exit();", file=f)
        print("    return 0;", file=f)
        print("}", file=f)

    print("", file=f)
    if not options.get("no_convenience_defines"):
        print("#undef FUNCTION", file=f)
        print("#define FUNCTION(name) static void name(struct __comp_state *__comp_inst, long period)", file=f)
        print("#undef EXTRA_SETUP", file=f)
        print("#define EXTRA_SETUP() static int extra_setup(struct __comp_state *__comp_inst, char *prefix, long extra_arg)", file=f)
        print("#undef EXTRA_CLEANUP", file=f)
        print("#define EXTRA_CLEANUP() static void extra_cleanup(void)", file=f)
        print("#undef fperiod", file=f)
        print("#define fperiod (period * 1e-9)", file=f)
        for name, type, array, dir, value, personality in pins:
            print("#undef %s" % to_c(name), file=f)
            print("#undef %s_ptr" % to_c(name), file=f)
            if array:
                print("#define %s_ptr(i) (__comp_inst->%s_p[i])" % (to_c(name), to_c(name)), file=f)
                if dir == 'in':
                    print("#define %s(i) (0+*(__comp_inst->%s_p[i]))" % (to_c(name), to_c(name)), file=f)
                else:
                    print("#define %s(i) (*(__comp_inst->%s_p[i]))" % (to_c(name), to_c(name)), file=f)
            else:
                print("#define %s_ptr (__comp_inst->%s_p)" % (to_c(name), to_c(name)), file=f)
                if dir == 'in':
                    print("#define %s (0+*__comp_inst->%s_p)" % (to_c(name), to_c(name)), file=f)
                else:
                    print("#define %s (*__comp_inst->%s_p)" % (to_c(name), to_c(name)), file=f)
        for name, type, array, dir, value, personality in params:
            print("#undef %s" % to_c(name), file=f)
            if array:
                print("#define %s(i) (__comp_inst->%s_p[i])" % (to_c(name), to_c(name)), file=f)
            else:
                print("#define %s (__comp_inst->%s_p)" % (to_c(name), to_c(name)), file=f)

        for type, name, array, value in variables:
            name = name.replace("*", "")
            print("#undef %s" % name, file=f)
            print("#define %s (__comp_inst->%s_p)" % (name, name), file=f)

        if has_data:
            print("#undef data", file=f)
            print("#define data (*(%s*)(__comp_inst->_data))" % options['data'], file=f)
        if has_personality:
            print("#undef personality", file=f)
            print("#define personality (__comp_inst->_personality)", file=f)

        if options.get("userspace"):
            print("#undef FOR_ALL_INSTS", file=f)
            if options.get("singleton"):
                print("#define __comp_inst __comp_first_inst", file=f)
                print("#define FOR_ALL_INSTS()", file=f)
            else:
                print("#define FOR_ALL_INSTS() struct __comp_state *__comp_inst; for(__comp_inst = __comp_first_inst; __comp_inst; __comp_inst = __comp_inst->_next)", file=f)
    print("", file=f)
    print("", file=f)

def epilogue(f):
    data = options.get('data')
    print("", file=f)
    if data:
        print("static int __comp_get_data_size(void) { return sizeof(%s); }" % data, file=f)
    else:
        print("static int __comp_get_data_size(void) { return 0; }", file=f)

INSTALL, COMPILE, PREPROCESS, DOCUMENT, INSTALLDOC, VIEWDOC, MODINC = range(7)
modename = ("install", "compile", "preprocess", "document", "installdoc", "viewdoc", "print-modinc")

modinc = None
def find_modinc():
    global modinc
    if modinc: return modinc
    d = os.path.abspath(os.path.dirname(os.path.dirname(sys.argv[0])))
    for e in ['src', 'etc/linuxcnc', '/etc/linuxcnc', 'share/linuxcnc']:
        e = os.path.join(d, e, 'Makefile.modinc')
        if os.path.exists(e):
            modinc = e
            return e
    raise SystemExit("Unable to locate Makefile.modinc")

def build_usr(tempdir, filename, mode, origfilename):
    binname = os.path.basename(os.path.splitext(filename)[0])

    makefile = os.path.join(tempdir, "Makefile")
    f = open(makefile, "w")
    print("%s: %s" % (binname, filename), file=f)
    print("\t$(CC) -I%s -I$(EMC2_HOME)/include -I/usr/include/linuxcnc -URTAPI -U__MODULE__ -DULAPI -Os %s -o $@ $< -Wl,-rpath,$(LIBDIR) -L$(LIBDIR) -llinuxcnchal %s" % (

        os.path.abspath(os.path.dirname(origfilename)),
        options.get("extra_compile_args", ""),
        options.get("extra_link_args", "")), file=f)
    print("include %s" % find_modinc(), file=f)
    f.close()
    result = os.system("cd %s && make -S %s" % (tempdir, binname))
    if result != 0:
        raise SystemExit(os.WEXITSTATUS(result) or 1)
    output = os.path.join(tempdir, binname)
    if mode == INSTALL:
        shutil.copy(output, os.path.join(BINDIR, binname))
    elif mode == COMPILE:
        shutil.copy(output, os.path.join(os.path.dirname(origfilename),binname))

def build_rt(tempdir, filename, mode, origfilename):
    objname = os.path.basename(os.path.splitext(filename)[0] + ".o")
    makefile = os.path.join(tempdir, "Makefile")
    f = open(makefile, "w")
    print("obj-m += %s" % objname, file=f)
    print("include %s" % find_modinc(), file=f)
    print("EXTRA_CFLAGS += -I%s" % os.path.abspath(os.path.dirname(origfilename)), file=f)
    print("EXTRA_CFLAGS += -I%s" % os.path.abspath('.'), file=f)
    f.close()
    if mode == INSTALL:
        target = "modules install"
    else:
        target = "modules"
    result = os.system("cd %s && make -S %s" % (tempdir, target))
    if result != 0:
        raise SystemExit(os.WEXITSTATUS(result) or 1)
    if mode == COMPILE:
        for extension in ".ko", ".so", ".o":
            kobjname = os.path.splitext(filename)[0] + extension
            if os.path.exists(kobjname):
                shutil.copy(kobjname, os.path.basename(kobjname))
                break
        else:
            raise SystemExit("Unable to copy module from temporary directory")

def finddoc(section=None, name=None):
    for item in docs:
        if ((section == None or section == item[0]) and
                (name == None or name == item[1])): return item
    return None

def finddocs(section=None, name=None):
    for item in docs:
        if ((section == None or section == item[0]) and
                (name == None or name == item[1])):
                    yield item

def to_hal_man_unnumbered(s):
    s = "%s.%s" % (comp_name, s)
    s = s.replace("_", "-")
    s = s.rstrip("-")
    s = s.rstrip(".")
    s = re.sub("#+", lambda m: "\\fI" + "M" * len(m.group(0)) + "\\fB", s)
    # s = s.replace("-", "\\-")
    return s


def to_hal_man(s):
    if options.get("singleton"):
        s = "%s.%s" % (comp_name, s)
    else:
        s = "%s.\\fIN\\fB.%s" % (comp_name, s)
    s = s.replace("_", "-")
    s = s.rstrip("-")
    s = s.rstrip(".")
    s = re.sub("#+", lambda m: "\\fI" + "M" * len(m.group(0)) + "\\fB", s)
    # s = s.replace("-", "\\-")
    return s

def document(filename, outfilename):

    a, b = parse(filename)
    if outfilename is None:
        outfilename = os.path.splitext(filename)[0] + ".1" if options.get("userspace") else ".9"

    f = open(outfilename, "w")

    has_personality = False
    for name, type, array, dir, value, personality in pins:
        if personality: has_personality = True
        if isinstance(array, tuple): has_personality = True
    for name, type, array, dir, value, personality in params:
        if personality: has_personality = True
        if isinstance(array, tuple): has_personality = True

    print(""".\\" -*- mode: troff; coding: utf-8 -*-
.\\"*******************************************************************
.\\"
.\\" This file was extracted from %s using halcompile.g.
.\\" Modify the source file.
.\\"
.\\"*******************************************************************
""" % filename, file=f)

    print(".TH %s \"%s\" \"%s\" \"LinuxCNC Documentation\" \"HAL Component\"" % (
        comp_name.upper(), "1" if options.get("userspace") else "9",
        time.strftime("%F")), file=f)

    print(".SH NAME\n", file=f)
    doc = finddoc('component')    
    if doc and doc[2]:
        if '\n' in doc[2]:
            firstline, rest = doc[2].split('\n', 1)
        else:
            firstline = doc[2]
            rest = ''
        print("%s \\- %s" % (doc[1], firstline), file=f)
    else:
        rest = ''
        print("%s" % doc[1], file=f)


    print(".SH SYNOPSIS", file=f)
    if options.get("userspace"):
        print(".B %s" % comp_name, file=f)
    else:
        if rest:
            print(rest, file=f)
        else:
            print(".HP", file=f)
            if options.get("homemod"):
                print("Custom Homing module loaded with \\fB[EMCMOT]HOMEMOD=%s\\fR"%comp_name, file=f)
            elif options.get("tpmod"):
                print("Custom Trajectory Planning module loaded with \\fB[TRAJ]TPMOD=%s\\fR"%comp_name, file=f)
            elif options.get("singleton") or options.get("count_function"):
                if has_personality:
                    print(".B loadrt %s personality=\\fIP\\fB" % comp_name, end='', file=f)
                else:
                    print(".B loadrt %s" % comp_name, end='', file=f)
            else:
                if has_personality:
                    print(".B loadrt %s [count=\\fIN\\fB|names=\\fIname1\\fB[,\\fIname2...\\fB]] [personality=\\fIP1,P2,...\\fB]" % comp_name, end='', file=f)
                else:
                    print(".B loadrt %s [count=\\fIN\\fB|names=\\fIname1\\fB[,\\fIname2...\\fB]]" % comp_name, end='', file=f)
            for type, name, default, doc in modparams:
                print(" [%s=\\fIN\\fB]" % name, end='', file=f)
            print("", file=f)

            hasparamdoc = False
            for type, name, default, doc in modparams:
                if doc: hasparamdoc = True

            if hasparamdoc:
                print(".RS 4", file=f)
                for type, name, default, doc in modparams:
                    print(".TP", file=f)
                    print("\\fB%s\\fR" % name, end='', file=f)
                    if default:
                        print(" [default: %s]" % default, file=f)
                    else:
                        print("", file=f)
                    print(doc, file=f)
                print(".RE", file=f)

        if options.get("constructable") and not options.get("singleton"):
            print(".PP\n.B newinst %s \\fIname\\fB" % comp_name, file=f)

    doc = finddoc('descr')    
    if doc and doc[1]:
        print(".SH DESCRIPTION\n", file=f)
        print("%s" % doc[1], file=f)

    if functions:
        print(".SH FUNCTIONS", file=f)
        for _, name, fp, doc in finddocs('funct'):
            print(".TP", file=f)
            print("\\fB%s\\fR" % to_hal_man(name), end='', file=f)
            if fp:
                print(" (requires a floating-point thread)", file=f)
            else:
                print("", file=f)
            print(doc, file=f)

    lead = ".TP"
    print(".SH PINS", file=f)
    for _, name, type, array, dir, doc, value, personality in finddocs('pin'):
        print(lead, file=f)
        print(".B %s\\fR" % to_hal_man(name), end=' ', file=f)
        print(type, dir, end=' ', file=f)
        if array:
            sz = name.count("#")
            if isinstance(array, tuple):
                print(" (%s=%0*d..%s)" % ("M" * sz, sz, 0, array[1]), end=' ', file=f)
            else:
                print(" (%s=%0*d..%0*d)" % ("M" * sz, sz, 0, sz, array-1), end=' ', file=f)
        if personality:
            print(" [if %s]" % personality, end=' ', file=f)
        if value:
            print("\\fR(default: \\fI%s\\fR)" % value, file=f)
        else:
            print("\\fR", file=f)
        if doc:
            print(doc, file=f)
            lead = ".TP"
        else:
            lead = ".br\n.ns\n.TP"

    lead = ".TP"
    if params:
        print(".SH PARAMETERS", file=f)
        for _, name, type, array, dir, doc, value, personality in finddocs('param'):
            print(lead, file=f)
            print(".B %s\\fR" % to_hal_man(name), end=' ', file=f)
            print(type, dir, end=' ', file=f)
            if array:
                sz = name.count("#")
                if isinstance(array, tuple):
                    print(" (%s=%0*d..%s)" % ("M" * sz, sz, 0, array[1]), end=' ', file=f)
                else:
                    print(" (%s=%0*d..%0*d)" % ("M" * sz, sz, 0, sz, array-1), end=' ', file=f)
            if personality:
                print(" [if %s]" % personality, end=' ', file=f)
            if value:
                print("\\fR(default: \\fI%s\\fR)" % value, file=f)
            else:
                print("\\fR", file=f)
            if doc:
                print(doc, file=f)
                lead = ".TP"
            else:
                lead = ".br\n.ns\n.TP"

    doc = finddoc('examples')
    if doc and doc[1]:
        print(".SH EXAMPLES\n", file=f)
        print("%s" % doc[1], file=f)

    doc = finddoc('see_also')    
    if doc and doc[1]:
        print(".SH SEE ALSO\n", file=f)
        print("%s" % doc[1], file=f)

    doc = finddoc('notes')    
    if doc and doc[1]:
        print(".SH NOTES\n", file=f)
        print("%s" % doc[1], file=f)

    doc = finddoc('author')    
    if doc and doc[1]:
        print(".SH AUTHOR\n", file=f)
        print("%s" % doc[1], file=f)

    doc = finddoc('license')    
    if doc and doc[1]:
        print(".SH LICENSE\n", file=f)
        print("%s" % doc[1], file=f)

def process(filename, mode, outfilename):
    tempdir = tempfile.mkdtemp()
    try:
        if outfilename is None:
            if mode == PREPROCESS:
                outfilename = os.path.splitext(filename)[0] + ".c"
            else:
                outfilename = os.path.join(tempdir,
                    os.path.splitext(os.path.basename(filename))[0] + ".c")

        a, b = parse(filename)
        base_name = os.path.splitext(os.path.basename(outfilename))[0]
        if comp_name != base_name:
            raise SystemExit("Component name (%s) does not match filename (%s)" % (comp_name, base_name))

        f = open(outfilename, "w")

        if options.get("homemod"): options["singleton"] = 1
        if options.get("tpmod"):   options["singleton"] = 1

        if options.get("userinit") and not options.get("userspace"):
            print("Warning: comp '%s' sets 'userinit' without 'userspace', ignoring" % filename, file=sys.stderr)

        if options.get("userspace"):
            if functions:
                raise SystemExit("Userspace components may not have functions")
        if not pins:
            raise SystemExit("Component must have at least one pin")
        prologue(f)
        lineno = a.count("\n") + 3

        if options.get("userspace"):
            if functions:
                raise SystemExit("May not specify functions with a userspace component.")
            f.write("#line %d \"%s\"\n" % (lineno, filename))
            f.write(b)
        else:
            if not functions or "FUNCTION" in b:
                f.write("#line %d \"%s\"\n" % (lineno, filename))
                f.write(b)
            elif len(functions) == 1:
                f.write("FUNCTION(%s) {\n" % functions[0][0])
                f.write("(void)period;\n")
                f.write("#line %d \"%s\"\n" % (lineno, filename))
                f.write(b)
                f.write("}\n")
            else:
                raise SystemExit("Must use FUNCTION() when more than one function is defined")
        epilogue(f)
        f.close()

        if mode != PREPROCESS:
            if options.get("userspace"):
                build_usr(tempdir, outfilename, mode, filename)
            else:
                build_rt(tempdir, outfilename, mode, filename)

    finally:
        shutil.rmtree(tempdir) 

def usage(exitval=0):
    print("""%(name)s: Build, compile, and install LinuxCNC HAL components

Usage:
           %(name)s [--compile|--preprocess|--document|--view-doc] compfile...
    [sudo] %(name)s [--install|--install-doc] compfile...
           %(name)s --compile --userspace cfile...
    [sudo] %(name)s --install --userspace cfile...
    [sudo] %(name)s --install --userspace pyfile...
           %(name)s --print-modinc

Option to set maximum 'personalities' items:
    --personalities=integer_value   (default is %(dflt)d)

Options to add compile and link flags (only for userspace, only for .c files)
    --extra-compile-args="-I/usr/include/..."
    --extra-link-args="-l..."

Do not use [sudo] for RIP installation.

""" % {'name': os.path.basename(sys.argv[0]),'dflt':MAX_PERSONALITIES})
    raise SystemExit(exitval)

def main():
    global require_license
    global MAX_USERSPACE_NAMES
    global MAX_PERSONALITIES
    require_license = True
    global require_unix_line_endings
    require_unix_line_endings = False
    mode = PREPROCESS
    outfile = None
    userspace = False
    global options
    options = {}
    try:
        opts, args = getopt.getopt(sys.argv[1:], "Uluijcpdo:h?P:",
                           ['unix', 'install', 'compile', 'preprocess', 'outfile=',
                            'document', 'help', 'userspace', 'install-doc',
                            'view-doc', 'require-license', 'print-modinc',
                            'personalities=', "extra-compile-args=",
                            'extra-link-args='])
    except getopt.GetoptError:
        usage(1)
    for k, v in opts:
        if k in ("-U", "--unix"):
            require_unix_line_endings = True
        if k in ("-u", "--userspace"):
            userspace = True
        if k in ("-i", "--install"):
            mode = INSTALL
        if k in ("-c", "--compile"):
            mode = COMPILE
        if k in ("-p", "--preprocess"):
            mode = PREPROCESS
        if k in ("-d", "--document"):
            mode = DOCUMENT
        if k in ("-j", "--install-doc"):
            mode = INSTALLDOC
        if k in ("-j", "--view-doc"):
            mode = VIEWDOC
        if k in ("--print-modinc",):
            mode = MODINC
        if k in ("-l", "--require-license"):
            require_license = True
        if k in ("-o", "--outfile"):
            if len(args) != 1:
                raise SystemExit("Cannot specify -o with multiple input files")
            outfile = v 
        if k in ("-P", "--personalities"):
            try:
                MAX_PERSONALITIES = int(v)
                print("MAX_PERSONALITIES=%d"%(MAX_PERSONALITIES))
            except Exception as detail:
                raise SystemExit("Bad value for -P (--personalities)=",v,"\n",detail)
        if k in ("--extra-compile-args"):
            option("extra_compile_args", v)
        if k in ("--extra-link-args"):
            option("extra_link_args", v)
        if k in ("-?", "-h", "--help"):
            usage(0)

    if outfile and mode != PREPROCESS and mode != DOCUMENT:
        raise SystemExit("Can only specify -o when preprocessing or documenting")

    if mode == MODINC:
        if args:
            raise SystemExit(
                "Can not specify input files when using --print-modinc")
        print(find_modinc())
        return 0

    for f in args:
        try:
            basename = os.path.basename(os.path.splitext(f)[0])
            if f.endswith(".comp") and mode == DOCUMENT:
                document(f, outfile)            
            elif f.endswith(".comp") and mode == VIEWDOC:
                tempdir = tempfile.mkdtemp()
                try:
                    outfile = os.path.join(tempdir, basename + ".9")
                    document(f, outfile)
                    os.spawnvp(os.P_WAIT, "man", ["man", outfile])
                finally:
                    shutil.rmtree(tempdir)
            elif f.endswith(".comp") and mode == INSTALLDOC:
                manpath = os.path.join(BASE, "share/man/man9")
                if not os.path.isdir(manpath):
                    manpath = os.path.join(BASE, "docs/man/man9")
                outfile = os.path.join(manpath, basename + ".9")
                print("INSTALLDOC", outfile)
                document(f, outfile)            
            elif f.endswith(".comp"):
                process(f, mode, outfile)
            elif f.endswith(".py") and mode == INSTALL:
                lines = open(f).readlines()
                if lines[0].startswith("#!"): del lines[0]
                lines.insert(0, "#!%s\n" % sys.executable)
                outfile = os.path.join(BINDIR, basename)
                try: os.unlink(outfile)
                except os.error: pass
                open(outfile, "w").writelines(lines)
                os.chmod(outfile, 0o555)
            elif f.endswith(".c") and mode != PREPROCESS:
                initialize()
                tempdir = tempfile.mkdtemp()
                try:
                    shutil.copy(f, tempdir)
                    if userspace:
                        build_usr(tempdir, os.path.join(tempdir, os.path.basename(f)), mode, f)
                    else:
                        build_rt(tempdir, os.path.join(tempdir, os.path.basename(f)), mode, f)
                finally:
                    shutil.rmtree(tempdir) 
            else:
                raise SystemExit("Unrecognized file type for mode %s: %r" % (modename[mode], f))
        except Exception as e:
            try:
                if outfile is not None: os.unlink(outfile)
            except: # os.error:
                pass
            raise
if __name__ == '__main__':
    main()

# vim:sw=4:sts=4:et
