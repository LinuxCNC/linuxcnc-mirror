# Some useful targets:
#
#     default (if no make target is specified): Build the software and
#         the docs (if enabled via configure).
#
#     build-software: Build all the realtime and non-realtime software
#         libraries & executables.
#
#     docs: Build all the documentation that's enabled through configure.
#
#
#     install: Install everything (realtime software, non-realtime
#         software, and docs).
#
#     install-software: Install all architecture-specific things (realtime
#         and non-realtime libraries and executables).
#
#     install-docs: Install all enabled documentation into $(DESTDIR).
#

SHELL:=/bin/bash

default: build-software

install: install-software install-docs

install-software: install-kernel-dep install-kernel-indep install-menu

ifeq ($(origin KERNELRELEASE), undefined)
#MAKEFLAGS += --warn-undefined-variables
endif

EXTRAFLAGS :=
EXTRA_DEBUG ?=

ifndef DESTDIR
DESTDIR :=
endif

# see http://www.cmcrossroads.com/ask-mr-make/6535-tracing-rule-execution-in-gnu-make
# to trace make execution of make in more detail:
# make VV=1
ifeq ("$(origin VV)", "command line")
    OLD_SHELL := $(SHELL)
    SHELL = $(warning Building $@$(if $<, (from $<))$(if $?, ($? newer)))$(OLD_SHELL)
endif

# Delete the default suffix rules
.SUFFIXES:
.PHONY: default build-software userspace modules clean modclean depclean install install-software install-docs python pythonclean cscope cscopeclean

# A "trivial build" is one which should not include dependency information
# either because it should be usable before dependency information can be
# generated or when it is invalid (clean, docclean) or when running as root
# when the user must guarantee in advance that everything is built
# (setuid, install)
ifeq ($(MAKECMDGOALS),)
TRIVIAL_BUILD=no
else
ifeq ($(filter-out docclean clean setuid install tags swish,$(MAKECMDGOALS)),)
TRIVIAL_BUILD=yes
else
TRIVIAL_BUILD=no
endif
endif

# Used to enable x86 only HAL parts
ifeq ($(shell uname -m),i686)
X86ARCH = true
endif
ifeq ($(shell uname -m),x86_64)
X86ARCH = true
endif

# Beautify output
# ---------------------------------------------------------------------------
#
# A simple variant is to prefix commands with $(Q) - that's useful
# for commands that shall be hidden in non-verbose mode.
#
#	$(Q)ln $@ :<
#
# If BUILD_VERBOSE equals 0 then the above command will be hidden.
# If BUILD_VERBOSE equals 1 then the above command is displayed.

ifeq ("$(origin V)", "command line")
  BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
  BUILD_VERBOSE = 0
endif

ifeq ($(BUILD_VERBOSE),1)
  Q =
else
  Q = @
endif

ifeq "$(findstring s,$(filter-out --%, $(MAKEFLAGS)))" ""
ECHO=@echo
VECHO=echo
else
ECHO=@true
VECHO=true
endif

ifeq ($(BASEPWD),)
BASEPWD := $(shell pwd)
export BASEPWD
include Makefile.inc
ifeq ($(origin PYTHONPATH),undefined)
PYTHONPATH:=$(EMC2_HOME)/lib/python
else
PYTHONPATH:=$(EMC2_HOME)/lib/python:$(PYTHONPATH)
endif
export PYTHONPATH
else
include $(BASEPWD)/Makefile.inc
endif
ifeq ($(RTPREFIX),)
$(error Makefile.inc must specify RTPREFIX and other variables)
endif

ifeq ($(TOOL_NML),yes)
  TOOL_NML_FLAG := -DTOOL_NML
  $(info TOOL_NML_FLAG =$(TOOL_NML_FLAG))
else
endif

cc-option = $(shell if $(CC) $(CFLAGS) $(1) -S -o /dev/null -xc /dev/null \
	     > /dev/null 2>&1; then echo "$(1)"; fi ;)
cxx-option = $(shell if $(CXX) $(CXXFLAGS) $(1) -S -o /dev/null -xc++ /dev/null \
            > /dev/null 2>&1; then echo "$(1)"; else echo "$(2)"; fi ;)

ifeq ($(origin KERNELRELEASE),undefined)
# When KERNELRELEASE is not defined, this is the userspace build.
# The "modules" target is the gateway to the kernel module build.
build-software: userspace modules
ifeq ($(RUN_IN_PLACE),yes)
ifneq ($(BUILD_SYS),uspace)
	@if [ -f ../bin/linuxcnc_module_helper ]; then if ! [ `id -u` = 0 -a -O ../bin/linuxcnc_module_helper -a -u ../bin/linuxcnc_module_helper ]; then $(VECHO) "You now need to run 'sudo make setuid' in order to run in place."; fi; fi
else
	@if [ -f ../bin/rtapi_app ]; then if ! [ `id -u` = 0 -a -O  ../bin/rtapi_app -a -u ../bin/rtapi_app ]; then $(VECHO) "You now need to run 'sudo make setuid' in order to run in place with access to hardware."; fi; fi
endif
endif

VERSION=$(shell cat ../VERSION)
dist:
	mkdir ../linuxcnc-$(VERSION)
	(cd ../ && git archive master) | (cd ../linuxcnc-$(VERSION) && tar xf -)
	(cd ../linuxcnc-$(VERSION) && ./debian/configure)
	(cd ../linuxcnc-$(VERSION)/src && ./autogen.sh)
	(cd .. && tar zcf linuxcnc-$(VERSION).tar.gz linuxcnc-$(VERSION))
	rm -rf ../linuxcnc-$(VERSION)

# Print 'entering' all the time
MAKEFLAGS += w

# Create the variables with := so that subsequent += alterations keep it
# as a "substitute at assignment time" variable
TARGETS :=
PYTARGETS :=
GENERATED_MANPAGES :=

# Submakefiles from each of these directories will be included if they exist
SUBDIRS := \
    libnml/linklist libnml/cms libnml/rcs libnml/inifile libnml/os_intf \
    libnml/nml libnml/buffer libnml/posemath libnml \
    \
    rtapi/examples/timer rtapi/examples/semaphore rtapi/examples/shmem \
    rtapi/examples/extint rtapi/examples/fifo rtapi/examples rtapi \
    \
    hal/components hal/drivers hal/drivers/mesa-hostmot2 \
    hal/user_comps/devices hal/user_comps/mb2hal \
    hal/user_comps hal/user_comps/vismach hal/user_comps/vfs11_vfd hal/classicladder hal/utils hal \
    hal/user_comps/vfdb_vfd hal/user_comps/wj200_vfd \
    hal/user_comps/pi500_vfd \
    hal/user_comps/huanyang-vfd \
	hal/user_comps/xhc-whb04b-6 \
    \
    emc/usr_intf/axis emc/usr_intf/touchy emc/usr_intf/stepconf emc/usr_intf/pncconf \
    emc/usr_intf/gremlin emc/usr_intf/gscreen emc/usr_intf/pyui emc/usr_intf/qtvcp \
    emc/usr_intf/gmoccapy emc/usr_intf/qtplasmac emc/usr_intf/mdro\
    emc/usr_intf emc/nml_intf emc/task emc/kinematics emc/tp emc/canterp \
    emc/motion emc/ini emc/rs274ngc emc/sai emc/pythonplugin \
    emc/motion-logger \
    emc/tooldata \
    emc \
    \
    module_helper \
    \
    po \
    \
    ../docs/src ../docs/src/source-highlight \
    \
    ../tests \



ULAPISRCS := rtapi/$(RTPREFIX)_ulapi.c

# Each item in INCLUDES is transformed into a -I directive later on
# The top directory is always included
INCLUDES := .

USERSRCS :=
PROGRAMS :=

# When used like $(call TOxxx, ...) these turn a list of source files
# into the corresponding list of object files, dependency files,
# or both.  When a source file has to be compiled with special flags,
# TOOBJSDEPS is used.  Confusingly, TOOBJSDEPS includes preprocessed source
# file names, but this is what allows 'make src.i' to produce proper
# preprocessed source when src.c needs a customized compile flag.
# See Submakefile.skel for an example.
TOOBJS = $(patsubst %.cc,objects/%.o,$(patsubst %.c,objects/%.o,$(1)))
TODEPS = $(patsubst %.cc,objects/%.d,$(patsubst %.c,objects/%.d,$(1)))
TOOBJSDEPS = $(call TOOBJS,$(1)) $(call TODEPS, $(1)) $(patsubst %.cc,%.ii,$(patsubst %.c,%.i,$(1)))

SUBMAKEFILES := $(patsubst %,%/Submakefile,$(SUBDIRS))
-include $(wildcard $(SUBMAKEFILES))

# This checks that all the things listed in USERSRCS are either C files
# or C++ files
ASSERT_EMPTY = $(if $(1), $(error "Should be empty but is not: $(1)"))
$(call ASSERT_EMPTY,$(filter-out %.c %.cc, $(USERSRCS)))

$(call TOOBJS,$(PYSRCS)) : EXTRAFLAGS += -fPIC -fno-strict-aliasing
USERSRCS += $(PYSRCS)

# Find the list of object files for each type of source file
CUSERSRCS := $(filter %.c,$(USERSRCS))
CXXUSERSRCS := $(filter %.cc,$(USERSRCS))
CUSEROBJS := $(call TOOBJS,$(CUSERSRCS))
CXXUSEROBJS += $(call TOOBJS,$(CXXUSERSRCS))

ifeq ($(TRIVIAL_BUILD),no)
# Find the dependency filenames, then include them all
DEPS := $(sort $(patsubst %.o,%.d,$(CUSEROBJS) $(CXXUSEROBJS)))
READ_DEPS = $(wildcard $(DEPS))
$(shell $(VECHO) 1>&2 Reading $(words $(READ_DEPS))/$(words $(DEPS)) dependency files)
-include $(READ_DEPS)
UNREAD_DEPS = $(filter-out $(READ_DEPS), $(DEPS))
$(shell $(VECHO) 1>&2 Done reading dependencies)
endif

# Each directory in $(INCLUDES) is passed as a -I directory when compiling.
INCLUDE := $(patsubst %,-I%, $(INCLUDES)) -I$(RTDIR)/include
INCLUDE += $(PYTHON_CPPFLAGS)
INCLUDE += $(LIBTIRPC_CFLAGS)

# Compilation options.	Perhaps some of these should come from Makefile.inc? (CXXFLAGS now does)
INTEGER_OVERFLOW_FLAGS := -fwrapv
OPT := -Os $(INTEGER_OVERFLOW_FLAGS)
DEBUG := $(DEBUG) -g -Wall -D_FORTIFY_SOURCE=2
CFLAGS := $(INCLUDE) $(OPT) $(DEBUG) -DULAPI -std=gnu11 -Werror=implicit-function-declaration $(CFLAGS) $(CPPFLAGS) $(EXTRA_DEBUG)
CXXFLAGS := $(INCLUDE) $(OPT) $(DEBUG) -DULAPI -Werror=overloaded-virtual $(CXXFLAGS) $(CPPFLAGS) $(EXTRA_DEBUG)
# In Debian 11, any inclusion of <boost/python.hpp> leads to several
# diagnostics from included headers about deprecated features. LinuxCNC does
# not directly use these deprecated features, but it does use boost::python.
# Silence the warnings just when they occur in files using boost/python.hpp by
# adding SILENCE_BOOST_INTERNAL_DIAGNOSTICS_FLAGS to the object's EXTRAFLAGS
SILENCE_BOOST_INTERNAL_DIAGNOSTICS_FLAGS = -DBOOST_ALLOW_DEPRECATED_HEADERS=1 -DBOOST_BIND_GLOBAL_PLACEHOLDERS=1
CFLAGS   += $(TOOL_NML_FLAG)
CXXFLAGS += $(TOOL_NML_FLAG)

ifeq ($(RUN_IN_PLACE),yes)
LDFLAGS := -L$(LIB_DIR) -Wl,-rpath,$(LIB_DIR) $(LIBTIRPC_LIBS) $(LDFLAGS)
else
LDFLAGS := -Wl,-rpath-link,../lib $(LIBTIRPC_LIBS) $(LDFLAGS)
endif

# Rules to make .o (object) files
$(sort $(CUSEROBJS)) : objects/%.o: %.c
	$(ECHO) Compiling $<
	@mkdir -p $(dir $@)
	@rm -f $@
	$(Q)$(CC) -c $(CFLAGS) $(EXTRAFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@

$(sort $(CXXUSEROBJS)) : objects/%.o: %.cc
	$(ECHO) Compiling $<
	@mkdir -p $(dir $@)
	@rm -f $@
	$(Q)$(CXX) -c $(CXXFLAGS) $(EXTRAFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@

# Rules to make .i (preprocessed) files
$(sort $(patsubst %.c,%.i,$(CUSERSRCS))): %.i: %.c
	$(ECHO) Preprocessing $< to $@
	$(Q)$(CC) -dD $(CFLAGS) $(EXTRAFLAGS) -E $< -o $@

$(sort $(patsubst %.cc,%.ii,$(CXXUSERSRCS))): %.ii: %.cc
	$(ECHO) Preprocessing $< to $@
	$(Q)$(CXX) -dD $(CXXFLAGS) $(EXTRAFLAGS) -E $< -o $@

ifeq ($(TRIVIAL_BUILD),no)
configure: configure.ac
	AUTOGEN_TARGET=configure ./autogen.sh

config.h.in: configure.ac
	AUTOGEN_TARGET=config.h.in ./autogen.sh

config.status: configure
	if [ -f config.status ]; then ./config.status --recheck; else \
	    echo 1>&2 "*** linuxcnc is not configured.	Run './configure' with appropriate flags."; \
	    exit 1; \
	fi
endif

Makefile: config.h
config.h: config.h.in config.status
	@./config.status -q --header=$@

INFILES = \
        ../scripts/linuxcnc ../scripts/realtime \
	../scripts/linuxcnc_info ../scripts/setup_designer \
	../scripts/haltcl ../scripts/rtapi.conf Makefile.inc Makefile.modinc \
	../tcl/linuxcnc.tcl ../scripts/halrun ../scripts/rip-environment \
	../scripts/linuxcncmkdesktop \
	../lib/python/nf.py \
	../share/desktop-directories/linuxcnc-doc.directory \
	../share/desktop-directories/linuxcnc-ref.directory \
	../share/desktop-directories/linuxcnc-cnc.directory \
	../share/menus/CNC.menu \
	../share/applications/linuxcnc.desktop \
	../share/applications/linuxcnc-latency.desktop \
	../share/applications/linuxcnc-latency-histogram.desktop \
	../share/applications/linuxcnc-pncconf.desktop \
	../share/applications/linuxcnc-stepconf.desktop \
	../scripts/linuxcnc_var \
	../scripts/halcmd_twopass \
	../scripts/runtests \

$(INFILES): %: %.in config.status
	@./config.status --file=$@

build-software: $(INFILES)

# For each file to be copied to ../include, its location in the source tree
# is listed here.  Note that due to $(INCLUDE), defined above, the include
# files in the source tree are the ones used when building linuxcnc.  The copy
# in ../include is used when building external components of linuxcnc.
SRCHEADERS := \
    emc/linuxcnc.h \
    emc/ini/emcIniFile.hh \
    emc/ini/iniaxis.hh \
    emc/ini/inijoint.hh \
    emc/ini/inispindle.hh \
    emc/ini/initraj.hh \
    emc/ini/inihal.hh \
    emc/kinematics/cubic.h \
    emc/kinematics/kinematics.h \
    emc/kinematics/switchkins.h \
    emc/kinematics/genhexkins.h \
    emc/kinematics/genserkins.h \
    emc/kinematics/pentakins.h \
    emc/kinematics/pumakins.h \
    emc/tp/tc.h \
    emc/tp/tc_types.h \
    emc/tp/tcq.h \
    emc/tp/tp.h \
    emc/tp/tp_types.h \
    emc/tp/spherical_arc.h \
    emc/tp/blendmath.h \
    emc/motion/emcmotcfg.h \
    emc/motion/motion.h \
    emc/motion/homing.h \
    emc/motion/simple_tp.h \
    emc/motion/state_tag.h \
    emc/motion/usrmotintf.h \
    emc/motion/axis.h \
    emc/nml_intf/canon.hh \
    emc/nml_intf/canon_position.hh \
    emc/nml_intf/emctool.h \
    emc/nml_intf/emc.hh \
    emc/nml_intf/emc_nml.hh \
    emc/nml_intf/emccfg.h \
    emc/nml_intf/emcglb.h \
    emc/nml_intf/emcpos.h \
    emc/nml_intf/emcpose.h \
    emc/nml_intf/interp_return.hh \
    emc/nml_intf/interpl.hh \
    emc/nml_intf/motion_types.h \
    emc/nml_intf/debugflags.h \
    emc/rs274ngc/interp_internal.hh \
    emc/rs274ngc/interp_fwd.hh \
    emc/rs274ngc/interp_base.hh \
    emc/rs274ngc/modal_state.hh \
    emc/rs274ngc/rs274ngc.hh \
    emc/rs274ngc/saicanon.hh \
    hal/hal.h \
    hal/hal_parport.h \
    hal/drivers/mesa-hostmot2/hostmot2-serial.h \
    libnml/buffer/locmem.hh \
    libnml/buffer/memsem.hh \
    libnml/buffer/phantom.hh \
    libnml/buffer/physmem.hh \
    libnml/buffer/recvn.h \
    libnml/buffer/rem_msg.hh \
    libnml/buffer/sendn.h \
    libnml/buffer/shmem.hh \
    libnml/buffer/tcpmem.hh \
    libnml/cms/cms.hh \
    libnml/cms/cms_aup.hh \
    libnml/cms/cms_cfg.hh \
    libnml/cms/cms_dup.hh \
    libnml/cms/cms_srv.hh \
    libnml/cms/cms_up.hh \
    libnml/cms/cms_user.hh \
    libnml/cms/cms_xup.hh \
    libnml/cms/cmsdiag.hh \
    libnml/cms/tcp_opts.hh \
    libnml/cms/tcp_srv.hh \
    libnml/inifile/inifile.h \
    libnml/inifile/inifile.hh \
    libnml/linklist/linklist.hh \
    libnml/nml/cmd_msg.hh \
    libnml/nml/nml.hh \
	libnml/nml/nml_oi.hh \
    libnml/nml/nml_srv.hh \
    libnml/nml/nml_type.hh \
    libnml/nml/nmldiag.hh \
    libnml/nml/nmlmsg.hh \
    libnml/nml/stat_msg.hh \
    libnml/os_intf/_sem.h \
    libnml/os_intf/sem.hh \
    libnml/os_intf/_shm.h \
    libnml/os_intf/shm.hh \
    libnml/os_intf/_timer.h \
    libnml/os_intf/timer.hh \
    libnml/posemath/posemath.h \
    libnml/posemath/gotypes.h \
    libnml/posemath/gomath.h \
    libnml/posemath/sincos.h \
    libnml/rcs/rcs.hh \
    libnml/rcs/rcs_exit.hh \
    libnml/rcs/rcs_print.hh \
    libnml/rcs/rcsversion.h \
    rtapi/rtapi.h \
    rtapi/rtapi_app.h \
    rtapi/rtapi_atomic.h \
    rtapi/rtapi_bitops.h \
    rtapi/rtapi_bool.h \
    rtapi/rtapi_byteorder.h \
    rtapi/rtapi_device.h \
    rtapi/rtapi_firmware.h \
    rtapi/rtapi_gfp.h \
    rtapi/rtapi_io.h \
    rtapi/rtapi_limits.h \
    rtapi/rtapi_list.h \
    rtapi/rtapi_math.h \
    rtapi/rtapi_math_i386.h \
    rtapi/rtapi_math64.h \
    rtapi/rtapi_mutex.h \
    rtapi/rtapi_parport.h \
    rtapi/rtapi_pci.h \
    rtapi/rtapi_slab.h \
    rtapi/rtapi_stdint.h \
    rtapi/rtapi_ctype.h \
    rtapi/rtapi_errno.h \
    rtapi/rtapi_string.h \
    rtapi/rtapi_vsnprintf.h

# the "headers" target installs all the header files in ../include
.PHONY: headers
HEADERS := $(patsubst %,../include/%,$(foreach h,$(SRCHEADERS),$(notdir $h)))
headers: $(HEADERS)

# install header files as part of the build
TARGETS += headers

# Add converting of %.po files
TARGETS += $(patsubst po/%.po, ../share/locale/%/LC_MESSAGES/linuxcnc.mo, $(wildcard po/*.po))
TARGETS += $(patsubst po/gmoccapy/%.po, ../share/locale/%/LC_MESSAGES/gmoccapy.mo, $(wildcard po/gmoccapy/*.po))
TARGETS += $(patsubst po/%.po, objects/%.msg, $(wildcard po/*.po))

# tooledit as standalone app:
TARGETS += ../bin/tooledit
../bin/tooledit: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/tooledit.tcl")>>$@
	$(Q)(echo "standalone_tooledit")>>$@
	$(Q)chmod +x $@

# ngcgui as standalone app:
TARGETS += ../bin/ngcgui
../bin/ngcgui: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/ngcgui.tcl")>>$@
	$(Q)(echo "::ngcgui::standalone_ngcgui")>>$@
	$(Q)chmod +x $@

# pyngcgui as standalone app:
TARGETS += ../bin/pyngcgui
../bin/pyngcgui: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(PYTHON)")>|$@
	$(Q)(echo "import pyngcgui")>>$@
	$(Q)(echo "pyngcgui.standalone_pyngcgui()")>>$@
	$(Q)chmod +x $@

# gremlin_view as standalone app:
TARGETS += ../bin/gremlin_view
../bin/gremlin_view: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(PYTHON)")>|$@
	$(Q)(echo "import gremlin_view")>>$@
	$(Q)(echo "gremlin_view.standalone_gremlin_view()")>>$@
	$(Q)chmod +x $@

# halshow as standalone app:
TARGETS += ../bin/halshow
../bin/halshow: Makefile
	$(Q)rm -f $@
	$(Q)(echo "#!$(TCLSH)")>|$@
	$(Q)(echo "source $(EMC2_TCL_LIB_DIR)/bin/halshow.tcl")>>$@
	$(Q)chmod +x $@

# And make userspace depend on $(TARGETS)
userspace: $(TARGETS)

pythonclean:
	rm -f $(PYTARGETS)
	find ../lib/python -name '*.so' -exec rm {} +
	find . -name __pycache__ | xargs -r rm -r
python: $(PYTARGETS)
userspace: python
inclean:
	$(RM) $(INFILES)
headersclean:
	$(RM) $(HEADERS)
clean: docclean pythonclean cscopeclean headersclean
distclean: clean inclean

# This is the gateway into the crazy world of "kbuild", the linux 2.6 system
# for building kernel modules.	Other kernel module build styles need to be
# accommodated here.
ifeq ($(BUILD_SYS),kbuild)

modules:
	MAKEFLAGS="$(filter-out --warn-undefined-variables,$(MAKEFLAGS))" \
	$(PYTHON) modsilent.py $(MAKE) KBUILD_EXTRA_SYMBOLS=$(moduledir)/Module.symvers -C $(KERNELDIR) M=$(PWD) CC=$(CC) V=$(BUILD_VERBOSE) modules
	-cp Module.symvers *$(MODULE_EXT) ../rtlib/
endif

# These rules clean things up.	'modclean' cleans files generated by 'modules'
# (except that it doesn't remove the modules that were copied to rtlib)
# 'clean' cleans everything but dependency files, and 'depclean' cleans them
# too.
modclean:
	find -name '.*.cmd' -or -name '*.ko' -or -name '*.mod.c' -or -name '*.mod.o' | xargs rm -f
	-rm -rf .tmp_versions
	find . -name .tmp_versions |xargs rm -rf
	-rm -f ../rtlib/*.ko
	-rm -f ../rtlib/*.so

depclean:
	-rm -rf depends

clean: genclean depclean modclean
genclean:
	find . -name '*.o' |xargs rm -f
	-rm -rf objects
	-rm -f $(TARGETS)
	-rm -f $(GENERATED_MANPAGES)
	-rm -f ../rtlib/*.$(MODULE_EXT)
	-rm -f hal/components/conv_*.comp

pycheck-python-files:
	@echo 'Checking *.py files for python3 compatibility...'
	@for d in ./hal; do \
		filelist=$(shell mktemp) && \
		find $$d -name '*.py' -type f > $$filelist && \
		cat $$filelist | xargs /usr/bin/env python3 -m py_compile && \
		rm -f $$filelist; \
	done

pycheck-python-script:
	@echo 'Checking "text/x-python" and "text/x-script.python" (by MIME type) files python3 compatibility...'
	@for d in ../tests; do \
		filelist=$(shell mktemp) && \
		find $$d -type f -exec file -F ':' -i {} \; | grep -e 'text/x-python' -e 'text/x-script.python' | awk -F ':' '{print $$1}' > $$filelist && \
		cat $$filelist | xargs /usr/bin/env python3 -m py_compile && \
		rm -f $$filelist; \
	done

pycheck: pycheck-python-files pycheck-python-script

# So that nothing is built as root, this rule does not depend on the touched
# files (Note that files in depends/ might be rebuilt, and there's little that
# can be done about it)
setuid:
ifeq ($(BUILD_SYS),uspace)
	chown root ../bin/rtapi_app
	chmod 4750 ../bin/rtapi_app
else
	chown root ../bin/pci_write
	chmod 4750 ../bin/pci_write
	chown root ../bin/pci_read
	chmod 4750 ../bin/pci_read
endif
	chown root ../bin/linuxcnc_module_helper
	chmod 4750 ../bin/linuxcnc_module_helper

# These rules allows a header file from this directory to be installed into
# ../include.  A pair of rules like these will exist in the Submakefile
# of each file that contains headers.
$(patsubst %,../include/%,$(wildcard *.h)): ../include/%.h: %.h
	$(Q)-cp $^ $@
$(patsubst %,../include/%,$(wildcard *.hh)): ../include/%.hh: %.hh
	$(Q)-cp $^ $@

INSTALL=install -o root
DIR=$(INSTALL) -d -m 0755
FILE=$(INSTALL) -m 0644
TREE=cp -dR
CONFIGFILE=$(INSTALL) -m 0644
EXE=$(INSTALL) -m 0755
SETUID=$(INSTALL) -m 4755
GLOB=$(wildcard $(1))

XDGDESKTOP = \
    ../share/applications/linuxcnc.desktop \
    ../share/applications/linuxcnc-latency.desktop \
    ../share/applications/linuxcnc-latency-histogram.desktop \
    ../share/applications/linuxcnc-pncconf.desktop \
    ../share/applications/linuxcnc-stepconf.desktop \

ifeq ($(RUN_IN_PLACE),yes)
define ERROR_MESSAGE
You configured run-in-place, but are trying to install.
For an installable version, run configure without --enable-run-in-place
and rebuild
endef
install install-software install-docs:
	$(error $(ERROR_MESSAGE))

MENUS = ../share/menus/CNC.menu \
    ../share/desktop-directories/cnc.directory
rip-install-menus rip-install-menu: $(MENUS) $(XDGDESKTOP)
	mkdir -p $(HOME)/.config/menus/applications-merged
	cp $< $(HOME)/.config/menus/applications-merged
else

DOCS=README.adoc INSTALL.adoc AUTHORS

# FIXME figure out how to install translated docs in subdirs
DOCS_HELP=$(call GLOB,../docs/help/*.adoc)

NC_FILES=$(filter-out %/butterfly.ngc,$(call GLOB,../nc_files/*))
TCL=$(call GLOB,../tcl/*.tcl)
TCL_BIN=$(call GLOB,../tcl/bin/*.tcl) ../tcl/bin/popimage

install-docs:
	$(DIR) $(DESTDIR)$(docsdir)
	$(FILE) $(addprefix ../docs/,$(DOCS)) $(DESTDIR)$(docsdir)
	$(FILE) $(DOCS_HELP) $(DESTDIR)$(docsdir)
	$(FILE) emc/usr_intf/axis/etc/axis_light_background $(DESTDIR)$(docsdir)
	$(FILE) emc/usr_intf/axis/README $(DESTDIR)$(docsdir)/README.axis

install-dirs:
	$(DIR) $(DESTDIR)$(EMC2_RTLIB_DIR) \
		$(DESTDIR)$(sysconfdir)/linuxcnc $(DESTDIR)$(bindir) $(DESTDIR)$(libdir) \
		$(DESTDIR)/$(libdir)/linuxcnc $(DESTDIR)$(includedir)/linuxcnc \
		$(DESTDIR)$(docsdir) $(DESTDIR)$(ncfilesdir) \
		$(DESTDIR)/etc/X11/app-defaults $(DESTDIR)$(tcldir)/bin \
		$(DESTDIR)$(tcldir)/scripts \
		$(DESTDIR)$(mandir)/man1 \
		$(DESTDIR)$(mandir)/man3 \
		$(DESTDIR)$(mandir)/man9 \
		$(DESTDIR)$(tcldir)/msgs \
		$(DESTDIR)$(localedir)/de/LC_MESSAGES \
		$(DESTDIR)$(datadir)/axis/images \
		$(DESTDIR)$(datadir)/axis/tcl \
		$(DESTDIR)$(datadir)/gscreen/images \
		$(DESTDIR)$(datadir)/gscreen/skins \
		$(DESTDIR)$(datadir)/qtvcp/images \
		$(DESTDIR)$(datadir)/qtvcp/screens \
		$(DESTDIR)$(datadir)/qtvcp/panels \
		$(DESTDIR)$(datadir)/qtvcp/widgets_ui \
		$(DESTDIR)$(datadir)/gmoccapy/icons \
		$(DESTDIR)$(datadir)/glade/catalogs \
		$(DESTDIR)$(datadir)/glade/pixmaps/hicolor/22x22/actions \
		$(DESTDIR)$(datadir)/gtksourceview-4/language-specs \
		$(DESTDIR)$(datadir)/linuxcnc/stepconf \
		$(DESTDIR)$(datadir)/linuxcnc/pncconf \
		$(DESTDIR)$(datadir)/linuxcnc/pncconf/pncconf-help \
		$(DESTDIR)$(datadir)/linuxcnc/hallib

install-kernel-indep: install-dirs
	$(FILE) ../docs/man/man1/*.1 $(DESTDIR)$(mandir)/man1
	$(FILE) $(filter-out %/skeleton.3hal, $(wildcard ../docs/man/man3/*.3hal)) $(DESTDIR)$(mandir)/man3
	$(FILE) $(filter-out %/skeleton.3rtapi, $(wildcard ../docs/man/man3/*.3rtapi)) $(DESTDIR)$(mandir)/man3
	$(FILE) $(filter-out %/skeleton.9, $(wildcard ../docs/man/man9/*.9)) $(DESTDIR)$(mandir)/man9
	$(FILE) objects/*.msg $(DESTDIR)$(tcldir)/msgs
	$(EXE) ../scripts/realtime $(DESTDIR)$(prefix)/lib/linuxcnc
	$(EXE) ../scripts/halrun $(DESTDIR)$(bindir)
	$(EXE) ../scripts/halcmd_twopass $(DESTDIR)$(bindir)
	$(EXE) ../scripts/haltcl $(DESTDIR)$(bindir)
	$(EXE) ../scripts/simulate_probe $(DESTDIR)$(bindir)
	$(EXE) ../scripts/sim_pin $(DESTDIR)$(bindir)
	$(FILE) ../*.png ../*.gif $(DESTDIR)$(datadir)/linuxcnc
	$(FILE) ../lib/hallib/*   $(DESTDIR)$(datadir)/linuxcnc/hallib
	$(FILE) ../tcl/*.png $(DESTDIR)$(tcldir)

	# install all the sample configs, including subdirs (tar is required on debian systems, and common on others)
	$(DIR) $(DESTDIR)$(sampleconfsdir)
	((cd ../configs && tar --exclude CVS --exclude .cvsignore --exclude .gitignore -cf - .) | (cd $(DESTDIR)$(sampleconfsdir) && tar -xf -))

	$(EXE) $(filter-out ../bin/rtapi_app ../bin/linuxcnc_module_helper ../bin/pci_write ../bin/pci_read ../bin/test_rtapi_vsnprintf, $(filter ../bin/%,$(TARGETS))) $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc_info $(DESTDIR)$(bindir)
	$(EXE) ../scripts/setup_designer $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcnc_var $(DESTDIR)$(bindir)
	$(EXE) ../scripts/monitor-xhc-hb04 $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-test $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-plot $(DESTDIR)$(bindir)
	$(EXE) ../scripts/latency-histogram $(DESTDIR)$(bindir)
	$(EXE) ../scripts/moveoff_gui $(DESTDIR)$(bindir)
	$(EXE) ../scripts/hal-histogram $(DESTDIR)$(bindir)
	$(EXE) ../scripts/xhc-hb04-accels $(DESTDIR)$(bindir)
	$(EXE) ../scripts/pyvcp_demo $(DESTDIR)$(bindir)
	$(EXE) ../scripts/gladevcp_demo $(DESTDIR)$(bindir)
	$(EXE) ../scripts/linuxcncmkdesktop $(DESTDIR)$(bindir)
	$(EXE) ../bin/update_ini $(DESTDIR)$(bindir)
	$(EXE) ../scripts/halreport $(DESTDIR)$(bindir)
	$(FILE) $(filter ../lib/%.a ../lib/%.so.0,$(TARGETS)) $(DESTDIR)$(libdir)
	cp --no-dereference $(wildcard ../lib/*.so) $(DESTDIR)$(libdir)
	-ldconfig $(DESTDIR)$(libdir)
	$(FILE) ../lib/linuxcnc/canterp.so $(DESTDIR)$(libdir)/linuxcnc/
	$(FILE) $(HEADERS) $(DESTDIR)$(includedir)/linuxcnc/
	$(TREE) $(NC_FILES) $(DESTDIR)$(ncfilesdir)
	$(EXE) ../nc_files/M101 $(DESTDIR)$(ncfilesdir)
	$(FILE) ../tcl/TkLinuxCNC $(DESTDIR)/etc/X11/app-defaults
	$(FILE) Makefile.modinc $(DESTDIR)$(datadir)/linuxcnc
	$(EXE) $(TCL) $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/hal.so $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/linuxcnc.so $(DESTDIR)$(tcldir)
	$(FILE) ../tcl/pkgIndex.tcl $(DESTDIR)$(tcldir)
	$(EXE) $(TCL_BIN) $(DESTDIR)$(tcldir)/bin
	$(FILE) ../tcl/scripts/balloon.tcl ../tcl/scripts/emchelp.tcl $(DESTDIR)$(tcldir)/scripts
	$(EXE) ../tcl/scripts/Set_Coordinates.tcl $(DESTDIR)$(tcldir)/scripts
	$(FILE) ../share/linuxcnc/*.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/stepconf/*.glade $(DESTDIR)$(prefix)/share/linuxcnc/stepconf
	$(FILE) ../share/linuxcnc/touchy.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/gscreen.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/gscreen2.glade $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/gmoccapy/gmoccapy.glade $(DESTDIR)$(prefix)/share/gmoccapy
	$(FILE) ../share/linuxcnc/pncconf/*.glade $(DESTDIR)$(prefix)/share/linuxcnc/pncconf
	$(FILE) ../share/linuxcnc/gremlin_view.ui $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/linuxcnc/popupkeyboard.ui $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../configs/common/linuxcnc.nml     $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../configs/common/linuxcnc_big.nml $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../src/emc/usr_intf/pncconf/pncconf-help/*.txt $(DESTDIR)$(prefix)/share/linuxcnc/pncconf/pncconf-help
	$(FILE) ../src/emc/usr_intf/pncconf/pncconf-help/*.png $(DESTDIR)$(prefix)/share/linuxcnc/pncconf/pncconf-help

	$(FILE) ../lib/python/gladevcp/hal_python.xml $(DESTDIR)$(datadir)/glade/catalogs/
	$(FILE) ../share/glade/pixmaps/hicolor/22x22/actions/widget*.png  $(DESTDIR)$(datadir)/glade/pixmaps/hicolor/22x22/actions/

	$(FILE) ../share/gtksourceview-4/language-specs/*.lang  $(DESTDIR)$(datadir)/gtksourceview-4/language-specs/

	$(FILE) ../src/hal/drivers/mesa-hostmot2/modbus/*.tmpl  $(DESTDIR)$(prefix)/share/linuxcnc/

install-kernel-indep: install-python
install-python: install-dirs
	$(DIR) $(DESTDIR)$(SITEPY) $(DESTDIR)$(SITEPY)/rs274
	$(DIR) $(DESTDIR)$(SITEPY)/touchy
	$(DIR) $(DESTDIR)$(SITEPY)/gscreen
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/designer/
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/writer
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/writer/ext
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/qtplasmac
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/qt_vismach
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/qt_ngcgui
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/qt_pdf
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/lib/gcode_utility
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/plugins
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/widgets
	$(DIR) $(DESTDIR)$(SITEPY)/qtvcp/widgets/widget_icons
	$(DIR) $(DESTDIR)$(SITEPY)/gmoccapy
	$(DIR) $(DESTDIR)$(SITEPY)/gladevcp
	$(DIR) $(DESTDIR)$(SITEPY)/stepconf
	$(DIR) $(DESTDIR)$(SITEPY)/pncconf
	$(DIR) $(DESTDIR)$(SITEPY)/pyui
	$(DIR) $(DESTDIR)$(SITEPY)/plasmac
	$(FILE) ../lib/python/*.py ../lib/python/*.so $(DESTDIR)$(SITEPY)
	$(FILE) ../lib/python/rs274/*.py $(DESTDIR)$(SITEPY)/rs274
	$(FILE) ../lib/python/touchy/*.py $(DESTDIR)$(SITEPY)/touchy
	$(FILE) ../lib/python/gscreen/*.py $(DESTDIR)$(SITEPY)/gscreen
	$(FILE) ../lib/python/qtvcp/*.{py,ui,txt} $(DESTDIR)$(SITEPY)/qtvcp
	$(FILE) ../lib/python/qtvcp/designer/*.txt $(DESTDIR)$(SITEPY)/qtvcp/designer
	$(FILE) ../lib/python/qtvcp/widgets/*.py $(DESTDIR)$(SITEPY)/qtvcp/widgets
	$(FILE) ../lib/python/qtvcp/widgets/widget_icons/*.{png,gif} $(DESTDIR)$(SITEPY)/qtvcp/widgets/widget_icons
	$(FILE) ../lib/python/qtvcp/plugins/*.py $(DESTDIR)$(SITEPY)/qtvcp/plugins
	$(TREE) ../lib/python/qtvcp/lib/* $(DESTDIR)$(SITEPY)/qtvcp/lib
	$(FILE) ../lib/python/gmoccapy/*.py $(DESTDIR)$(SITEPY)/gmoccapy
	$(FILE) ../lib/python/gladevcp/*.{py,png,glade} $(DESTDIR)$(SITEPY)/gladevcp
	$(FILE) ../lib/python/stepconf/*.py $(DESTDIR)$(SITEPY)/stepconf
	$(FILE) ../lib/python/pncconf/*.py $(DESTDIR)$(SITEPY)/pncconf
	$(FILE) ../lib/python/pyui/*.py $(DESTDIR)$(SITEPY)/pyui
	$(FILE) ../lib/python/plasmac/*.py $(DESTDIR)$(SITEPY)/plasmac
	$(EXE) ../bin/stepconf $(DESTDIR)$(bindir)
	$(EXE) ../bin/pncconf $(DESTDIR)$(bindir)
	$(EXE) ../bin/pyui $(DESTDIR)$(bindir)
	$(EXE) ../bin/hal_input $(DESTDIR)$(bindir)
	$(EXE) ../bin/mitsub_vfd $(DESTDIR)$(bindir)
	$(EXE) ../bin/mqtt-publisher $(DESTDIR)$(bindir)
	$(EXE) ../bin/z_level_compensation $(DESTDIR)$(bindir)
	$(EXE) ../bin/pmx485 $(DESTDIR)$(bindir)
	$(EXE) ../bin/sim-torch $(DESTDIR)$(bindir)
	$(EXE) ../bin/pyvcp $(DESTDIR)$(bindir)
	$(EXE) ../bin/gladevcp $(DESTDIR)$(bindir)
	$(EXE) ../bin/axis $(DESTDIR)$(bindir)
	$(EXE) ../bin/axis-remote $(DESTDIR)$(bindir)
	$(EXE) ../bin/debuglevel $(DESTDIR)$(bindir)
	$(EXE) ../bin/linuxcnctop $(DESTDIR)$(bindir)
	$(EXE) ../bin/mdi $(DESTDIR)$(bindir)
	$(EXE) ../bin/hal_manualtoolchange $(DESTDIR)$(bindir)
	$(EXE) ../bin/image-to-gcode $(DESTDIR)$(bindir)
	$(EXE) ../bin/touchy $(DESTDIR)$(bindir)
	$(EXE) ../bin/gscreen $(DESTDIR)$(bindir)
	$(EXE) ../bin/qtvcp $(DESTDIR)$(bindir)
	$(EXE) ../lib/python/qtvcp/designer/install_script $(DESTDIR)$(SITEPY)/qtvcp/designer/
	$(EXE) ../bin/gmoccapy $(DESTDIR)$(bindir)
	$(EXE) ../bin/teach-in $(DESTDIR)$(bindir)
	$(EXE) ../bin/scorbot-er-3 $(DESTDIR)$(bindir)
	$(EXE) $(patsubst %.py,../bin/%,$(VISMACH_PY)) $(DESTDIR)$(bindir)
	$(EXE) $(patsubst %.py,../bin/%,$(QTPLASMAC_PY)) $(DESTDIR)$(bindir)
	$(FILE) ../share/linuxcnc/linuxcnc-wizard.gif $(DESTDIR)$(prefix)/share/linuxcnc
	$(FILE) ../share/axis/images/*.{png,gif,xbm,ngc} $(DESTDIR)$(datadir)/axis/images
	$(FILE) ../share/axis/tcl/*.tcl $(DESTDIR)$(datadir)/axis/tcl
	$(FILE) ../share/gscreen/images/*.gif $(DESTDIR)$(datadir)/gscreen/images
	$(TREE) ../share/gscreen/skins/* $(DESTDIR)$(datadir)/gscreen/skins
	$(TREE) ../share/qtvcp/images/* $(DESTDIR)$(datadir)/qtvcp/images
	$(TREE) ../share/qtvcp/screens/* $(DESTDIR)$(datadir)/qtvcp/screens
	$(TREE) ../share/qtvcp/panels/* $(DESTDIR)$(datadir)/qtvcp/panels
	$(TREE) ../share/qtvcp/widgets_ui/* $(DESTDIR)$(datadir)/qtvcp/widgets_ui
	$(TREE) ../share/gmoccapy/icons/* $(DESTDIR)$(datadir)/gmoccapy/icons

install-kernel-dep:
	$(DIR)	$(DESTDIR)$(moduledir)/linuxcnc \
		$(DESTDIR)$(bindir) \
		$(DESTDIR)$(sysconfdir)/linuxcnc
	$(FILE) ../rtlib/*$(MODULE_EXT) $(DESTDIR)$(EMC2_RTLIB_DIR)
	$(SETUID) ../bin/linuxcnc_module_helper $(DESTDIR)$(bindir)
ifneq "$(BUILD_SYS)" "uspace"
	$(FILE) Module.symvers $(DESTDIR)$(EMC2_RTLIB_DIR)
	$(SETUID) ../bin/pci_write $(DESTDIR)$(bindir)
	$(SETUID) ../bin/pci_read $(DESTDIR)$(bindir)
else
	$(SETUID) ../bin/rtapi_app $(DESTDIR)$(bindir)
endif
	$(FILE) ../scripts/rtapi.conf $(DESTDIR)$(sysconfdir)/linuxcnc
endif # RUN_IN_PLACE

CONF=../configs
COMMON=$(CONF)/common
CONFILES=$(addsuffix /$(1), $(filter-out $(COMMON) $(CONF),\
                                         ${shell find ${CONF} -type d -print}))
install-menu: $(XDGDESKTOP)
	$(DIR) $(DESTDIR)/usr/share/applications/
	for d in $(XDGDESKTOP); do \
	    $(FILE) $$d $(DESTDIR)/usr/share/applications/ ; \
	done

endif # userspace

ifneq ($(KERNELRELEASE),)
include $(BASEPWD)/hal/components/Submakefile
endif

# KERNELRELEASE is nonempty, therefore we are building modules using the
# "kbuild" system.  $(BASEPWD) is used here, instead of relative paths, because
# that's what kbuild seems to require

EXTRA_CFLAGS := $(filter-out -ffast-math,$(RTFLAGS)) -D__MODULE__ -I$(BASEPWD)/../include -I$(BASEPWD) -I$(BASEPWD)/libnml/linklist \
	-I$(BASEPWD)/libnml/cms -I$(BASEPWD)/libnml/rcs -I$(BASEPWD)/libnml/inifile \
	-I$(BASEPWD)/libnml/os_intf -I$(BASEPWD)/libnml/nml -I$(BASEPWD)/libnml/buffer \
	-I$(BASEPWD)/libnml/posemath -I$(BASEPWD)/rtapi -I$(BASEPWD)/hal \
	-I$(BASEPWD)/emc/nml_intf -I$(BASEPWD)/emc/kinematics -I$(BASEPWD)/emc/tp -I$(BASEPWD)/emc/motion \
	-DSEQUENTIAL_SUPPORT -DHAL_SUPPORT -DDYNAMIC_PLCSIZE -DRT_SUPPORT -DOLD_TIMERS_MONOS_SUPPORT -DMODBUS_IO_MASTER \
	-fno-fast-math $(call cc-option,-mieee-fp) -fno-unsafe-math-optimizations \
	-Wno-declaration-after-statement \
	$(INTEGER_OVERFLOW_FLAGS)
ifneq ($(KERNELRELEASE),)
ifeq ($(RTARCH):$(RTAI):$(filter $(RTFLAGS),-msse),x86_64:3:)
EXTRA_CFLAGS += -msse
EXTRA_CFLAGS += -g
endif
endif
EXTRA_CFLAGS += -fno-builtin-sin -fno-builtin-cos -fno-builtin-sincos

ifdef SEQUENTIAL_SUPPORT
EXTRA_CFLAGS += -DSEQUENTIAL_SUPPORT
endif

# For each module, there's an addition to obj-m or obj-$(CONFIG_foo)
# plus a definition of foo-objs, which contains the full path to the
# object file(s) that the module contains.  Unfortunately, this setup pollutes
# the source directory with object files and other temporaries, but I can't
# find a way around it.

# Subdirectory:  rtapi
ifneq ($(BUILD_SYS),uspace)
obj-$(CONFIG_RTAPI) += rtapi.o
rtapi-objs := rtapi/$(RTPREFIX)_rtapi.o
endif

# Subdirectory: rtapi/examples (unneeded?)

# Subdirectory: hal/components
obj-$(CONFIG_BOSS_PLC) += boss_plc.o
boss_plc-objs := hal/components/boss_plc.o $(MATHSTUB)
obj-$(CONFIG_DEBOUNCE) += debounce.o
debounce-objs := hal/components/debounce.o $(MATHSTUB)
obj-$(CONFIG_ENCODER) += encoder.o
encoder-objs := hal/components/encoder.o $(MATHSTUB)
obj-$(CONFIG_ENUM) += enum.o
enum-objs := hal/components/enum.o $(MATHSTUB)
obj-$(CONFIG_COUNTER) += counter.o
counter-objs := hal/components/counter.o $(MATHSTUB)
obj-$(CONFIG_ENCODER_RATIO) += encoder_ratio.o
encoder_ratio-objs := hal/components/encoder_ratio.o $(MATHSTUB)
obj-$(CONFIG_STEPGEN) += stepgen.o
stepgen-objs := hal/components/stepgen.o $(MATHSTUB)
obj-$(CONFIG_LCD) += lcd.o
lcd-objs := hal/components/lcd.o $(MATHSTUB)
obj-$(CONFIG_MATRIX_KB) += matrix_kb.o
matrix_kb-objs := hal/components/matrix_kb.o $(MATHSTUB)
obj-$(CONFIG_MUX_GENERIC) += mux_generic.o
mux_generic-objs := hal/components/mux_generic.o $(MATHSTUB)
obj-$(CONFIG_PWMGEN) += pwmgen.o
pwmgen-objs := hal/components/pwmgen.o $(MATHSTUB)
obj-$(CONFIG_SIGGEN) += siggen.o
siggen-objs := hal/components/siggen.o $(MATHSTUB)
obj-$(CONFIG_PID) += pid.o
pid-objs := hal/components/pid.o $(MATHSTUB)
obj-$(CONFIG_THREADS) += threads.o
threads-objs := hal/components/threads.o $(MATHSTUB)
obj-$(CONFIG_SUPPLY) += supply.o
supply-objs := hal/components/supply.o $(MATHSTUB)
obj-$(CONFIG_SIM_ENCODER) += sim_encoder.o
sim_encoder-objs := hal/components/sim_encoder.o $(MATHSTUB)
obj-$(CONFIG_WEIGHTED_SUM) += weighted_sum.o
weighted_sum-objs := hal/components/weighted_sum.o $(MATHSTUB)
obj-$(CONFIG_WATCHDOG) += watchdog.o
watchdog-objs := hal/components/watchdog.o $(MATHSTUB)
obj-$(CONFIG_MODMATH) += modmath.o
modmath-objs := hal/components/modmath.o $(MATHSTUB)
obj-$(CONFIG_STREAMER) += streamer.o
streamer-objs := hal/components/streamer.o $(MATHSTUB)
obj-$(CONFIG_SAMPLER) += sampler.o
sampler-objs := hal/components/sampler.o $(MATHSTUB)

# Subdirectory: hal/drivers
obj-$(CONFIG_HAL_PARPORT) += hal_parport.o
hal_parport-objs := hal/drivers/hal_parport.o $(MATHSTUB)
ifeq ($(X86ARCH),true)
obj-$(CONFIG_HAL_SPEAKER) += hal_speaker.o
hal_speaker-objs := hal/drivers/hal_speaker.o $(MATHSTUB)
endif
ifneq ($(BUILD_SYS),uspace)
obj-$(CONFIG_PCI_8255) += pci_8255.o
pci_8255-objs := hal/drivers/pci_8255.o
obj-$(CONFIG_HAL_TIRO) += hal_tiro.o
hal_tiro-objs := hal/drivers/hal_tiro.o $(MATHSTUB)
obj-$(CONFIG_HAL_STG) += hal_stg.o
hal_stg-objs := hal/drivers/hal_stg.o $(MATHSTUB)
obj-$(CONFIG_HAL_VTI) += hal_vti.o
hal_vti-objs := hal/drivers/hal_vti.o $(MATHSTUB)
obj-$(CONFIG_HAL_EVOREG) += hal_evoreg.o
hal_evoreg-objs := hal/drivers/hal_evoreg.o $(MATHSTUB)
obj-$(CONFIG_HAL_MOTENC) += hal_motenc.o
hal_motenc-objs := hal/drivers/hal_motenc.o $(MATHSTUB)
obj-$(CONFIG_HAL_AX521H) += hal_ax5214h.o
hal_ax5214h-objs := hal/drivers/hal_ax5214h.o $(MATHSTUB)
obj-$(CONFIG_HAL_SKELETON) += hal_skeleton.o
hal_skeleton-objs := hal/drivers/hal_skeleton.o $(MATHSTUB)
obj-$(CONFIG_OPTO_AC5) += opto_ac5.o
opto_ac5-objs := hal/drivers/opto_ac5.o $(MATHSTUB)
endif
obj-$(CONFIG_HAL_GM) += hal_gm.o
hal_gm-objs := hal/drivers/hal_gm.o $(MATHSTUB)
obj-$(CONFIG_HAL_PPMC) += hal_ppmc.o
hal_ppmc-objs := hal/drivers/hal_ppmc.o $(MATHSTUB)

#Don't attempt to build Pi / Beaglebone GPIO for RTAI
ifeq ($(BUILD_SYS),uspace)
obj-$(CONFIG_HAL_BB_GPIO) += hal_bb_gpio.o
hal_bb_gpio-objs  :=			\
    hal/drivers/hal_bb_gpio.o	\
    $(MATHSTUB)
obj-$(CONFIG_HAL_PI_GPIO) += hal_pi_gpio.o
hal_pi_gpio-objs := 				\
    hal/drivers/hal_pi_gpio.o 		\
    hal/drivers/cpuinfo.o 		\
    $(MATHSTUB)
ifdef LIBGPIOD_VER
obj-$(CONFIG_HAL_GPIO) += hal_gpio.o
hal_gpio-objs  :=			\
    hal/drivers/hal_gpio.o	        \
    $(MATHSTUB)
endif
endif

obj-$(CONFIG_HOSTMOT2) += hostmot2.o hm2_test.o hm2_pci.o hm2_7i43.o hm2_7i90.o setsserial.o
ifeq ($(BUILD_SYS),uspace)
obj-$(CONFIG_HOSTMOT2) += hm2_eth.o hm2_spi.o hm2_rpspi.o hm2_spix.o
endif
hostmot2-objs :=			  \
    hal/drivers/mesa-hostmot2/hostmot2.o  \
    hal/drivers/mesa-hostmot2/abs_encoder.o\
    hal/drivers/mesa-hostmot2/bitfile.o   \
    hal/drivers/mesa-hostmot2/bspi.o	  \
    hal/drivers/mesa-hostmot2/dpll.o	  \
    hal/drivers/mesa-hostmot2/encoder.o   \
    hal/drivers/mesa-hostmot2/inm.o	  \
    hal/drivers/mesa-hostmot2/inmux.o	  \
    hal/drivers/mesa-hostmot2/ioport.o	  \
    hal/drivers/mesa-hostmot2/led.o	  \
    hal/drivers/mesa-hostmot2/pins.o	  \
    hal/drivers/mesa-hostmot2/pktuart.o   \
    hal/drivers/mesa-hostmot2/pwmgen.o	  \
    hal/drivers/mesa-hostmot2/oneshot.o	  \
    hal/drivers/mesa-hostmot2/periodm.o	  \
    hal/drivers/mesa-hostmot2/raw.o	  \
    hal/drivers/mesa-hostmot2/rcpwmgen.o  \
    hal/drivers/mesa-hostmot2/resolver.o  \
    hal/drivers/mesa-hostmot2/sserial.o   \
    hal/drivers/mesa-hostmot2/ssr.o       \
    hal/drivers/mesa-hostmot2/outm.o      \
    hal/drivers/mesa-hostmot2/stepgen.o   \
    hal/drivers/mesa-hostmot2/tp_pwmgen.o \
    hal/drivers/mesa-hostmot2/tram.o	  \
    hal/drivers/mesa-hostmot2/uart.o	  \
    hal/drivers/mesa-hostmot2/watchdog.o  \
    hal/drivers/mesa-hostmot2/xy2mod.o    \
    $(MATHSTUB)
hm2_7i90-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_7i90.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_7i43-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_7i43.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_pci-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_pci.o   \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
hm2_eth-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_eth.o   \
    hal/drivers/mesa-hostmot2/eshellf.o   \
    $(MATHSTUB)
hm2_spi-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_spi.o   \
    hal/drivers/mesa-hostmot2/llio_info.o \
    $(MATHSTUB)
hm2_rpspi-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_rpspi.o \
    hal/drivers/mesa-hostmot2/llio_info.o \
    hal/drivers/mesa-hostmot2/eshellf.o   \
    $(MATHSTUB)
hm2_spix-objs  :=			  \
    hal/drivers/mesa-hostmot2/hm2_spix.o  \
    hal/drivers/mesa-hostmot2/spix_rpi5.o \
    hal/drivers/mesa-hostmot2/spix_rpi3.o \
    hal/drivers/mesa-hostmot2/spix_spidev.o \
    hal/drivers/mesa-hostmot2/llio_info.o \
    hal/drivers/mesa-hostmot2/eshellf.o \
    $(MATHSTUB)
hm2_test-objs :=			  \
    hal/drivers/mesa-hostmot2/hm2_test.o  \
    hal/drivers/mesa-hostmot2/bitfile.o   \
    $(MATHSTUB)
setsserial-objs :=			  \
    hal/drivers/mesa-hostmot2/setsserial.o  \
    $(MATHSTUB)

obj-$(CONFIG_CLASSICLADDER_RT) += classicladder_rt.o
classicladder_rt-objs := hal/classicladder/module_hal.o $(MATHSTUB)
classicladder_rt-objs += hal/classicladder/arithm_eval.o
classicladder_rt-objs += hal/classicladder/arrays.o
classicladder_rt-objs += hal/classicladder/calc.o
classicladder_rt-objs += hal/classicladder/calc_sequential.o
classicladder_rt-objs += hal/classicladder/manager.o
classicladder_rt-objs += hal/classicladder/symbols.o
classicladder_rt-objs += hal/classicladder/vars_access.o

ifdef SEQUENTIAL_SUPPORT
classicladder_rt-objs += hal/classicladder/calc_sequential_rt.o
endif

obj-m += scope_rt.o
scope_rt-objs := hal/utils/scope_rt.o $(MATHSTUB)

obj-m += hal_lib.o
hal_lib-objs := hal/hal_lib.o $(MATHSTUB)

obj-m += trivkins.o
trivkins-objs := emc/kinematics/trivkins.o
trivkins-objs += emc/kinematics/kins_util.o

obj-m += maxkins.o
maxkins-objs := emc/kinematics/maxkins.o

obj-m += rotatekins.o
rotatekins-objs := emc/kinematics/rotatekins.o

obj-m += tripodkins.o
tripodkins-objs := emc/kinematics/tripodkins.o

obj-m += corexykins.o
corexykins-objs := emc/kinematics/corexykins.o

obj-m += lineardeltakins.o
lineardeltakins-objs := emc/kinematics/lineardeltakins.o

obj-m += pentakins.o
pentakins-objs := emc/kinematics/pentakins.o
pentakins-objs += libnml/posemath/_posemath.o
pentakins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-m += rotarydeltakins.o
rotarydeltakins-objs := emc/kinematics/rotarydeltakins.o
rotarydeltakins-objs += libnml/posemath/_posemath.o
rotarydeltakins-objs += libnml/posemath/sincos.o $(MATHSTUB)

obj-m += rosekins.o
rosekins-objs := emc/kinematics/rosekins.o

obj-m += scorbot-kins.o
scorbot-kins-objs := emc/kinematics/scorbot-kins.o

ifeq ($(origin userkfuncs), undefined)
  # use template:
  USERKFUNCS = emc/kinematics/userkfuncs.o
else
  # user specified file for userkfuncs
  # works with rtpreempt only (not rtai)
  ifeq ("$(wildcard $(userkfuncs))","")
    $(error No such file: $(userkfuncs))
  endif
  $(shell touch $(userkfuncs))
  USERKFUNCS = $(subst .c,.o,$(userkfuncs))
  $(info !!!USERKFUNCS=$(USERKFUNCS))
endif

#----------------------------------------------------------------
# SWITCHKINS
obj-m += genhexkins.o
genhexkins-objs := emc/kinematics/genhexkins.o
genhexkins-objs += libnml/posemath/_posemath.o
genhexkins-objs += libnml/posemath/sincos.o $(MATHSTUB)
genhexkins-objs += emc/kinematics/kins_util.o
genhexkins-objs += emc/kinematics/switchkins.o
genhexkins-objs += $(USERKFUNCS)

obj-m += genserkins.o
genserkins-objs := emc/kinematics/genserkins.o
genserkins-objs += emc/kinematics/genserfuncs.o
genserkins-objs += libnml/posemath/gomath.o
genserkins-objs += libnml/posemath/sincos.o $(MATHSTUB)
genserkins-objs += emc/kinematics/kins_util.o
genserkins-objs += emc/kinematics/switchkins.o
genserkins-objs += $(USERKFUNCS)

obj-m += xyzac-trt-kins.o
xyzac-trt-kins-objs := emc/kinematics/xyzac-trt-kins.o
xyzac-trt-kins-objs += emc/kinematics/trtfuncs.o
xyzac-trt-kins-objs += emc/kinematics/kins_util.o
xyzac-trt-kins-objs += emc/kinematics/switchkins.o
xyzac-trt-kins-objs += $(USERKFUNCS)

obj-m += xyzbc-trt-kins.o
xyzbc-trt-kins-objs := emc/kinematics/xyzbc-trt-kins.o
xyzbc-trt-kins-objs += emc/kinematics/trtfuncs.o
xyzbc-trt-kins-objs += emc/kinematics/kins_util.o
xyzbc-trt-kins-objs += emc/kinematics/switchkins.o
xyzbc-trt-kins-objs += $(USERKFUNCS)

obj-m += scarakins.o
scarakins-objs := emc/kinematics/scarakins.o
scarakins-objs += libnml/posemath/_posemath.o
scarakins-objs += libnml/posemath/sincos.o $(MATHSTUB)
scarakins-objs += emc/kinematics/kins_util.o
scarakins-objs += emc/kinematics/switchkins.o
scarakins-objs += $(USERKFUNCS)

obj-m += pumakins.o
pumakins-objs := emc/kinematics/pumakins.o
pumakins-objs += libnml/posemath/_posemath.o
pumakins-objs += libnml/posemath/sincos.o $(MATHSTUB)
pumakins-objs += emc/kinematics/kins_util.o
pumakins-objs += emc/kinematics/switchkins.o
pumakins-objs += $(USERKFUNCS)

obj-m += 5axiskins.o
5axiskins-objs := emc/kinematics/5axiskins.o
5axiskins-objs += libnml/posemath/_posemath.o
5axiskins-objs += libnml/posemath/sincos.o $(MATHSTUB)
5axiskins-objs += emc/kinematics/kins_util.o
5axiskins-objs += emc/kinematics/switchkins.o
5axiskins-objs += $(USERKFUNCS)
#----------------------------------------------------------------

obj-$(CONFIG_MOTMOD) += motmod.o
motmod-objs := emc/kinematics/cubic.o
motmod-objs += emc/motion/axis.o
motmod-objs += emc/motion/motion.o
motmod-objs += emc/motion/command.o
motmod-objs += emc/motion/control.o
motmod-objs += emc/motion/simple_tp.o
motmod-objs += emc/motion/emcmotutil.o
motmod-objs += emc/motion/stashf.o
motmod-objs += emc/motion/dbuf.o

obj-m += homemod.o
homemod-objs := emc/motion/homemod.o
homemod-objs += emc/motion/homing.o

obj-m += tpmod.o
tpmod-objs := emc/tp/tpmod.o
tpmod-objs += emc/tp/tc.o
tpmod-objs += emc/tp/tcq.o
tpmod-objs += emc/tp/tp.o
tpmod-objs += emc/tp/spherical_arc.o
tpmod-objs += emc/tp/blendmath.o
tpmod-objs += emc/nml_intf/emcpose.o
tpmod-objs += libnml/posemath/_posemath.o
tpmod-objs += libnml/posemath/sincos.o $(MATHSTUB)

TORTOBJS = $(foreach file,$($(patsubst %.o,%,$(1))-objs), objects/rt$(file))
ifeq ($(BUILD_SYS),uspace)
EXTRA_CFLAGS += -fPIC -Os $(filter -Wextra,$(CFLAGS))
RTOBJS := $(sort $(foreach mod,$(obj-m),$(call TORTOBJS,$(mod))))
RTDEPS := $(sort $(RTOBJS:.o=.d))

modules: $(patsubst %.o,../rtlib/%.so,$(obj-m))
../rtlib/%.so:
	$(ECHO) Linking $@
	$(Q)ld -d -r -o objects/$*.tmp $^
	$(Q)objcopy -j .rtapi_export -O binary objects/$*.tmp objects/$*.sym
	$(Q)(echo '{ global : '; tr -s '\0' < objects/$*.sym | xargs -r0 printf '%s;\n' | grep .; echo 'local : * ; };') > objects/$*.ver
	$(Q)$(CC) -shared -Bsymbolic -Wl,--version-script,objects/$*.ver -o $@ $^ -lm $(LDFLAGS)
	$(Q)chmod -x $@

RTFLAGS += -fno-strict-aliasing -fwrapv

# Rules to make .o (object) files
$(sort $(RTOBJS)) : objects/rt%.o : %.c
	$(ECHO) Compiling realtime $<
	@rm -f $@
	@mkdir -p $(dir $@)
	$(Q)$(CC) -c $(OPT) $(DEBUG) $(EXTRA_DEBUG) -DRTAPI \
		$(EXTRA_CFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@
endif

ifeq ($(BUILD_SYS),normal)
modules: $(patsubst %,../rtlib/%,$(obj-m))
RTOBJS := $(sort $(foreach mod,$(obj-m),$(call TORTOBJS,$(mod))))
RTDEPS := $(sort $(patsubst objects/%.o,depends/%.d, $(RTOBJS)))

# Rules to make .o (object) files
$(sort $(RTOBJS)) : objects/rt%.o : %.c
	$(ECHO) Compiling realtime $<
	@rm -f $@
	@mkdir -p $(dir $@)
	$(Q)$(CC) -c -DRTAPI -nostdinc -isystem $(shell $(CC) -print-file-name=include) -I$(KERNELDIR)/include $(EXTRA_CFLAGS) \
		-MP -MD -MF "${@:.o=.d}" -MT "$@" \
		$< -o $@
endif

ifneq "$(filter normal uspace,$(BUILD_SYS))" ""
ifneq "$(BUILD_SYS)" "uspace"
../rtlib/rtapi$(MODULE_EXT): $(addprefix objects/rt,$(rtapi-objs))
endif
../rtlib/classicladder_rt$(MODULE_EXT): $(addprefix objects/rt,$(classicladder_rt-objs))
../rtlib/boss_plc$(MODULE_EXT): $(addprefix objects/rt,$(boss_plc-objs))
../rtlib/debounce$(MODULE_EXT): $(addprefix objects/rt,$(debounce-objs))
../rtlib/encoder$(MODULE_EXT): $(addprefix objects/rt,$(encoder-objs))
../rtlib/enum$(MODULE_EXT): $(addprefix objects/rt,$(enum-objs))
../rtlib/counter$(MODULE_EXT): $(addprefix objects/rt,$(counter-objs))
../rtlib/encoder_ratio$(MODULE_EXT): $(addprefix objects/rt,$(encoder_ratio-objs))
../rtlib/stepgen$(MODULE_EXT): $(addprefix objects/rt,$(stepgen-objs))
../rtlib/lcd$(MODULE_EXT): $(addprefix objects/rt,$(lcd-objs))
../rtlib/matrix_kb$(MODULE_EXT): $(addprefix objects/rt,$(matrix_kb-objs))
../rtlib/mux_generic$(MODULE_EXT): $(addprefix objects/rt,$(mux_generic-objs))
../rtlib/pwmgen$(MODULE_EXT): $(addprefix objects/rt,$(pwmgen-objs))
../rtlib/siggen$(MODULE_EXT): $(addprefix objects/rt,$(siggen-objs))
../rtlib/pid$(MODULE_EXT): $(addprefix objects/rt,$(pid-objs))
../rtlib/threads$(MODULE_EXT): $(addprefix objects/rt,$(threads-objs))
../rtlib/supply$(MODULE_EXT): $(addprefix objects/rt,$(supply-objs))
../rtlib/sim_encoder$(MODULE_EXT): $(addprefix objects/rt,$(sim_encoder-objs))
../rtlib/weighted_sum$(MODULE_EXT): $(addprefix objects/rt,$(weighted_sum-objs))
../rtlib/watchdog$(MODULE_EXT): $(addprefix objects/rt,$(watchdog-objs))
../rtlib/modmath$(MODULE_EXT): $(addprefix objects/rt,$(modmath-objs))
../rtlib/streamer$(MODULE_EXT): $(addprefix objects/rt,$(streamer-objs))
../rtlib/sampler$(MODULE_EXT): $(addprefix objects/rt,$(sampler-objs))
../rtlib/hal_parport$(MODULE_EXT): $(addprefix objects/rt,$(hal_parport-objs))
#../rtlib/uparport$(MODULE_EXT): $(addprefix objects/rt,$(uparport-objs))
ifneq ($(BUILD_SYS),uspace)
../rtlib/pci_8255$(MODULE_EXT): $(addprefix objects/rt,$(pci_8255-objs))
../rtlib/hal_tiro$(MODULE_EXT): $(addprefix objects/rt,$(hal_tiro-objs))
../rtlib/hal_stg$(MODULE_EXT): $(addprefix objects/rt,$(hal_stg-objs))
../rtlib/hal_vti$(MODULE_EXT): $(addprefix objects/rt,$(hal_vti-objs))
../rtlib/hal_evoreg$(MODULE_EXT): $(addprefix objects/rt,$(hal_evoreg-objs))
../rtlib/hal_motenc$(MODULE_EXT): $(addprefix objects/rt,$(hal_motenc-objs))
../rtlib/hal_ax5214h$(MODULE_EXT): $(addprefix objects/rt,$(hal_ax5214h-objs))
../rtlib/hal_skeleton$(MODULE_EXT): $(addprefix objects/rt,$(hal_skeleton-objs))
../rtlib/opto_ac5$(MODULE_EXT): $(addprefix objects/rt,$(opto_ac5-objs))
endif
../rtlib/hal_ppmc$(MODULE_EXT): $(addprefix objects/rt,$(hal_ppmc-objs))
ifeq ($(X86ARCH),true)
../rtlib/hal_speaker$(MODULE_EXT): $(addprefix objects/rt,$(hal_speaker-objs))
endif
../rtlib/scope_rt$(MODULE_EXT): $(addprefix objects/rt,$(scope_rt-objs))
../rtlib/hal_lib$(MODULE_EXT): $(addprefix objects/rt,$(hal_lib-objs))
../rtlib/motmod$(MODULE_EXT): $(addprefix objects/rt,$(motmod-objs))
../rtlib/tpmod$(MODULE_EXT): $(addprefix objects/rt,$(tpmod-objs))
../rtlib/homemod$(MODULE_EXT): $(addprefix objects/rt,$(homemod-objs))
../rtlib/trivkins$(MODULE_EXT): $(addprefix objects/rt,$(trivkins-objs))
../rtlib/5axiskins$(MODULE_EXT): $(addprefix objects/rt,$(5axiskins-objs))
../rtlib/maxkins$(MODULE_EXT): $(addprefix objects/rt,$(maxkins-objs))
../rtlib/rotatekins$(MODULE_EXT): $(addprefix objects/rt,$(rotatekins-objs))
../rtlib/tripodkins$(MODULE_EXT): $(addprefix objects/rt,$(tripodkins-objs))
../rtlib/corexykins$(MODULE_EXT): $(addprefix objects/rt,$(corexykins-objs))
../rtlib/lineardeltakins$(MODULE_EXT): $(addprefix objects/rt,$(lineardeltakins-objs))
../rtlib/pentakins$(MODULE_EXT): $(addprefix objects/rt,$(pentakins-objs))
../rtlib/rotarydeltakins$(MODULE_EXT): $(addprefix objects/rt,$(rotarydeltakins-objs))
../rtlib/rosekins$(MODULE_EXT): $(addprefix objects/rt,$(rosekins-objs))
../rtlib/scorbot-kins$(MODULE_EXT): $(addprefix objects/rt,$(scorbot-kins-objs))
../rtlib/hal_gm$(MODULE_EXT): $(addprefix objects/rt,$(hal_gm-objs))
../rtlib/hostmot2$(MODULE_EXT): $(addprefix objects/rt,$(hostmot2-objs))
../rtlib/hm2_test$(MODULE_EXT): $(addprefix objects/rt,$(hm2_test-objs))
../rtlib/hm2_pci$(MODULE_EXT): $(addprefix objects/rt,$(hm2_pci-objs))
../rtlib/hm2_7i43$(MODULE_EXT): $(addprefix objects/rt,$(hm2_7i43-objs))
../rtlib/hm2_7i90$(MODULE_EXT): $(addprefix objects/rt,$(hm2_7i90-objs))
../rtlib/setsserial$(MODULE_EXT): $(addprefix objects/rt,$(setsserial-objs))
../rtlib/hal_parport$(MODULE_EXT): $(addprefix objects/rt,$(hal_parport-objs))
../rtlib/hal_ppmc$(MODULE_EXT): $(addprefix objects/rt,$(hal_ppmc-objs))
../rtlib/hm2_eth$(MODULE_EXT): $(addprefix objects/rt,$(hm2_eth-objs))
../rtlib/hm2_spi$(MODULE_EXT): $(addprefix objects/rt,$(hm2_spi-objs))
../rtlib/hm2_rpspi$(MODULE_EXT): $(addprefix objects/rt,$(hm2_rpspi-objs))
../rtlib/hm2_spix$(MODULE_EXT): $(addprefix objects/rt,$(hm2_spix-objs))
../rtlib/hal_bb_gpio$(MODULE_EXT): $(addprefix objects/rt,$(hal_bb_gpio-objs))
../rtlib/hal_pi_gpio$(MODULE_EXT): $(addprefix objects/rt,$(hal_pi_gpio-objs))
ifdef LIBGPIOD_VER
../rtlib/hal_gpio$(MODULE_EXT): $(addprefix objects/rt,$(hal_gpio-objs))
endif

#----------------------------------------------------------------
# SWITCHKINS
../rtlib/genhexkins$(MODULE_EXT):     $(addprefix objects/rt,$(genhexkins-objs))
../rtlib/genserkins$(MODULE_EXT):     $(addprefix objects/rt,$(genserkins-objs))
../rtlib/xyzac-trt-kins$(MODULE_EXT): $(addprefix objects/rt,$(xyzac-trt-kins-objs))
../rtlib/xyzbc-trt-kins$(MODULE_EXT): $(addprefix objects/rt,$(xyzbc-trt-kins-objs))
../rtlib/scarakins$(MODULE_EXT):      $(addprefix objects/rt,$(scarakins-objs))
../rtlib/pumakins$(MODULE_EXT):       $(addprefix objects/rt,$(pumakins-objs))
#----------------------------------------------------------------

ifeq ($(TRIVIAL_BUILD),no)
READ_RTDEPS = $(wildcard $(RTDEPS))
$(shell $(VECHO) 1>&2 Reading $(words $(READ_RTDEPS))/$(words $(RTDEPS)) realtime dependency files)
-include $(READ_RTDEPS)
$(shell $(VECHO) 1>&2 Done reading realtime dependencies)
endif
endif

# Phony so that it is always rebuilt when requested, not because it
# shouldn't exist as a file
.PHONY: tags
tags:
	ctags-exuberant \
		--extra=+fq \
		--exclude=depends --exclude=objects --exclude=.mod.c \
		'--langmap=make:+(Submakefile),make:+(Makefile.inc),c:+.comp' \
		-I EXPORT_SYMBOL+,RTAPI_MP_INT+,RTAPI_MP_LONG+,RTAPI_MP_STRING+ \
		-I RTAPI_MP_ARRAY_INT+,RTAPI_MP_ARRAY_LONG+,RTAPI_MP_ARRAY_STRING+ \
		-I MODULE_AUTHOR+,MODULE_DESCRIPTION+,MODULE_LICENSE+ \
		-R . ../tcl ../scripts ../share/axis/tcl
	rm -f TAGS
	find . -type f -name '*.[ch]' -printf '%P\0' |xargs -0 etags -l c --append
	find . -type f -name '*.cc' -printf '%P\0' |xargs -0 etags -l c++ --append
	find . -type f -name '*.hh' -printf '%P\0' |xargs -0 etags -l c++ --append

etags:
	etags	--extra=+fq \
		--exclude=depends --exclude=objects --exclude=.mod.c \
		'--langmap=make:+(Submakefile),make:+(Makefile.inc),c:+.comp' \
		-I EXPORT_SYMBOL+,RTAPI_MP_INT+,RTAPI_MP_LONG+,RTAPI_MP_STRING+ \
		-I RTAPI_MP_ARRAY_INT+,RTAPI_MP_ARRAY_LONG+,RTAPI_MP_ARRAY_STRING+ \
		-I MODULE_AUTHOR+,MODULE_DESCRIPTION+,MODULE_LICENSE+ \
		-R . ../tcl ../scripts ../share/axis/tcl
	find . -type f -name '*.[ch]' |xargs etags --language-force=C --append
	find . -type f -name '*.cc' |xargs etags --language-force=C++ --append
	find . -type f -name '*.hh' |xargs etags --language-force=C++ --append

.PHONY: swish
swish:
	swish-e -c .swish_config -v 0 -i $(BASEPWD) \
		$(dir $(BASEPWD))tcl \
		$(dir $(BASEPWD))share/axis/tcl \
		$(dir $(BASEPWD))lib/python \
		$(dir $(BASEPWD))scripts \
		$(dir $(BASEPWD))configs \
		$(dir $(BASEPWD))docs/src \
		$(dir $(BASEPWD))docs/man/man1

# When you depend on objects/var-ZZZ you are depending on the contents of the
# variable ZZZ, which is assumed to depend on a Makefile, a Submakefile, or
# Makefile.inc
objects/var-%: Makefile $(wildcard $(SUBMAKEFILES)) Makefile.inc
	@mkdir -p $(dir $@)
	@echo $($*) > $@.tmp
	@sh move-if-change $@.tmp $@

../lib/%.so: ../lib/%.so.0
	ln -sf $(notdir $<) $@

cscope:
	cscope -Rb

cscopeclean:
	bash -c 'for f in `find ./ -name "cscope.*out"`;do rm $$f;done'
# vim:ts=8:sts=8:sw=8:noet:
