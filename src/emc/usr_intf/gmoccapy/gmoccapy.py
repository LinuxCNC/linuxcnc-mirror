#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
"""
    A GUI for LinuxCNC based on gladevcp and Python
    Based on the design of moccagui from Tom
    and with a lot of code from gscreen from Chris Morley
    and with the help from Michael Haberler
    and Chris Morley and some more

    Copyright 2012 / 2017 Norbert Schechner
    nieson@web.de

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

"""

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GdkPixbuf
from gi.repository import GLib

import traceback           # needed to launch traceback errors
import hal                 # base hal class to react to hal signals
import hal_glib            # needed to make our own hal pins
import sys                 # handle system calls
import os                  # needed to get the paths and directories
import atexit              # needed to register child's to be closed on closing the GUI
import subprocess          # to launch onboard and other processes
import tempfile            # needed only if the user click new in edit mode to open a new empty file
import linuxcnc            # to get our own error system
import locale              # for setting the language of the GUI
import gettext             # to extract the strings to be translated
from collections import OrderedDict # needed for proper jog button arrangement
from time import strftime  # needed for the clock in the GUI
#from Gtk._Gtk import main_quit

# Throws up a dialog with debug info when an error is encountered
def excepthook(exc_type, exc_obj, exc_tb):
    try:
        w = app.widgets.window1
    except KeyboardInterrupt:
        sys.exit()
    except NameError:
        w = None
    lines = traceback.format_exception(exc_type, exc_obj, exc_tb)
    message ="Found an error!\nThe following information may be useful in troubleshooting:\n\n" + "".join(lines)
    LOG.error(message)
    m = Gtk.MessageDialog(parent = w,
                          modal = True ,
                          destroy_with_parent = True,
                          message_type = Gtk.MessageType.ERROR,
                          text = message,
                          buttons = Gtk.ButtonsType.OK,)

    m.show()
    m.run()
    m.destroy()


sys.excepthook = excepthook

# constants
#         # gmoccapy  #"
_RELEASE = " 3.5.0"
_INCH = 0                         # imperial units are active
_MM = 1                           # metric units are active

# set names for the tab numbers, its easier to understand the code
# Bottom Button Tabs
_BB_MANUAL = 0
_BB_MDI = 1
_BB_AUTO = 2
_BB_HOME = 3
_BB_TOUCH_OFF = 4
_BB_SETUP = 5
_BB_EDIT = 6
_BB_TOOL = 7
_BB_LOAD_FILE = 8
#_BB_HOME_JOINTS will not be used, we will reorder the notebooks to get the correct page shown

# Default button size for bottom buttons
_DEFAULT_BB_SIZE = (90, 56)

_TEMPDIR = tempfile.gettempdir()  # Now we know where the tempdir is, usually /tmp

# set up paths to files
BASE = os.path.abspath(os.path.join(os.path.dirname(sys.argv[0]), ".."))
LIBDIR = os.path.join(BASE, "lib", "python")
sys.path.insert(0, LIBDIR)

# set up paths to files, part two
CONFIGPATH = os.environ['CONFIG_DIR']
DATADIR = os.path.join(BASE, "share", "gmoccapy")
IMAGEDIR = os.path.join(DATADIR, "images")
XMLNAME = os.path.join(DATADIR, "gmoccapy.glade")
THEMEDIR = "/usr/share/themes"
USERTHEMEDIR = os.path.join(os.path.expanduser("~"), ".themes")
LOCALEDIR = os.path.join(BASE, "share", "locale")
ICON_THEME_DIR = os.path.join(DATADIR, "icons")
USER_ICON_THEME_DIR = os.path.join(os.path.expanduser("~"), ".icons")
DEFAULT_ICON_THEME = "classic"
DEFAULT_GCODE_THEME = "classic"

# path to TCL for external programs eg. halshow
TCLPATH = os.environ['LINUXCNC_TCL_DIR']

# the ICONS should must exist in the icon theme
ALERT_ICON = "dialog_warning"
INFO_ICON = "dialog_information"



class gmoccapy(object):
    def __init__(self, argv):

        # prepare for translation / internationalisation
        locale.setlocale(locale.LC_ALL, '')
        locale.bindtextdomain("gmoccapy", LOCALEDIR)
        gettext.install("gmoccapy", localedir=LOCALEDIR)

        # CSS styling
        css = b"""
            button {
                padding: 0;
            }
            #gcode_edit {
                padding: 3px;
                margin: 1px;
            }
            /* #__jog_incr_buttons *:checked {
                background: rgba(230,230,50,0.8);
            } */
            #jog_incr_buttons *:active, #jog_buttons *:active {
                background: rgba(230,230,50,0.8);
            }
            #eb_program_label, #eb_blockheight_label {
                background: rgba(0,0,0,1);
            }
            progress, trough {
                min-height: 5px;
            }
            progress, text {
                font-size: 13.3px;
            }
        """
        screen = Gdk.Screen.get_default()
        provider = Gtk.CssProvider()
        provider.load_from_data(css)
        style_context = Gtk.StyleContext()
        style_context.add_provider_for_screen(screen, provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        # needed components to communicate with hal and linuxcnc
        self.halcomp = hal.component("gmoccapy")
        self.command = linuxcnc.command()
        self.stat = linuxcnc.stat()

        self.error_channel = linuxcnc.error_channel()
        # initial poll, so all is up to date
        self.stat.poll()
        self.error_channel.poll()

        # set INI path for INI info class before widgets are loaded
        INFO = Info(ini=argv[2])

        self.builder = Gtk.Builder()
        # translation of the glade file will be done with
        self.builder.set_translation_domain("gmoccapy")
        self.builder.add_from_file(XMLNAME)

        self.widgets = widgets.Widgets(self.builder)

        self.initialized = False  # will be set True after the window has been shown and all
                                  # basic settings has been finished, so we avoid some actions
                                  # because we cause click or toggle events when initializing
                                  # widget states.

        self.start_line = 0  # needed for start from line

        self.active_gcodes = []   # this are the formatted G-code values
        self.active_mcodes = []   # this are the formatted M-code values
        self.gcodes = []          # this are the unformatted G-code values to check if an update is required
        self.mcodes = []          # this are the unformatted M-code values to check if an update is required

        self.distance = 0         # This global will hold the jog distance
        self.tool_change = False  # this is needed to get back to manual mode after a tool change
        self.load_tool = False    # We use this avoid mode switching on reloading the tool on start up of the GUI
        self.macrobuttons = []    # The list of all macros defined in the INI file
        self.fo_counts = 0        # need to calculate difference in counts to change the feed override slider
        self.so_counts = 0        # need to calculate difference in counts to change the spindle override slider
        self.jv_counts = 0        # need to calculate difference in counts to change the jog_vel slider
        self.ro_counts = 0        # need to calculate difference in counts to change the rapid override slider

        self.spindle_override = 1 # holds the feed override value and is needed to be able to react to halui pin
        self.feed_override = 1    # holds the spindle override value and is needed to be able to react to halui pin
        self.rapidrate = 1        # holds the rapid override value and is needed to be able to react to halui pin

        self.incr_rbt_list = []   # we use this list to add hal pin to the button later
        self.jog_increments = []  # This holds the increment values
        self.unlock = False       # this value will be set using the hal pin unlock settings

        # needed to display the labels
        self.system_list = ("0", "G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3")
        self.dro_size = 28           # The size of the DRO, user may want them bigger on bigger screen
        self.axisnumber_four = ""    # we use this to get the number of the 4-th axis
        self.axisletter_four = None  # we use this to get the letter of the 4-th axis
        self.axisnumber_five = ""    # we use this to get the number of the 5-th axis
        self.axisletter_five = None  # we use this to get the letter of the 5-th axis

        self.notification = notification.Notification()  # Our own message system
        self.notification.connect("message_deleted", self._on_message_deleted)
        self.last_key_event = None, 0  # needed to avoid the auto repeat function of the keyboard
        self.all_homed = False         # will hold True if all axis are homed
        self.faktor = 1.0              # needed to calculate velocities

        self.xpos = 40        # The X Position of the main Window
        self.ypos = 30        # The Y Position of the main Window
        self.width = 980      # The width of the main Window
        self.height = 750     # The height of the main Window

        self.gcodeerror = ""   # we need this to avoid multiple messages of the same error

        self.file_changed = False
        self.widgets.hal_action_saveas.connect("saved-as", self.saved_as)

        self.lathe_mode = None # we need this to check if we have a lathe config
        self.backtool_lathe = False
        self.diameter_mode = False

        # the default theme = System Theme we store here to be able to go back to that one later
        self.default_theme = Gtk.Settings.get_default().get_property("gtk-theme-name")
        self.icon_theme = Gtk.IconTheme()
        self.icon_theme.append_search_path(ICON_THEME_DIR)
        self.icon_theme.append_search_path(USER_ICON_THEME_DIR)

        self.dialogs = dialogs.Dialogs()
        self.dialogs.connect("play_sound", self._on_play_sound)

        # check the arguments given from the command line (Ini file)
        self.user_mode = False
        self.logofile = None
        for index, arg in enumerate(argv):
            LOG.debug("{0} = {1}".format(index, arg))
            if arg == "-user_mode":
                self.user_mode = True
                self.widgets.tbtn_setup.set_sensitive(False)
                LOG.debug(_("user mode selected"))
            if arg == "-logo":
                self.logofile = str(argv[ index + 1 ])
                LOG.debug(_("logo entry found = {0}").format(self.logofile))
                self.logofile = self.logofile.strip("\"\'")
                if not os.path.isfile(self.logofile):
                    self.logofile = None
                    message = _("Logofile entry found, but could not be converted to path.")
                    message += "\n" + _("The file path should not contain any spaces")
                    LOG.warning(message)

        # check if the user want a Logo (given as command line argument)
        if self.logofile:
            self.widgets.img_logo.set_from_file(self.logofile)
            self.widgets.img_logo.show()

            page2 = self.widgets.ntb_jog_JA.get_nth_page(2)
            self.widgets.ntb_jog_JA.reorder_child(page2, 0)
            page1 = self.widgets.ntb_jog_JA.get_nth_page(1)
            self.widgets.ntb_jog_JA.reorder_child(page1, -1)

        # Our own class to get information from INI the file we use this way, to be sure
        # to get a valid result, as the checks are done in that module
        self._get_ini_data()

        self._get_pref_data()

        self.tool_measure_OK = self._check_toolmeasurement()

        # make all widgets we create dynamically
        self._make_DRO()
        self._make_ref_axis_button()
        self._make_touch_button()
        self._make_jog_increments()
        self._make_jog_button()
        if not self.trivial_kinematics:
            # we need joint jogging button
            self._make_joints_button()
            self._arrange_joint_button()
        self._make_macro_button()

        # if we have a lathe, we need to rearrange some stuff
        # we will do that in a separate function
        if self.lathe_mode:
            self._make_lathe()
        else:
            self.widgets.rbt_view_y2.hide()
            # X Offset is not necessary on a mill
            self.widgets.lbl_tool_offset_x.hide()
            self.widgets.lbl_offset_x.hide()
            self.widgets.btn_tool_touchoff_x.hide()
            self.widgets.lbl_hide_tto_x.show()

        self._arrange_dro()
        self._arrange_jog_button()

        self._make_hal_pins()

        self._init_user_messages()

        # set the title of the window, to show the release
        self.widgets.window1.set_title("gmoccapy {} for LinuxCNC {}".format(_RELEASE, linuxcnc.version))
        self.widgets.lbl_version.set_label("<b>gmoccapy\n{0}</b>".format(_RELEASE))

        panel = gladevcp.makepins.GladePanel(self.halcomp, XMLNAME, self.builder, None)

        self.halcomp.ready()

        self.builder.connect_signals(self)

        # this are settings to be done before window show
        self._init_preferences()

        # finally show the window
        self.widgets.window1.show()

        self._init_dynamic_tabs()
        self._init_tooleditor()
        self._init_themes()
        self._init_icon_themes()
        self._init_sourceview_themes()
        self._init_audio()
        self._init_gremlin()
        self._init_kinematics_type()
        self._init_hide_cursor()
        self._init_hide_tooltips()
        self._init_offsetpage()
        self._init_keybindings()
        self._init_IconFileSelection()
        self._init_keyboard()

        # now we initialize the file to load widget
        self._init_file_to_load()

        self._show_offset_tab(False)
        self._show_tooledit_tab(False)
        self._show_iconview_tab(False)

        # those two containers are disabled by default to allow fullscreen 
        # with smaller display resolutions
        self.widgets["hbx_upper"].show()
        self.widgets["ntb_button"].show()

        # the velocity settings
        self.widgets.adj_spindle_bar_min.set_value(self.min_spindle_rev)
        self.widgets.adj_spindle_bar_max.set_value(self.max_spindle_rev)
        self.widgets.spindle_feedback_bar.set_property("min", float(self.min_spindle_rev))
        self.widgets.spindle_feedback_bar.set_property("max", float(self.max_spindle_rev))

        # Popup Messages position and size
        self.widgets.adj_x_pos_popup.set_value(self.prefs.getpref("x_pos_popup", 45, float))
        self.widgets.adj_y_pos_popup.set_value(self.prefs.getpref("y_pos_popup", 55, float))
        self.widgets.adj_width_popup.set_value(self.prefs.getpref("width_popup", 250, float))
        self.widgets.adj_max_messages.set_value(self.prefs.getpref("max_messages", 10, float))
        self.widgets.fontbutton_popup.set_font(self.prefs.getpref("message_font", "sans 10", str))
        self.widgets.chk_use_frames.set_active(self.prefs.getpref("use_frames", True, bool))


        # should the tool in spindle be reloaded on startup?
        self.widgets.chk_reload_tool.set_active(self.prefs.getpref("reload_tool", True, bool))

        # and the rest of the widgets
        self.widgets.rbt_manual.set_active(True)
        self.widgets.ntb_jog.set_current_page(0)

        opt_blocks = self.prefs.getpref("blockdel", False, bool)
        self.widgets.tbtn_optional_blocks.set_active(opt_blocks)
        self.command.set_block_delete(opt_blocks)

        #optional_stops = self.prefs.getpref( "opstop", False, bool )
        #self.widgets.tbtn_optional_stops.set_active( optional_stops )
        #self.command.set_optional_stop( optional_stops )

        self.widgets.chk_show_dro.set_active(self.prefs.getpref("enable_dro", False, bool))
        self.widgets.chk_show_offsets.set_active(self.prefs.getpref("show_offsets", False, bool))
        self.widgets.chk_show_dtg.set_active(self.prefs.getpref("show_dtg", False, bool))
        self.widgets.chk_show_offsets.set_sensitive(self.widgets.chk_show_dro.get_active())
        self.widgets.chk_show_dtg.set_sensitive(self.widgets.chk_show_dro.get_active())
        self.widgets.cmb_mouse_button_mode.set_active(self.prefs.getpref("mouse_btn_mode", 4, int))

        self.widgets.tbtn_view_tool_path.set_active(self.prefs.getpref("view_tool_path", True, bool))
        self.widgets.tbtn_view_dimension.set_active(self.prefs.getpref("view_dimension", True, bool))
        view = self.prefs.getpref("view", "p", str)
        self.widgets["rbt_view_{0}".format(view)].set_active(True)
        self.widgets.gremlin.set_property("view", view)

        GSTAT = hal_glib.GStat()
        GSTAT.connect("graphics-gcode-properties", self.on_gcode_properties)
        GSTAT.connect("file-loaded", self.on_hal_status_file_loaded)

        # get if run from line should be used
        rfl = self.prefs.getpref("run_from_line", "no_run", str)
        # and set the corresponding button active
        self.widgets["rbtn_{0}_from_line".format(rfl)].set_active(True)
        if rfl == "no_run":
            self.widgets.btn_from_line.set_sensitive(False)
        else:
            self.widgets.btn_from_line.set_sensitive(True)

        # get the way to unlock the setting
        unlock = self.prefs.getpref("unlock_way", "use", str)
        # and set the corresponding button active
        self.widgets["rbt_{0}_unlock".format(unlock)].set_active(True)
        # if Hal pin should be used, only set the button active, if the pin is high
        if unlock == "hal" and not self.halcomp["unlock-settings"]:
            self.widgets.tbtn_setup.set_sensitive(False)

        # check if the user want to display preview window instead of offsetpage widget
        state = self.prefs.getpref("show_preview_on_offset", False, bool)
        if state:
            self.widgets.rbtn_show_preview.set_active(True)
        else:
            self.widgets.rbtn_show_offsets.set_active(True)

        # check if keyboard shortcuts should be used and set the chkbox widget
        self.widgets.chk_use_kb_shortcuts.set_active(self.prefs.getpref("use_keyboard_shortcuts",
                                                                        False, bool))

        # check the highlighting type
        # the following would load the python language
        # self.widgets.gcode_view.set_language("python")
        LANGDIR = os.path.join(BASE, "share", "Gtksourceview-4", "language-specs")
        file_path = os.path.join(LANGDIR, "gcode.lang")
        if os.path.isfile(file_path):
            LOG.info("Gcode.lang found")
            self.widgets.gcode_view.set_language("gcode", LANGDIR)

        # set the user colors and digits of the DRO
        self.widgets.abs_colorbutton.set_rgba(self._get_RGBA_color(self.abs_color))
        self.widgets.rel_colorbutton.set_rgba(self._get_RGBA_color(self.rel_color))
        self.widgets.dtg_colorbutton.set_rgba(self._get_RGBA_color(self.dtg_color))
        self.widgets.homed_colorbtn.set_rgba(self._get_RGBA_color(self.homed_color))
        self.widgets.unhomed_colorbtn.set_rgba(self._get_RGBA_color(self.unhomed_color))

        self.widgets.adj_dro_digits.set_value(self.dro_digits)
        # the adjustment change signal will set the dro_digits correct, so no extra need here.

        self.widgets.chk_toggle_readout.set_active(self.toggle_readout)

        self.widgets.adj_start_spindle_RPM.set_value(self.spindle_start_rpm)
        self.widgets.gcode_view.set_editable(False)
        self.widgets.ntb_user_tabs.remove_page(0)

        # call the function to change the button status
        # so every thing is ready to start
        widgetlist = ["rbt_manual", "rbt_mdi", "rbt_auto", "btn_homing", "btn_touch", "btn_tool",
                      "ntb_jog", "ntb_jog_JA", "vbtb_jog_incr", "hbox_jog_vel",
                      "spc_feed", "btn_feed_100", "rbt_forward", "btn_index_tool",
                      "rbt_reverse", "rbt_stop", "tbtn_flood", "tbtn_mist", "btn_change_tool",
                      "btn_select_tool_by_no", "btn_spindle_100", "spc_rapid", "spc_spindle",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z"
        ]
        #
        self._sensitize_widgets(widgetlist, False)

        # if limit switch active, activate ignore-checkbox
        if any(self.stat.limit):
            self.widgets.ntb_jog.set_sensitive(True)

        # this must be done last, otherwise we will get wrong values
        # because the window is not fully realized
        self._init_notification()

        # since the main loop is needed to handle the UI and its events, blocking calls like sleep()
        # will block the UI as well, so everything goes through event handlers (aka callbacks)
        # The GLib.timeout_add() function sets a function to be called at regular intervals
        # the time between calls to the function, in milliseconds
        # CYCLE_TIME = time, in milliseconds, that display will sleep between polls
        cycle_time = self.get_ini_info.get_cycle_time()
        GLib.timeout_add( cycle_time, self._periodic )  # time between calls to the function, in milliseconds
        GLib.timeout_add(1000, self._periodic_1s)       # 1 s timer for elapsed time display
        self.elapsed_time_run = 0
        self.progress = 0

        self._startup_message()

        # This allows sourcing an user defined file
        rcfile = "~/.gmoccapyrc"
        user_command_file = self.get_ini_info.get_user_command_file()
        if user_command_file:
            rcfile = user_command_file
        rcfile = os.path.expanduser(rcfile)
        if os.path.exists(rcfile):
            try:
                exec(compile(open(rcfile, "rb").read(), rcfile, 'exec'))
            except:
                tb = traceback.format_exc()
                LOG.error(tb)
                self.notification.add_message(_("Error in") + " " + rcfile + "\n" \
                    + _("Please check the console output."), ALERT_ICON)

        # Custom css file, e.g.:
        #     button:checked {
        #         background: rgba(230,230,50,0.8);
        #     }

        css_file = "~/.gmoccapy_css"
        user_css_file = self.get_ini_info.get_user_css_file()
        if user_css_file:
            css_file = user_css_file
        css_file = os.path.expanduser(css_file)
        if os.path.exists(css_file):
            provider_custom = Gtk.CssProvider()
            try:
                provider_custom.load_from_path(css_file)
                style_context.add_provider_for_screen(screen, provider_custom, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
            except:
                tb = traceback.format_exc()
                LOG.error(tb)
                self.notification.add_message(_("Error in") + " " + css_file + "\n" \
                    + _("Please check the console output."), ALERT_ICON)

    def _startup_message(self):
        title = _("Important change(s)")
        # This array holds information about new features or things that changed. They can be hidden using the dialog's checkbox.
        # It is important that strings are only appended to this array (not removed), otherwise some messages could get hidden unwanted.
        messages =[ _("<b>3.5.0 (LinuxCNC 2.10.0): Gmoccapy does no longer automatically retain G43 after a toolchange!</b>\n"\
                    "Automatic reactivation of G43 is possible using a REMAP.\n"\
                    "Examples can be found in the Gmoccapy sim configurations."),
                    # _("<b>3.5.1 (LinuxCNC 2.10.1): </b> Example for new feature"),
                    ]
        hide_message = self.prefs.getpref("hide_startup_messsage", 0, int)
        if hide_message < len(messages):
            message = "\n\n".join(messages[hide_message:])
            message += _('\n\nFor more information see the <a href="https://linuxcnc.org/docs/html/gui/gmoccapy_release_notes.txt">release notes</a>.')

            dont_show = self.dialogs.show_user_message(self, message, title, checkbox = True)
            if dont_show:
                self.prefs.putpref("hide_startup_messsage", len(messages), str)

    def _get_ini_data(self):
        self.get_ini_info = getiniinfo.GetIniInfo()
        # get the axis list from INI
        self.axis_list = self.get_ini_info.get_axis_list()
        # get the joint axis relation from INI
        self.joint_axis_dic, self.double_axis_letter = self.get_ini_info.get_joint_axis_relation()
        # if it's a lathe config, set the tool editor style
        self.lathe_mode = self.get_ini_info.get_lathe()
        if self.lathe_mode:
            # we do need to know also if we have a backtool lathe
            self.backtool_lathe = self.get_ini_info.get_backtool_lathe()

        # check if the user want actual or commanded for the DRO
        self.dro_actual = self.get_ini_info.get_position_feedback_actual()
        # the given Jog Increments
        self.jog_increments = self.get_ini_info.get_increments()
        # check if NO_FORCE_HOMING is used in INI
        self.no_force_homing = self.get_ini_info.get_no_force_homing()
        # do we use a identity kinematics or do we have to distinguish
        # JOINT and Axis modes?
        self.trivial_kinematics = self.get_ini_info.get_trivial_kinematics()
        units = self.get_ini_info.get_machine_units()
        if units == "mm" or units == "cm":
            self.metric = True
        else:
            self.metric = False
        self.no_force_homing = self.get_ini_info.get_no_force_homing()

        # get the values for the sliders
        self.rabbit_jog = self.get_ini_info.get_jog_vel()
        self.jog_rate_max = self.get_ini_info.get_max_jog_vel()

        self.spindle_override_max = self.get_ini_info.get_max_spindle_override()
        self.spindle_override_min = self.get_ini_info.get_min_spindle_override()
        self.feed_override_max = self.get_ini_info.get_max_feed_override()
        self.dro_actual = self.get_ini_info.get_position_feedback_actual()

    def _get_pref_data(self):
        self.prefs = preferences.preferences(self.get_ini_info.get_preference_file_path())

        # the size and digits of the DRO
        # set default values according to the machine units
        digits = 3
        if self.stat.linear_units != _MM:
            digits = 4
        self.dro_digits = self.prefs.getpref("dro_digits", digits, int)
        self.dro_size = self.prefs.getpref("dro_size", 28, int)

        # the colors of the DRO
        self.abs_color = self.prefs.getpref("abs_color", "#0000FF", str)         # blue
        self.rel_color = self.prefs.getpref("rel_color", "#000000", str)         # black
        self.dtg_color = self.prefs.getpref("dtg_color", "#FFFF00", str)         # yellow
        self.homed_color = self.prefs.getpref("homed_color", "#00FF00", str)     # green
        self.unhomed_color = self.prefs.getpref("unhomed_color", "#FF0000", str) # red

        # do we want gremlin dro ?
        self.enable_gremlin_dro = self.prefs.getpref("enable_dro", False, bool)

        # the scale to be applied to the counts of the hardware mpg wheel, to avoid to much turning
        self.scale_jog_vel = self.prefs.getpref("scale_jog_vel", self.jog_rate_max / 100, float)
        self.scale_spindle_override = self.prefs.getpref("scale_spindle_override", 1, float)
        self.scale_feed_override = self.prefs.getpref("scale_feed_override", 1, float)
        self.scale_rapid_override = self.prefs.getpref("scale_rapid_override", 1, float)

        # the velocity settings
        self.min_spindle_rev = self.prefs.getpref("spindle_bar_min", 0.0, float)
        self.max_spindle_rev = self.prefs.getpref("spindle_bar_max", 6000.0, float)

        self.turtle_jog_factor = self.prefs.getpref('turtle_jog_factor', 20, int)
        self.hide_turtle_jog_button = self.prefs.getpref("hide_turtle_jog_button", False, bool)

        self.unlock_code = self.prefs.getpref("unlock_code", "123", str)  # get unlock code

        self.toggle_readout = self.prefs.getpref("toggle_readout", True, bool)

        # if there is a INI Entry for default spindle speed, we will use that one as default
        # but if there is a setting in our preference file, that one will beet the INI entry
        default_spindle_speed = self.get_ini_info.get_default_spindle_speed()
        self.spindle_start_rpm = self.prefs.getpref( 'spindle_start_rpm', default_spindle_speed, float )

        self.kbd_height = self.prefs.getpref("kbd_height", 250, int)
        self.kbd_width = self.prefs.getpref("kbd_width", 880, int)
        self.kbd_set_height = self.prefs.getpref("kbd_set_height", False, bool)
        self.kbd_set_width = self.prefs.getpref("kbd_set_width", False, bool)

        # Activate the recently open tab page
        self.widgets.ntb_tool_and_code_info.set_current_page(self.prefs.getpref("info_tab_page", 0, int))

###############################################################################
##                     create widgets dynamically                            ##
###############################################################################

    def _make_DRO(self):
        LOG.debug("Entering make_DRO")
        LOG.debug("axis_list = {0}".format(self.axis_list))

        # we build one DRO for each axis
        self.dro_dic = {}
        for pos, axis in enumerate(self.axis_list):
            joint = self._get_joint_from_joint_axis_dic(axis)
            dro = Combi_DRO()
            dro.set_joint_no(joint)
            dro.set_axis(axis)
            dro.change_axisletter(axis.upper())
            dro.show()
            dro.set_property("name", "Combi_DRO_{0}".format(pos))
            dro.set_property("abs_color", self._get_RGBA_color(self.abs_color))
            dro.set_property("rel_color", self._get_RGBA_color(self.rel_color))
            dro.set_property("dtg_color", self._get_RGBA_color(self.dtg_color))
            dro.set_property("homed_color", self._get_RGBA_color(self.homed_color))
            dro.set_property("unhomed_color", self._get_RGBA_color(self.unhomed_color))
            dro.set_property("actual", self.dro_actual)
            dro.connect("clicked", self._on_DRO_clicked)
            dro.connect('axis_clicked', self._on_DRO_axis_clicked)
            self.dro_dic[dro.get_property("name")] = dro
#            print dro.name

    def _get_RGBA_color(self, color_str):
        color = Gdk.RGBA()
        color.parse(color_str)
        return Gdk.RGBA(color.red, color.green, color.blue, color.alpha)


    def _get_joint_from_joint_axis_dic(self, value):
        # if the selected axis is a double axis we will get the joint from the
        # master axis, which should end with 0
        if value in self.double_axis_letter:
            value = value + "0"
        return list(self.joint_axis_dic.keys())[list(self.joint_axis_dic.values()).index(value)]

    def _make_ref_axis_button(self):
        LOG.debug("Entering make ref axis button")

        # check if we need axis or joint homing button
        if self.trivial_kinematics:
            # lets find out, how many axis we got
            dic = self.axis_list
            name_prefix = "axis"
            name_prefix_sg = _("axis")
            name_prefix_pl = _("axes")
        else:
            # lets find out, how many joints we got
            dic = self.joint_axis_dic
            name_prefix = "joint"
            name_prefix_sg = _("joint")
            name_prefix_pl = _("joints")
        num_elements = len(dic)

        # as long as the number of axis is less 6 we can use the standard layout
        # we can display 6 axis without the second space label
        # and 7 axis if we do not display the first space label either
        # if we have more than 7 axis, we need arrows to switch the visible ones
        if num_elements < 7:
            lbl = self._get_space_label("lbl_space_0")
            self.widgets.hbtb_ref.pack_start(lbl,True,True,0)

        btn = self._new_button_with_predefined_image(
            name="ref_all",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_ref_all
        )
        btn.set_property("tooltip-text", _("Press to home all {0}").format(name_prefix_pl))
        btn.connect("clicked", self._on_btn_home_clicked)
        # we use pack_start, so the widgets will be moved from right to left
        # and are displayed the way we want
        self.widgets.hbtb_ref.pack_start(btn,True,True,0)

        if num_elements > 7:
            # show the previous arrow to switch visible homing button)
            btn = self._new_button_with_predefined_image(
                name="previous_button",
                size=_DEFAULT_BB_SIZE,
                image=self.widgets.img_ref_paginate_prev
            )
            btn.set_property("tooltip-text", _("Press to display previous homing button"))
            btn.connect("clicked", self._on_btn_previous_clicked)
            self.widgets.hbtb_ref.pack_start(btn,True,True,0)
            btn.hide()

        # do not use this label, to allow one more axis
        if num_elements < 6:
            lbl = self._get_space_label("lbl_space_2")
            self.widgets.hbtb_ref.pack_start(lbl,True,True,0)

        for pos, elem in enumerate(dic):
            btn = self._new_button_with_predefined_image(
                name=f"home_{name_prefix}_{elem}",
                size=_DEFAULT_BB_SIZE,
                image_name=f"img_ref_{elem}"
            )
            btn.set_property("tooltip-text", _("Press to home {0} {1}").format(name_prefix_sg, str(elem).upper()))
            btn.connect("clicked", self._on_btn_home_clicked)

            self.widgets.hbtb_ref.pack_start(btn,True,True,0)

            # if we have more than 7 axis we need to hide some button
            if num_elements > 7:
                if pos > 5:
                    btn.hide()

        if num_elements > 7:
            # show the next arrow to switch visible homing button)
            btn = self._new_button_with_predefined_image(
                name="next_button",
                size=_DEFAULT_BB_SIZE,
                image=self.widgets.img_ref_paginate_next
            )
            btn.set_property("tooltip-text", _("Press to display next homing button"))
            btn.connect("clicked", self._on_btn_next_clicked)
            self.widgets.hbtb_ref.pack_start(btn,True,True,0)

        # if there is space left, fill it with space labels
        start = self.widgets.hbtb_ref.child_get_property(btn,"position")
        for count in range(start + 1 , 8):
            lbl = self._get_space_label("lbl_space_{0}".format(count))
            self.widgets.hbtb_ref.pack_start(lbl,True,True,0)

        btn = self._new_button_with_predefined_image(
            name="unref_all",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_unref_all
        )
        btn.set_property("tooltip-text", _("Press to unhome all {0}").format(name_prefix_pl))
        btn.connect("clicked", self._on_btn_unhome_clicked)
        self.widgets.hbtb_ref.pack_start(btn,True,True,0)

        btn = self._new_button_with_predefined_image(
            name="home_back",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_ref_menu_close
        )
        btn.set_property("tooltip-text", _("Press to return to main button list"))
        btn.connect("clicked", self._on_btn_home_back_clicked)
        self.widgets.hbtb_ref.pack_start(btn,True,True,0)

        self.ref_button_dic = {}
        children = self.widgets.hbtb_ref.get_children()
        for child in children:
            self.ref_button_dic[child.get_property("name")] = child

    def _get_space_label(self, name):
        lbl = Gtk.Label.new("")
        lbl.set_property("name", name)
        lbl.set_size_request(*_DEFAULT_BB_SIZE)
        lbl.show()
        return lbl

    def _new_button_with_predefined_image(self, name, size, image = None, image_name = None):
        btn = Gtk.Button()
        btn.set_size_request(*size)
        btn.set_halign(Gtk.Align.CENTER)
        btn.set_valign(Gtk.Align.CENTER)
        btn.set_property("name", name)
        try:
            if image:
                btn.set_image(image)
            elif image_name:
                btn.set_image(self.widgets[image_name])
            else:
                raise ValueError("Either image or image_name must not be None")
        except Exception as e:
            LOG.warning(f"Error creating button with predefined image: {e}")
            missing_image = Gtk.Image()
            # TODO: Deprecated
            missing_image.set_from_stock(Gtk.STOCK_MISSING_IMAGE, Gtk.IconSize.BUTTON)
            btn.set_image(missing_image)
        btn.show_all()
        return btn

    def _get_button_with_image(self, name, filepath, icon_name):
        LOG.debug("get button with image")
        image = Gtk.Image()
        image.set_size_request(72,48)
        btn = Gtk.Button.new()
        btn.set_size_request(*_DEFAULT_BB_SIZE)
        btn.set_halign(Gtk.Align.CENTER)
        btn.set_valign(Gtk.Align.CENTER)
        btn.set_property("name", name)
        try:
            if filepath:
                pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_size(filepath, 48, 48)
                image.set_from_pixbuf(pixbuf)
            else:
                image.set_from_icon_name(icon_name, Gtk.IconSize.DIALOG)
            btn.add(image)
        except Exception as e:
            LOG.error(e)
            message = _("could not resolve the image path '{0}' given for button '{1}'".format(filepath, name))
            LOG.error(message)
            image.set_from_icon_name("image-missing", Gtk.IconSize.DIALOG)
            btn.add(image)

        btn.show_all()
        return btn

    def _remove_button(self, dic, box):
        for child in dic:
            box.remove(dic[child])

    def _on_btn_next_clicked(self, widget):
        # remove all buttons from container
        self._remove_button(self.ref_button_dic, self.widgets.hbtb_ref)

        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["ref_all"], True, True, 0)
        self.ref_button_dic["ref_all"].show()
        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["previous_button"], True, True, 0)
        self.ref_button_dic["previous_button"].show()

        start = len(self.axis_list) - 6
        end = len(self.axis_list)

        # now put the needed widgets in the container
        for axis in self.axis_list[start : end]:
            name = "home_axis_{0}".format(axis.lower())
            self.ref_button_dic[name].show()
            self.widgets.hbtb_ref.pack_start(self.ref_button_dic[name], True, True, 0)

        self._put_unref_and_back()

    def _on_btn_next_touch_clicked(self, widget):
        self._remove_button(self.touch_button_dic, self.widgets.hbtb_touch_off)

        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["edit_offsets"],True,True,0)
        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["previous_button"],True,True,0)
        self.touch_button_dic["previous_button"].show()

        start = len(self.axis_list) - 5
        end = len(self.axis_list)

        # now put the needed widgets in the container
        for axis in self.axis_list[start : end]:
            name = "touch_{0}".format(axis.lower())
            self.touch_button_dic[name].show()
            self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic[name], True, True, 0)

        self._put_set_active_and_back()

    def _on_btn_next_macro_clicked(self, widget):
        # remove all buttons from container
        self._remove_button(self.macro_dic, self.widgets.hbtb_MDI)

        self.widgets.hbtb_MDI.pack_start(self.macro_dic["previous_button"],True,True,0)
        self.macro_dic["previous_button"].show()

        end = len(self.macro_dic) - 3 # reduced by next, previous and keyboard
        start = end - 8

        # now put the needed widgets in the container
        for pos in range(start, end):
            name = "macro_{0}".format(pos)
            self.widgets.hbtb_MDI.pack_start(self.macro_dic[name], True, True, 0)
            self.macro_dic[name].show()

        self.widgets.hbtb_MDI.pack_start(self.macro_dic["keyboard"],True,True,0)
        self.macro_dic["keyboard"].show()

    def _on_btn_previous_clicked(self, widget):
        LOG.debug("previous")
        self._remove_button(self.ref_button_dic, self.widgets.hbtb_ref)

        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["ref_all"], True, True, 0)
        self.ref_button_dic["ref_all"].show()

        start = 0
        end = 6

        # now put the needed widgets in the container
        for axis in self.axis_list[start : end]:
            name = "home_axis_{0}".format(axis.lower())
            self.ref_button_dic[name].show()
            self.widgets.hbtb_ref.pack_start(self.ref_button_dic[name], True, True, 0)

        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["next_button"], True, True, 0)
        self.ref_button_dic["next_button"].show()

        self._put_unref_and_back()

    def _on_btn_previous_touch_clicked(self, widget):
        self._remove_button(self.touch_button_dic, self.widgets.hbtb_touch_off)

        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["edit_offsets"],True,True,0)

        if self.tool_measure_OK:
            end = 4
        else:
            end = 5

        start = 0
        # now put the needed widgets in the container
        for axis in self.axis_list[start : end]:
            name = "touch_{0}".format(axis.lower())
            self.touch_button_dic[name].show()
            self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic[name], True, True, 0)

        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["next_button"],True,True,0)
        self.touch_button_dic["next_button"].show()

        if self.tool_measure_OK:
            self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["block_height"],True,True,0)

        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["zero_offsets"],True,True,0)
        self._put_set_active_and_back()

    def _on_btn_previous_macro_clicked(self, widget):
        # remove all buttons from container
        self._remove_button(self.macro_dic, self.widgets.hbtb_MDI)

        start = 0
        end = 8

        # now put the needed widgets in the container
        for pos in range(start, end):
            name = "macro_{0}".format(pos)
            self.widgets.hbtb_MDI.pack_start(self.macro_dic[name], True, True, 0)
            self.macro_dic[name].show()

        self.widgets.hbtb_MDI.pack_start(self.macro_dic["next_button"],True,True,0)
        self.macro_dic["next_button"].show()

        self.widgets.hbtb_MDI.pack_start(self.macro_dic["keyboard"],True,True,0)
        self.macro_dic["keyboard"].show()

    def _put_set_active_and_back(self):
        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["zero_offsets"], True, True, 0)
        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["set_active"], True, True, 0)
        self.widgets.hbtb_touch_off.pack_start(self.touch_button_dic["touch_back"], True, True, 0)

    def _put_unref_and_back(self):
        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["unref_all"], True, True, 0)
        self.widgets.hbtb_ref.pack_start(self.ref_button_dic["home_back"], True, True, 0)

    def _make_touch_button(self):
        LOG.debug("Entering make touch button")

        dic = self.axis_list
        num_elements = len(dic)
        end = 7

        if self.tool_measure_OK:
            # we will have 3 buttons on the right side
            end -= 1

        lbl = Gtk.Label.new(_("edit\noffsets"))
        lbl.set_visible(True)
        lbl.set_justify(Gtk.Justification.CENTER)
        btn = Gtk.ToggleButton.new()
        btn.set_size_request(*_DEFAULT_BB_SIZE)
        btn.set_halign(Gtk.Align.CENTER)
        btn.set_valign(Gtk.Align.CENTER)
        btn.add(lbl)
        btn.connect("toggled", self.on_tbtn_edit_offsets_toggled)
        btn.set_property("tooltip-text", _("Press to edit the offsets"))
        btn.set_property("name", "edit_offsets")
        # TODO: Use CSS for this if still needed
        # btn.override_background_color(Gtk.StateFlags.ACTIVE, Gdk.RGBA(1.0, 1.0, 0.0, 1.0))
        self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
        btn.show()

        if num_elements > 6:
            # show the previous arrow to switch visible touch button)
            btn = self._new_button_with_predefined_image(
                name="previous_button",
                size=_DEFAULT_BB_SIZE,
                image=self.widgets.img_touch_paginate_prev
            )
            btn.set_property("tooltip-text", _("Press to display previous homing button"))
            btn.connect("clicked", self._on_btn_previous_touch_clicked)
            self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
            end -= 1
            btn.hide()

        for pos, axis in enumerate(dic):
            btn = self._new_button_with_predefined_image(
                name=f"touch_{axis}",
                size=_DEFAULT_BB_SIZE,
                image_name=f"img_touch_{axis}"
            )
            btn.set_property("tooltip-text", _("Press to set touch off value for axis {0}").format(axis.upper()))
            btn.connect("clicked", self._on_btn_set_value_clicked)

            #print("Touch button Name = ",name)

            self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)

            if pos > end - 2:
                btn.hide()

        if num_elements > (end - 1):
            # show the next arrow to switch visible homing button)
            btn = self._new_button_with_predefined_image(
                name="next_button",
                size=_DEFAULT_BB_SIZE,
                image=self.widgets.img_touch_paginate_next
            )
            btn.set_property("tooltip-text", _("Press to display next homing button"))
            btn.connect("clicked", self._on_btn_next_touch_clicked)
            self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
            btn.show()
            end -= 1

        # if there is space left, fill it with space labels
        start = self.widgets.hbtb_touch_off.child_get_property(btn,"position")
        for count in range(start + 1 , end):
            lbl = self._get_space_label("lbl_space_{0}".format(count))
            self.widgets.hbtb_touch_off.pack_start(lbl,True,True,0)
            lbl.show()

        btn = self.widgets.offsetpage1.wTree.get_object("zero_g92_button")
        btn.set_size_request(*_DEFAULT_BB_SIZE)
        btn.set_halign(Gtk.Align.CENTER)
        btn.set_valign(Gtk.Align.CENTER)
        self.widgets.offsetpage1.buttonbox.remove(btn)
        btn.connect("clicked", self.on_btn_zero_g92_clicked)
        btn.set_property("tooltip-text", _("Press to reset all G92 offsets"))
        btn.set_property("name", "zero_offsets")
        self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
        btn.show()

        if self.tool_measure_OK:
            btn = Gtk.Button.new_with_label(_(" Block\nHeight"))
            btn.set_size_request(*_DEFAULT_BB_SIZE)
            btn.set_halign(Gtk.Align.CENTER)
            btn.set_valign(Gtk.Align.CENTER)
            btn.connect("clicked", self.on_btn_block_height_clicked)
            btn.set_property("tooltip-text", _("Press to enter new value for block height"))
            btn.set_property("name", "block_height")
            self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
            btn.show()

        lbl = Gtk.Label.new(_("set\nselected"))
        lbl.set_visible(True)
        lbl.set_justify(Gtk.Justification.CENTER)
        btn = Gtk.Button.new()
        btn.set_size_request(*_DEFAULT_BB_SIZE)
        btn.set_halign(Gtk.Align.CENTER)
        btn.set_valign(Gtk.Align.CENTER)
        btn.add(lbl)
        btn.connect("clicked", self._on_btn_set_selected_clicked)
        btn.set_property("tooltip-text", _("Press to set the selected coordinate system to be the active one"))
        btn.set_property("name", "set_active")
        self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
        btn.show()

        btn = self._new_button_with_predefined_image(
            name="touch_back",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_touch_menu_close
        )
        btn.set_property("tooltip-text", _("Press to return to main button list"))
        btn.connect("clicked", self._on_btn_home_back_clicked)
        self.widgets.hbtb_touch_off.pack_start(btn,True,True,0)
        btn.show()

        self.touch_button_dic = {}
        children = self.widgets.hbtb_touch_off.get_children()
        for child in children:
            self.touch_button_dic[child.get_property("name")] = child

    def _check_toolmeasurement(self):
        # tool measurement probe settings
        xpos, ypos, zpos, maxprobe = self.get_ini_info.get_tool_sensor_data()
        if not xpos or not ypos or not zpos or not maxprobe:
            self.widgets.lbl_tool_measurement.show()
            LOG.info(_("No valid probe config in INI file. Tool measurement disabled."))
            self.widgets.chk_use_tool_measurement.set_active(False)
            self.widgets.chk_use_tool_measurement.set_sensitive(False)
            return False
        else:
            self.widgets.lbl_tool_measurement.hide()
            self.widgets.spbtn_probe_height.set_value(self.prefs.getpref("probeheight", -1.0, float))
            self.widgets.spbtn_search_vel.set_value(self.prefs.getpref("searchvel", 75.0, float))
            self.widgets.spbtn_probe_vel.set_value(self.prefs.getpref("probevel", 10.0, float))
            self.widgets.chk_use_tool_measurement.set_active(self.prefs.getpref("use_toolmeasurement", False, bool))
            self.widgets.lbl_x_probe.set_label(str(xpos))
            self.widgets.lbl_y_probe.set_label(str(ypos))
            self.widgets.lbl_z_probe.set_label(str(zpos))
            self.widgets.lbl_maxprobe.set_label(str(maxprobe))
            LOG.info(_("Found valid probe config in INI file. Will use auto tool measurement."))
            return True

    def _make_jog_increments(self):
        LOG.debug("Entering make jog increments")
        # Now we will build the option buttons to select the Jog-rates
        # We do this dynamically, because users are able to set them in INI File
        # because of space on the screen only 10 items are allowed
        # jogging increments

        self.incr_rbt_dic = {}

        # We get the increments from INI File
        if len(self.jog_increments) > 10:
            LOG.warning(_("To many increments given in INI file for this screen. "
            "Only the first 10 will be reachable through this screen."))
            # we shorten the increment list to 10 (first is default = 0)
            self.jog_increments = self.jog_increments[0:11]

        # The first radio button is created to get a radio button group
        # The group is called according the name off  the first button
        # We use the pressed signal, not the toggled, otherwise two signals will be emitted
        # One from the released button and one from the pressed button
        # we make a list of the buttons to later add the hardware pins to them
        label = _("Continuous")
        rbt = Gtk.RadioButton(label = label)
        rbt.set_property("name","rbt_0")
        rbt.connect("pressed", self._jog_increment_changed)
        self.widgets.vbtb_jog_incr.pack_start(rbt, True, True, 0)
        rbt.set_property("draw_indicator", False)
        rbt.show()
        self.incr_rbt_dic[rbt.get_property("name")] = rbt
        # the rest of the buttons are now added to the group
        # self.no_increments is set while setting the hal pins with self._check_len_increments
        for item in range(1, len(self.jog_increments)):
            name = "rbt_{0}".format(item)
            rbt = Gtk.RadioButton.new_from_widget(self.incr_rbt_dic["rbt_0"])
            rbt.set_label(label = self.jog_increments[item])
            rbt.set_property("name",name)
            rbt.connect("pressed", self._jog_increment_changed)
            self.widgets.vbtb_jog_incr.pack_start(rbt, True, True, 0)
            rbt.set_property("draw_indicator", False)
            rbt.show()
            self.incr_rbt_dic[rbt.get_property("name")] = rbt
        self.incr_rbt_dic["rbt_0"].set_active(True)
        self.active_increment = "rbt_0"
        # Set name to assign CSS
        self.widgets.vbtb_jog_incr.set_property("name","jog_incr_buttons")

    def _jog_increment_changed(self, widget,):
        # first cancel any joints jogging
        JOGMODE = self._get_jog_mode()
        if self.stat.task_mode == linuxcnc.MODE_MANUAL:
            for jnum in range(self.stat.joints):
                self.command.jog(linuxcnc.JOG_STOP, JOGMODE, jnum)
        self.distance = self._parse_increment(widget.get_property("name"))
        self.halcomp["jog.jog-increment"] = self.distance
        self.active_increment = widget.get_property("name")

    def _on_btn_jog_pressed(self, widget, button_name, shift=False):
        LOG.debug("Jog Button pressed = {0}".format(button_name))

        # only in manual mode we will allow jogging the axis at this development state
        # needed to avoid error on start up, machine not on
        if not self.stat.enabled or self.stat.task_mode != linuxcnc.MODE_MANUAL:
            return

        joint_no_or_axis_index = self._get_joint_no_or_axis_index(button_name)
        if joint_no_or_axis_index is None:
            LOG.debug("Did not get an axis number in jog part of the code")
            return

        # if shift = True, then the user pressed SHIFT for Jogging and
        # wants to jog at full speed
        # This can only happen on keyboard jogging, not with the on screen jog button
        # We just only use one function for both cases
        if shift:
            # There are no keyboard shortcuts to home angular axis, but
            # we implement the possibility for future options
            if button_name[0] in "abc":
                value = self.widgets.spc_ang_jog_vel.get_property("max") / 60
            else:
                value = self.jog_rate_max
        else:
            if button_name[0] in "abc":
                value = self.widgets.spc_ang_jog_vel.get_value() / 60
            else:
                value = self.widgets.spc_lin_jog_vel.get_value() / 60

        velocity = value * (1 / self.faktor)

        if button_name[1] == "+":
            dir = 1
        else:
            dir = -1

        JOGMODE = self._get_jog_mode()

        if self.distance != 0:  # incremental jogging
            distance = self.distance
            if self.lathe_mode and self.diameter_mode and button_name[0] == "x":
                distance = self.distance/2
            self.command.jog(linuxcnc.JOG_INCREMENT, JOGMODE, joint_no_or_axis_index, dir * velocity, distance)
        else:  # continuous jogging
            self.command.jog(linuxcnc.JOG_CONTINUOUS, JOGMODE, joint_no_or_axis_index, dir * velocity)

    def _on_btn_jog_released(self, widget, button_name, shift=False):
        LOG.debug("Jog Button released = {0}".format(button_name))
        # only in manual mode we will allow jogging the axis at this development state
        if not self.stat.enabled or self.stat.task_mode != linuxcnc.MODE_MANUAL:
            return

        joint_no_or_axis_index = self._get_joint_no_or_axis_index(button_name)

        JOGMODE = self._get_jog_mode()

        # Otherwise the movement would stop before the desired distance was moved
        if self.distance != 0:
            pass
        else:
            self.command.jog(linuxcnc.JOG_STOP, JOGMODE, joint_no_or_axis_index)

    def _get_jog_mode(self):
        # self.stat.motion_mode ==
        # 1 = Joint
        # 2 = MDI
        # 3 = TELOP
        if self.stat.motion_mode == 1:
                JOGMODE = 1
        else :
            JOGMODE = 0
        return JOGMODE

    def _get_joint_no_or_axis_index(self, button_name):
        joint_btn = False
        if not button_name[0] in "xyzabcuvw":
            LOG.debug("Axis button")
            # OK, it may be a Joints button
            if button_name[0] in "012345678":
                LOG.debug("joint button")
                joint_btn = True
            else:
                LOG.warning(_("unknown jog command {0}".format(button_name)))
                return None

        if not joint_btn:
            # get the axisnumber from the index as specified in python interface documentation
            if self.all_homed:
                joint_no_or_axis_index = "xyzabcuvw".index(button_name[0])
            else:
                joint_no_or_axis_index = self._get_joint_from_joint_axis_dic(button_name[0])
        else:
            joint_no_or_axis_index = int(button_name[0])

        return joint_no_or_axis_index

    def _make_jog_button(self):
        LOG.debug("Entering make jog button")

        self.jog_button_dic = OrderedDict()

        for axis in self.axis_list:
            for direction in ["+","-"]:
                name = "{0}{1}".format(str(axis), direction)
                btn = Gtk.Button.new_with_label(name.upper())
                btn.set_halign(Gtk.Align.CENTER)
                btn.set_valign(Gtk.Align.CENTER)
                btn.set_property("name", name)
                btn.connect("pressed", self._on_btn_jog_pressed, name)
                btn.connect("released", self._on_btn_jog_released, name)
                btn.set_property("tooltip-text", _("Press to jog axis {0}".format(axis)))
                # Set name to assign CSS
                self.widgets.vbx_jog_button.set_property("name", "jog_buttons")
                btn.set_size_request(48,48)

                self.jog_button_dic[name] = btn

    def _make_joints_button(self):
        LOG.debug("Entering make joints button")

        self.joints_button_dic = {}

        for joint in range(0, self.stat.joints):
            for direction in ["+","-"]:
                name = "{0}{1}".format(str(joint), direction)
                btn = Gtk.Button.new_with_label(name.upper())
                btn.set_halign(Gtk.Align.CENTER)
                btn.set_valign(Gtk.Align.CENTER)
                btn.set_property("name", name)
                btn.connect("pressed", self._on_btn_jog_pressed, name)
                btn.connect("released", self._on_btn_jog_released, name)
                btn.set_property("tooltip-text", _("Press to jog joint {0}".format(joint)))
                btn.set_size_request(48,48)

                self.joints_button_dic[name] = btn

    # check if macros are in the INI file and add them to MDI Button List
    def _make_macro_button(self):
        LOG.debug("Entering make macro button")

        macros = self.get_ini_info.get_macros()

        # if no macros at all are found, we receive a NONE, so we have to check:
        if not macros:
            num_macros = 0
            # no return here, otherwise we will not get filling labels
        else:
            num_macros = len(macros)

        LOG.debug("found {0} Macros".format(num_macros))

        if num_macros > 16:
            message = _("Found more than 16 macros, will use only the first 16.")
            LOG.info(message)

            num_macros = 16

        btn = self._new_button_with_predefined_image(
            name="previous_button",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_macro_paginate_prev
        )
        btn.hide()
        btn.set_property("tooltip-text", _("Press to display previous macro button"))
        btn.connect("clicked", self._on_btn_previous_macro_clicked)
        self.widgets.hbtb_MDI.pack_start(btn,True,True,0)

        for pos in range(0, num_macros):
            name = macros[pos]

            image = self._check_macro_for_image(name)
            if image:
                LOG.debug("Macro {0} has image link".format(name))
                LOG.debug("Image = {0}".format(image))
                btn = self._get_button_with_image("macro_{0}".format(pos), image, None)
            else:
                lbl = name.split()[0]
                # shorten / break line of the name if it is to long
                if len(lbl) > 11:
                    lbl = lbl[0:10] + "\n" + lbl[11:20]
                btn = Gtk.Button.new_with_label(lbl)
                btn.set_size_request(*_DEFAULT_BB_SIZE)
                btn.set_halign(Gtk.Align.CENTER)
                btn.set_valign(Gtk.Align.CENTER)
                btn.set_property("name","macro_{0}".format(pos))
            btn.set_property("tooltip-text", _("Press to run macro {0}".format(name)))
            btn.connect("clicked", self._on_btn_macro_pressed, name)
            btn.position = pos
            btn.show()
            self.widgets.hbtb_MDI.pack_start(btn, True, True, 0)

        btn = self._new_button_with_predefined_image(
            name="next_button",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_macro_paginate_next
        )
        btn.set_property("tooltip-text", _("Press to display next macro button"))
        btn.connect("clicked", self._on_btn_next_macro_clicked)
        btn.hide()
        self.widgets.hbtb_MDI.pack_start(btn,True,True,0)

        # if there is still place, we fill it with empty labels, to be sure the button will not be on different
        # places if the amount of macros change.
        if num_macros < 9:
            for pos in range(num_macros, 9):
                lbl = Gtk.Label()
                lbl.set_property("name","lbl_space_{0}".format(pos))
                lbl.set_text("")
                self.widgets.hbtb_MDI.pack_start(lbl, True, True, 0)
                lbl.show()

        btn = self.widgets.btn_macro_menu_toggle_keyboard = self._new_button_with_predefined_image(
            name="keyboard",
            size=_DEFAULT_BB_SIZE,
            image=self.widgets.img_macro_menu_keyboard
        )
        btn.set_property("tooltip-text", _("Press to display the virtual keyboard"))
        btn.connect("clicked", self.on_btn_show_kbd_clicked)
        self.widgets.hbtb_MDI.pack_start(btn,True,True,0)

        self.macro_dic = {}

        children = self.widgets.hbtb_MDI.get_children()
        for child in children:
            self.macro_dic[child.get_property("name")] = child

        if num_macros >= 9:
            self.macro_dic["next_button"].show()
            for pos in range(8, num_macros):
                self.macro_dic["macro_{0}".format(pos)].hide()

    def _check_macro_for_image(self, name):
        image = False
        for path in self.get_ini_info.get_subroutine_paths().split(":"):
            file = path + "/" + name.split()[0] + ".ngc"
            if os.path.isfile(file):
                macrofile = open(file, "r")
                lines = macrofile.readlines()
                macrofile.close()
                for line in lines:
                    if line[0] == ";":
                        continue
                    if "image" in line.lower():
                        image = True
                        break

        # should be like that in ngc file
        # (IMAGE, /home/my_home/my_image_dir/my_image.png)
        # so we need to get the correct image path
        if image:
            image = line.split(",")[1]
            image = image.strip()
            image = image.replace(")","")
            if "~" in image:
                image = image.replace("~", os.path.expanduser("~"))
            image = os.path.abspath(image)

        return image

    # if this is a lathe we need to rearrange some button and add a additional DRO
    def _make_lathe(self):
        LOG.debug("we have a lathe here")

        # if we have a lathe, we will need an additional DRO to display
        # diameter and radius simultaneous, we will call that one Combi_DRO_9, as that value
        # should never be used due to the limit in axis from 0 to 8
        dro = Combi_DRO()
        dro.set_property("name", "Combi_DRO_9")
        dro.set_property("abs_color", self._get_RGBA_color(self.abs_color))
        dro.set_property("rel_color", self._get_RGBA_color(self.rel_color))
        dro.set_property("dtg_color", self._get_RGBA_color(self.dtg_color))
        dro.set_property("homed_color", self._get_RGBA_color(self.homed_color))
        dro.set_property("unhomed_color", self._get_RGBA_color(self.unhomed_color))
        dro.set_property("actual", self.dro_actual)

        joint = self._get_joint_from_joint_axis_dic("x")
        dro.set_joint_no(joint)
        dro.set_axis("x")
        dro.change_axisletter("D")
        dro.set_property("diameter", True)
        dro.show()

        dro.connect("clicked", self._on_DRO_clicked)
        dro.connect('axis_clicked', self._on_DRO_axis_clicked)
        self.dro_dic[dro.get_property("name")] = dro

        self.dro_dic["Combi_DRO_0"].change_axisletter("R")

        # For a lathe we don"t need the following button
        self.widgets.rbt_view_p.hide()
        self.widgets.rbt_view_x.hide()
        self.widgets.rbt_view_z.hide()
        self.widgets.lbl_hide_tto_x.hide()
        self.widgets.lbl_blockheight.hide()

        # but we have to show this one
        self.widgets.rbt_view_y2.show()

        if self.backtool_lathe:
            view = "y2"
        else:
            view = "y"
        self.prefs.putpref("view", view)

        self.widgets["rbt_view_{0}".format(view)].set_active(True)
        self.widgets.gremlin.set_property("view", view)

        self._switch_to_g7(False)

        # we need to arrange the jog button,
        # a lathe should have at least X and Z axis
        if not "x" in self.axis_list or not "z" in self.axis_list:
            message = _("* GMOCCAPY ERROR *")
            message += _("this is not a lathe, as a lathe must have at least\n")
            message += _("an X and an Z axis\n")
            message += _("Wrong lathe configuration, we will leave here")
            self.dialogs.warning_dialog(self, _("Very critical situation"), message, sound = False)
            sys.exit()
        else:
            if not len(self.axis_list) == 2 and not len(self.axis_list) < 6:
                self._arrange_jog_button_by_axis()
                return
            count = 0
            for btn_name in self.jog_button_dic:
                if btn_name == "x+":
                    col = 1
                    row = 2
                    if self.backtool_lathe:
                        row = 0
                elif btn_name == "x-":
                    col = 1
                    row = 0
                    if self.backtool_lathe:
                        row = 2
                elif btn_name == "z+":
                    col = 2
                    row = 1
                elif btn_name == "z-":
                    col = 0
                    row = 1
                else:
                    if count < 2:
                        col = 3
                    elif count < 4:
                        col = 2
                    else:
                        col = 0
                    if "+" in btn_name:
                        row = 0
                    else:
                        row = 2
                    count +=1

                self.widgets.grid_jog_btn_axes.attach(self.jog_button_dic[btn_name], col, row, 1, 1)
                self.jog_button_dic[btn_name].show()

    def _arrange_dro(self, attach=True):
        def _place_special(attach):
            LOG.debug("Place DRO 3x2")
            dro_order = self._get_DRO_order()
            for dro, dro_name in enumerate(dro_order):
                if dro < 3:
                    size = self.dro_size * 0.75
                    if attach:
                        self.widgets.grid_DRO.attach(self.dro_dic[dro_name], 0, int(dro), 2, 1)
                else:
                    size = self.dro_size * 0.65
                    if attach:
                        if dro == 3:
                            self.widgets.grid_DRO.attach(self.dro_dic[dro_name], 0, int(dro), 1, 1)
                        else:
                            self.widgets.grid_DRO.attach(self.dro_dic[dro_name], 1, int(dro-1), 1, 1)
                self.dro_dic[dro_name].set_property("font_size", size)

        def _place_in_table(cols, dro_size, attach):
            LOG.debug("Place DRO in table")
            col = 0
            row = 0
            dro_order = self._get_DRO_order()
            for dro, dro_name in enumerate(dro_order):
                # as a lathe might not have all Axis, we check if the key is in directory
                if dro_name not in list(self.dro_dic.keys()):
                    continue
                self.dro_dic[dro_name].set_property("font_size", dro_size)
                if attach:
                    self.widgets.grid_DRO.attach(self.dro_dic[dro_name], col, row, 1, 1)
                if cols > 1:
                    # calculate if we have to place in the first or the second column
                    if (dro % 2 == 1):
                        col = 0
                        row +=1
                    else:
                        col += 1
                else:
                    row += 1
        LOG.debug("Arrange DRO, length = {}".format(len(self.dro_dic)))
        
        # if we have less than 4 axis, we can resize the table, as we have
        # enough space to display each one in it's own line
        if len(self.dro_dic) < 4:
            _place_in_table(1, self.dro_size, attach)

        # having 4 DRO we need to reduce the size, to fit the available space
        elif len(self.dro_dic) == 4:
            _place_in_table(1, self.dro_size * 0.75, attach)

        # having 5 axis we will display 3 in an one line and two DRO share
        # the last line, the size of the DRO must be reduced also
        # this is a special case so we do not use _place_in_table
        elif len(self.dro_dic) == 5:
            _place_special(attach)

        else:
            LOG.debug("DRO: more than 5 axes ")
            _place_in_table(2, self.dro_size * 0.65, attach)

        # set values to dro size adjustments
        self.widgets.adj_dro_size.set_value(self.dro_size)


    def _get_DRO_order(self):
        LOG.debug("get DRO order")
        dro_order = []
        # if Combi_DRO_9 exist we have a lathe with an additional DRO for diameter mode
        if "Combi_DRO_9" in list(self.dro_dic.keys()):
            for dro_name in ["Combi_DRO_0", "Combi_DRO_9", "Combi_DRO_1", "Combi_DRO_2", "Combi_DRO_3",
                             "Combi_DRO_4", "Combi_DRO_5", "Combi_DRO_6", "Combi_DRO_7", "Combi_DRO_8"]:
                if dro_name in list(self.dro_dic.keys()):
                    dro_order.append(dro_name)
        else:
            dro_order = sorted(self.dro_dic.keys())
        return dro_order

    def _arrange_jog_button(self):
        LOG.debug("arrange JOG button")

        # if we have a lathe, we have done the arrangement in _make_lathe
        # but if the lathe has more than 5 axis we will use standard
        if self.lathe_mode:
            return

        if len(self.axis_list) > 5:
            self._arrange_jog_button_by_axis()
            return

        if not "x" in self.axis_list or not "y" in self.axis_list or not "z" in self.axis_list:
            message = _("this is not a usual config\n")
            message += _("we miss one of X , Y or Z axis\n")
            message += _("We will use by axisletter ordered jog button")
            LOG.warning(message)
            self._arrange_jog_button_by_axis()
            return

        if len(self.axis_list) < 3:
            LOG.debug("Less than 3 axis")
        else:
            LOG.debug("less than 6 axis")

        count = 0
        for btn_name in self.jog_button_dic:
            if btn_name == "x+":
                col = 2
                row = 1
            elif btn_name == "x-":
                col = 0
                row = 1
            elif btn_name == "y+":
                col = 1
                row = 0
            elif btn_name =="y-":
                col = 1
                row = 2
            elif btn_name == "z+":
                col = 3
                row = 0
            elif btn_name == "z-":
                col = 3
                row = 2
            # order of the data in the dict matters for extra buttons.
            # This is why we use ordered dict for self.jog_button_dic
            else:
                if count < 2:
                    col = 2
                else:
                    col = 0
                if "+" in btn_name:
                    row = 0
                else:
                    row = 2
                count +=1
            self.widgets.grid_jog_btn_axes.attach(self.jog_button_dic[btn_name], col, row, 1, 1)
        self.widgets.grid_jog_btn_axes.show_all()

    def _arrange_jog_button_by_axis(self):
        LOG.debug("arrange JOG button by axis")
        LOG.debug(sorted(self.jog_button_dic.keys()))
        # check if amount of axis is an even number, adapt the needed lines
        cols = 4
        if len(self.dro_dic) % 2 == 0:
            rows = len(self.dro_dic) / 2
        else:
            rows = (len(self.dro_dic) + 1) / 2

        #print (cols,rows)

        col = 0
        row = 0

        for pos, btn in enumerate("xyzabcuvw"):
            btn_name = "{0}-".format(btn)
            if btn_name not in list(self.jog_button_dic.keys()):
                continue

            self.widgets.grid_jog_btn_axes.attach(self.jog_button_dic[btn_name],
                                        col, row, 1, 1)
            btn_name = "{0}+".format(btn)
            self.widgets.grid_jog_btn_axes.attach(self.jog_button_dic[btn_name],
                                        col+1, row, 1, 1)

            row +=1

            # calculate if we have to place in the first or the second column
            if row >= rows:
                col = 2
                row = 0
        self.widgets.grid_jog_btn_axes.show_all()

    def _arrange_joint_button(self):
        LOG.debug("arrange JOINTS button")
        LOG.debug("Found {0} Joints Button".format(len(self.joints_button_dic)))

        cols = 4
        if self.stat.joints % 2 == 0:
            rows = self.stat.joints / 2
        else:
            rows = (self.stat.joints + 1) / 2


        col = 0
        row = 0

        for joint in range(0, self.stat.joints):
            #print(joint)

            joint_name = "{0}-".format(joint)
            self.widgets.grid_jog_btn_joints.attach(self.joints_button_dic[joint_name],
                                    col, row, 1, 1)

            joint_name = "{0}+".format(joint)
            self.widgets.grid_jog_btn_joints.attach(self.joints_button_dic[joint_name],
                                    col+1, row, 1, 1)

            row +=1

            # calculate if we have to place in the first or the second column
            if row >= rows:
                col = 2
                row = 0

        self.widgets.grid_jog_btn_joints.show_all()

    def _init_preferences(self):
        # check if NO_FORCE_HOMING is used in INI
        # disable reload tool on start up, if True
        if self.no_force_homing:
            self.widgets.chk_reload_tool.set_sensitive(False)
            self.widgets.chk_reload_tool.set_active(False)
            self.widgets.lbl_reload_tool.set_visible(True)

        # set the slider limits
        self.widgets.spc_lin_jog_vel.set_property("min", 0)
        self.widgets.spc_lin_jog_vel.set_property("max", self.jog_rate_max)
        self.widgets.spc_lin_jog_vel.set_value(self.rabbit_jog)

        self.widgets.spc_spindle.set_property("min", self.spindle_override_min * 100)
        self.widgets.spc_spindle.set_property("max", self.spindle_override_max * 100)
        self.widgets.spc_spindle.set_value(100)

        self.widgets.spc_rapid.set_property("min", 0)
        self.widgets.spc_rapid.set_property("max", 100)
        self.widgets.spc_rapid.set_value(100)

        self.widgets.spc_feed.set_property("min", 0)
        self.widgets.spc_feed.set_property("max", self.feed_override_max * 100)
        self.widgets.spc_feed.set_value(100)

        # the scales to apply to the count of the hardware mpg wheel, to avoid to much turning
        self.widgets.adj_scale_jog_vel.set_value(self.scale_jog_vel)
        self.widgets.adj_scale_spindle_override.set_value(self.scale_spindle_override)
        self.widgets.adj_scale_feed_override.set_value(self.scale_feed_override)
        self.widgets.adj_scale_rapid_override.set_value(self.scale_rapid_override)

        # set and get all information for turtle jogging
        # self.rabbit_jog will be used in future to store the last value
        # so it can be recovered after jog_vel_mode switch
        self.widgets.chk_turtle_jog.set_active(self.hide_turtle_jog_button)
        self.widgets.adj_turtle_jog_factor.configure(self.turtle_jog_factor, 1,
                                                     100, 1, 0, 0)
        if self.hide_turtle_jog_button:
            self.widgets.tbtn_turtle_jog.hide()
            self.turtle_jog_factor = 1
        self.turtle_jog = self.rabbit_jog / self.turtle_jog_factor

        # and according to machine units the digits to display
        if self.stat.linear_units == _MM:
            self.widgets.spc_lin_jog_vel.set_digits(0)
            self.widgets.spc_lin_jog_vel.set_property("unit", _("mm/min"))
        else:
            self.widgets.spc_lin_jog_vel.set_digits(2)
            self.widgets.spc_lin_jog_vel.set_property("unit", _("inch/min"))

        # the size of the DRO ; self.dro_size is initialized in _get_pref_data
        self.widgets.adj_dro_size.set_value(self.dro_size)

        # hide the angular jog vel if no angular joint is used
        if not "a" in self.axis_list and not "b" in self.axis_list and not "c" in self.axis_list:
            self.widgets.spc_ang_jog_vel.hide()
        else:
            self.widgets.spc_ang_jog_vel.set_property("min", self.get_ini_info.get_min_ang_jog_vel())
            self.widgets.spc_ang_jog_vel.set_property("max", self.get_ini_info.get_max_ang_jog_vel())
            self.widgets.spc_ang_jog_vel.set_value(self.get_ini_info.get_default_ang_jog_vel())

# =============================================================
# Dynamic tabs handling Start

    def _init_dynamic_tabs(self):
        # dynamic tabs setup
        self._dynamic_childs = {}
        # register all tabs, so they will be closed together with the GUI
        atexit.register(self._kill_dynamic_childs)

        tab_names, tab_locations, tab_cmd = self.get_ini_info.get_embedded_tabs()
        if not tab_names:
            self.widgets.tbtn_user_tabs.set_sensitive( False )
            self.user_tabs_enabled = False
            LOG.info(_("No embedded tabs configured or invalid configuration. No tabs will be added!"))
            return

        try:
            for t, c, name in zip(tab_names, tab_cmd, tab_locations):
                nb = self.widgets[name]
                if c == "hide":
                    LOG.debug("hide widget: {0}, type: {1}".format(name, type(self.widgets[name])))
                    nb.hide()
                    continue
                xid = self._dynamic_tab(nb, t)
                if not xid: continue
                cmd = c.replace('{XID}', str(xid))
                child = subprocess.Popen(cmd.split())
                self._dynamic_childs[xid] = child
                nb.show_all()
        except:
            LOG.error(_("ERROR, trying to initialize the user tabs or panels, check for typos"))
            LOG.error(tab_locations)

        self.set_up_user_tab_widgets(tab_locations)

    # adds the embedded object to a notebook tab or box
    def _dynamic_tab(self, widget, text):
        s = Gtk.Socket()
        try:
            widget.append_page(s, Gtk.Label.new(" " + text + " "))
        except:
            try:
                widget.pack_end(s, True, True, 0)
            except:
                return None
        return s.get_id()

    # Gotta kill the embedded processes when gmoccapy closes
    #@atexit.register
    def _kill_dynamic_childs(self):
        LOG.debug("_kill_dynamic_childs")
        for child in list(self._dynamic_childs.values()):
            if type(child) == Gtk.Socket:
                if self.onboard:
                    self._kill_keyboard()
                    child.disconnect(child.get_id())
            else:
                child.terminate()

    def set_up_user_tab_widgets(self, tab_locations):
        if tab_locations:
            # make sure the user tabs button is disabled
            # if no ntb_user_tabs in location is given
            if "ntb_user_tabs" in tab_locations:
                self.widgets.tbtn_user_tabs.set_sensitive( True )
                self.user_tabs_enabled = True
            else:
                self.widgets.tbtn_user_tabs.set_sensitive( False )
                self.user_tabs_enabled = False

            if "ntb_preview" in tab_locations:
                self.widgets.ntb_preview.set_property( "show-tabs", True )

            # This is normally only used for the plasma screen layout
            if "box_coolant_and_spindle" in tab_locations:
                widgetlist = ["box_spindle", "box_cooling", "frm_spindle"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "box_cooling" in tab_locations:
                widgetlist = ["frm_cooling"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "box_spindle" in tab_locations:
                widgetlist = ["frm_spindle"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "box_vel_info" in tab_locations:
                widgetlist = ["vbx_overrides", "frm_rapid_override", "frm_feed_override"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "box_custom_1" in tab_locations:
                self.widgets.box_custom_1.show()

            if "box_custom_2" in tab_locations:
                self.widgets.box_custom_2.show()

            if "box_custom_3" in tab_locations:
                self.widgets.box_custom_3.show()

            if "box_custom_4" in tab_locations:
                self.widgets.box_custom_4.show()

            if "box_tool_and_code_info" in tab_locations:
                widgetlist = ["frm_tool_info", "frm_gcode", "active_speed_label", "lbl_speed"]
                for widget in widgetlist:
                    self.widgets[widget].hide()
                self.widgets.btn_tool.set_sensitive( False )

            if "box_code_info" in tab_locations:
                widgetlist = ["frm_gcode"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "box_tool_info" in tab_locations:
                widgetlist = ["frm_tool_info"]
                for widget in widgetlist:
                    self.widgets[widget].hide()

            if "hbox_jog" in tab_locations:
                widgetlist = ["vbx_jog_button", "vbtb_jog_incr"]
                for widget in widgetlist:
                    self.widgets[widget].hide()
                    
            if "box_dro_side" in tab_locations:
                self.widgets.box_dro_side.show()

# Dynamic tabs handling End
# =============================================================

    # and we load the tooltable data
    def _init_tooleditor(self):

       # get the path to the tool table
        tooltable = self.get_ini_info.get_toolfile()
        if not tooltable:
            message = _("Did not find a toolfile file in [EMCIO] TOOL_TABLE")
            LOG.error(message)
            self.dialogs.warning_dialog(self, _("Very critical situation"), message, sound = False)
            sys.exit()
        toolfile = os.path.join(CONFIGPATH, tooltable)
        self.widgets.tooledit1.set_filename(toolfile)

        # first we hide all the axis columns the unhide the ones we want
        self.widgets.tooledit1.set_visible("abcxyzuvwijq", False)
        for axis in self.axis_list:
            self.widgets.tooledit1.set_visible("{0}".format(axis), True)

        # if it's a lathe config we show lathe related columns
        if self.lathe_mode:
            self.widgets.tooledit1.set_visible("ijq", True)
            if not self.get_ini_info.get_lathe_wear_offsets():
                # hide the wear offset tabs
                self.widgets.tooledit1.set_lathe_display(False)

        self.widgets.tooledit1.hide_buttonbox(True)

    def _init_themes(self):
        # If there are themes then add them to combo box
        model = self.widgets.theme_choice.get_model()
        model.clear()
        model.append(("Follow System Theme",))
        themes = []
        if os.path.exists(USERTHEMEDIR):
            names = os.listdir(USERTHEMEDIR)
            names.sort()
            for dirs in names:
                try:
                    sbdirs = os.listdir(os.path.join(USERTHEMEDIR, dirs))
                    if 'gtk-3.0' in sbdirs:
                        themes.append(dirs)
                except:
                    pass
        if os.path.exists(THEMEDIR):
            names = os.listdir(THEMEDIR)
            names.sort()
            for dirs in names:
                try:
                    sbdirs = os.listdir(os.path.join(THEMEDIR, dirs))
                    if 'gtk-3.0' in sbdirs:
                        themes.append(dirs)
                except:
                    pass
        temp = 0
        theme_name = self.prefs.getpref("gtk_theme", "Follow System Theme", str)
        for index, theme in enumerate(themes):
            model.append((theme,))
            if theme == theme_name:
                temp = index + 1
        self.widgets.theme_choice.set_active(temp)
        settings = Gtk.Settings.get_default()
        if not theme_name == "Follow System Theme":
            settings.set_property("gtk-theme-name", theme_name)

    def _init_icon_themes(self):
        valid_icon_themes = icon_theme_helper.find_valid_icon_themes([USER_ICON_THEME_DIR, ICON_THEME_DIR])

        model = self.widgets.icon_theme_choice.get_model()
        model.clear()
        for icon_theme in valid_icon_themes:
            model.append(icon_theme)

        icon_theme_preference = self.prefs.getpref("icon_theme", DEFAULT_ICON_THEME, str)
        icon_theme_choice = self.widgets.icon_theme_choice
        with(icon_theme_choice.handler_block(icon_theme_helper.find_handler_id_by_signal(icon_theme_choice, "changed"))):
            try:
                selected_index = [icon_theme[1] for icon_theme in valid_icon_themes].index(icon_theme_preference)
                icon_theme_choice.set_active(selected_index)
            except ValueError:
                LOG.warning(f"Preferred icon-theme '{icon_theme_preference}' not found. Switching to first valid icon theme: '{valid_icon_themes[0][1]}'.")
                icon_theme_choice.set_active(0)

        # load icon theme
        self._set_icon_theme(model[icon_theme_choice.get_active_iter()][1])
        
    def _init_sourceview_themes(self):
        model = self.widgets.sourceview_theme_choice.get_model()
        model.clear()
        
        valid_themes = self.widgets["gcode_view"].get_style_schemes()
        for theme in valid_themes:
            model.append((theme,))

        theme_preference = self.prefs.getpref("gcode_theme", DEFAULT_GCODE_THEME, str)
        theme_choice = self.widgets.sourceview_theme_choice
        try:
            selected_index = valid_themes.index(theme_preference)
            theme_choice.set_active(selected_index)
        except ValueError:
            LOG.warning(f"Preferred G-code theme '{theme_preference}' not found. Switching to first valid G-code theme: '{valid_themes[0]}'.")
            theme_choice.set_active(0)

        # load theme
        self._set_sourceview_theme(model[theme_choice.get_active_iter()][0])


    def _init_audio(self):
        # try to add ability for audio feedback to user.
        audio_enabled = self.prefs.getpref('audio_enabled', True, bool)
        self.widgets.chk_en_audio.set_active(audio_enabled)
        if _AUDIO_AVAILABLE:
            LOG.info(_("Audio available!"))
            if audio_enabled:
                self.audio_active = True
                LOG.info(_("Audio enabled!"))
            else:
                self.audio_active = False
                LOG.info(_("Audio disabled!"))
            # the sounds to play if an error or message rises
            self.alert_sound = "/usr/share/sounds/freedesktop/stereo/dialog-warning.oga"
            self.error_sound = "/usr/share/sounds/freedesktop/stereo/dialog-error.oga"

            self.audio = player.Player()
            self.alert_sound = self.prefs.getpref('audio_alert', self.alert_sound, str)
            self.error_sound = self.prefs.getpref('audio_error', self.error_sound, str)
            self.widgets.audio_alert_chooser.set_filename(self.alert_sound)
            self.widgets.audio_error_chooser.set_filename(self.error_sound)
        else:
            self.audio_active = False
            LOG.warning(_("No audio available! PYGST library not installed? "
            "Is python-gstX.XX installed?"))

            self.widgets.audio_alert_chooser.set_sensitive(False)
            self.widgets.audio_error_chooser.set_sensitive(False)

    # init the preview
    def _init_gremlin( self ):
        LOG.debug("Entering init gremlin")

        grid_size = self.prefs.getpref( 'grid_size', 1.0, float )
        self.widgets.grid_size.set_value( grid_size )
        self.widgets.gremlin.grid_size = grid_size
        view = view = self.prefs.getpref("view", "p", str )
        self.widgets.gremlin.set_property("view", view )
        self.widgets.gremlin.set_property("metric_units", int( self.stat.linear_units ) )
        self.widgets.gremlin.set_property("mouse_btn_mode", self.prefs.getpref( "mouse_btn_mode", 4, int ) )
        self.widgets.gremlin.set_property("use_commanded", not self.dro_actual)
        self.widgets.gremlin.set_property("enable_dro", self.enable_gremlin_dro)
        # Set name to assign CSS
        self.widgets.eb_program_label.set_name("eb_program_label")
        self.widgets.eb_blockheight_label.set_name("eb_blockheight_label")

    def _init_kinematics_type (self):
        LOG.debug("Kinematics type changed")
        if self.stat.kinematics_type != linuxcnc.KINEMATICS_IDENTITY:
            self.widgets.gremlin.set_property("enable_dro", True )
            self.widgets.gremlin.use_joints_mode = True
            self.widgets.tbtn_switch_mode.show()
            self.widgets.tbtn_switch_mode.set_label(_(" Joint\nmode"))
            self.widgets.tbtn_switch_mode.set_sensitive(False)
            self.widgets.tbtn_switch_mode.set_active(True)
            self.widgets.lbl_replace_mode_btn.hide()
            self.widgets.ntb_jog_JA.set_current_page(1)
        else:
            self.widgets.gremlin.set_property("enable_dro", self.enable_gremlin_dro )
            self.widgets.gremlin.use_joints_mode = False
            self.widgets.tbtn_switch_mode.hide()
            self.widgets.lbl_replace_mode_btn.show()
            self.widgets.ntb_jog_JA.set_current_page(0)

    # init the function to hide the cursor
    def _init_hide_cursor(self):
        hide_cursor = self.prefs.getpref('hide_cursor', False, bool)
        self.widgets.chk_hide_cursor.set_active(hide_cursor)

    # init to hide tooltips
    def _init_hide_tooltips(self):
        self.widgets_with_tooltips = []
        for widget in self.widgets:
            if hasattr(widget, "set_has_tooltip"):
                self.widgets_with_tooltips.append(widget)
        self.hide_tooltips = self.prefs.getpref("hide_tooltips", False, bool)
        self.widgets.chk_hide_tooltips.set_active(self.hide_tooltips)
        self._set_enable_tooltips(not self.hide_tooltips)

# =============================================================
# Onboard keyboard handling Start

    # shows "Onboard" virtual keyboard if available
    # else error message
    def _init_keyboard(self, args="", x="", y=""):
        self.onboard = False

        # now we check if onboard is installed
        try:
            if os.path.isfile("/usr/bin/onboard"):
                self.onboard_kb = subprocess.Popen(["onboard", "--xid", args, x, y],
                                                   stdin=subprocess.PIPE,
                                                   stdout=subprocess.PIPE,
                                                   close_fds=True)
                LOG.info(_("Virtual keyboard program found : <onboard>"))
            else:
                LOG.warning(_("No virtual keyboard installed, we checked for <onboard>. "
                "Try 'sudo apt-get install onboard'."))
                self._no_virt_keyboard()
                return
            sid = self.onboard_kb.stdout.readline()
            self.socket = Gtk.Socket()
            self.widgets.key_box.add(self.socket)
            self.socket.add_id(int(sid))
            self.socket.show_all()
            self.widgets.key_box.show_all()
            self._dynamic_childs[int(sid)] = self.socket
            self.onboard = True
        except Exception as e:
            LOG.error(_("Error with launching virtual keyboard. Is onboard installed?"
            "Try sudo apt-get install onboard"))
            traceback.print_exc()
            self._no_virt_keyboard()


        # get when the keyboard should be shown
        # and set the corresponding button active
        # only if onbaoard keyboard is ok.
        if self.onboard:
            self.widgets.chk_use_kb_on_offset.set_active(self.prefs.getpref("show_keyboard_on_offset",
                                                                            False, bool))
            self.widgets.chk_use_kb_on_tooledit.set_active(self.prefs.getpref("show_keyboard_on_tooledit",
                                                                              False, bool))
            self.widgets.chk_use_kb_on_edit.set_active(self.prefs.getpref("show_keyboard_on_edit",
                                                                          True, bool))
            self.widgets.chk_use_kb_on_mdi.set_active(self.prefs.getpref("show_keyboard_on_mdi",
                                                                         True, bool))
            self.widgets.chk_use_kb_on_file_selection.set_active(self.prefs.getpref("show_keyboard_on_file_selection",
                                                                                    False, bool))
        else:
            self._no_virt_keyboard()

    def _no_virt_keyboard(self):
        # In this case we will disable the corresponding part on the settings page
        self.widgets.chk_use_kb_on_offset.set_active(False)
        self.widgets.chk_use_kb_on_tooledit.set_active(False)
        self.widgets.chk_use_kb_on_edit.set_active(False)
        self.widgets.chk_use_kb_on_mdi.set_active(False)
        self.widgets.chk_use_kb_on_file_selection.set_active(False)
        self.widgets.frm_keyboard.set_sensitive(False)
        self._change_kbd_image("img_macro_menu_stop")
        self.macro_dic["keyboard"].set_sensitive(False)
        self.macro_dic["keyboard"].set_property("tooltip-text", _("interrupt running macro"))
        self.widgets.btn_keyb.set_sensitive(False)

    def _kill_keyboard(self):
        LOG.debug("_kill_keyboard")
        try:
            self.onboard_kb.kill()
            self.onboard_kb.terminate()
            self.onboard_kb = None
        except:
            try:
                self.onboard_kb.kill()
                self.onboard_kb.terminate()
                self.onboard_kb = None
            except:
                pass

# Onboard keyboard handling End
# =============================================================

    def _init_offsetpage(self):
        temp = "xyzabcuvw"
        self.widgets.offsetpage1.set_col_visible(temp, False)
        temp = ""
        for axis in self.axis_list:
            temp = temp + axis
        self.widgets.offsetpage1.set_col_visible(temp, True)

        parameterfile = self.get_ini_info.get_parameter_file()
        if not parameterfile:
            message = _("Did not find a parameter file in [RS274NGC] PARAMETER_FILE")
            LOG.error(message)
            self.dialogs.warning_dialog(self, _("Very critical situation"), message, sound = False)
            sys.exit()
        path = os.path.join(CONFIGPATH, parameterfile)
        self.widgets.offsetpage1.set_filename(path)

        self.widgets.offsetpage1.set_display_follows_program_units()
        if self.stat.program_units != 1:
            self.widgets.offsetpage1.set_to_mm()
            self.widgets.offsetpage1.machine_units_mm = _MM
        else:
            self.widgets.offsetpage1.set_to_inch()
            self.widgets.offsetpage1.machine_units_mm = _INCH
        self.widgets.offsetpage1.hide_buttonbox(True)
        self.widgets.offsetpage1.set_row_visible("1", False)
        self.widgets.offsetpage1.set_font("sans 12")
        self.widgets.offsetpage1.set_foreground_color(self._get_RGBA_color("#28D0D9"))
        self.widgets.offsetpage1.selection_mask = ("Tool", "G5x", "Rot")
        names = []
        default_names = self.widgets.offsetpage1.get_names()
        for system, name in default_names:
            system_name = "system_name_{0}".format(system).lower()
            name = self.prefs.getpref(system_name, name, str)
            names.append([system, name])
        self.widgets.offsetpage1.set_names(names)

    # Icon file selection stuff
    def _init_IconFileSelection(self):
        self.widgets.IconFileSelection1.set_property("start_dir", self.get_ini_info.get_program_prefix())

        file_ext = self.get_ini_info.get_file_ext()
        filetypes = ""
        for ext in file_ext:
            filetypes += ext.replace("*.", "") + ","
        self.widgets.IconFileSelection1.set_property("filetypes", filetypes)

        jump_to_dir = self.prefs.getpref("jump_to_dir", os.path.expanduser("~"), str)
        self.widgets.jump_to_dir_chooser.set_current_folder(jump_to_dir)
        self.widgets.IconFileSelection1.set_property("jump_to_dir", jump_to_dir)

        self.widgets.IconFileSelection1.show_buttonbox(False)
        self.widgets.IconFileSelection1.show_filelabel(False)

        # now we initialize the button states
        self.widgets.btn_home.set_sensitive(self.widgets.IconFileSelection1.btn_home.get_sensitive())
        self.widgets.btn_dir_up.set_sensitive(self.widgets.IconFileSelection1.btn_dir_up.get_sensitive())
        self.widgets.btn_sel_prev.set_sensitive(self.widgets.IconFileSelection1.btn_sel_prev.get_sensitive())
        self.widgets.btn_sel_next.set_sensitive(self.widgets.IconFileSelection1.btn_sel_next.get_sensitive())
        self.widgets.btn_select.set_sensitive(self.widgets.IconFileSelection1.btn_select.get_sensitive())
        self.widgets.btn_jump_to.set_sensitive(self.widgets.IconFileSelection1.btn_jump_to.get_sensitive())
        self.widgets.btn_jump_to.set_sensitive(self.widgets.IconFileSelection1.btn_jump_to.get_sensitive())

    # init the keyboard shortcut bindings
    def _init_keybindings(self):
        try:
            accel_group = Gtk.AccelGroup()
            self.widgets.window1.add_accel_group(accel_group)
            self.widgets.button_estop.add_accelerator("clicked", accel_group, 65307, 0, Gtk.ACCEL_LOCKED)
        except:
            pass
        self.widgets.window1.connect("key_press_event", self.on_key_event, 1)
        self.widgets.window1.connect("key_release_event", self.on_key_event, 0)

    # Initialize the file to load dialog, setting an title and the correct
    # folder as well as a file filter
    def _init_file_to_load(self):
        file_dir = self.get_ini_info.get_program_prefix()
        self.widgets.file_to_load_chooser.set_current_folder(file_dir)
        title = _("Select the file you want to be loaded at program start")
        self.widgets.file_to_load_chooser.set_title(title)
        self.widgets.ff_file_to_load.set_name("linuxcnc files")
        self.widgets.ff_file_to_load.add_pattern("*.ngc")
        file_ext = self.get_ini_info.get_file_ext()
        for ext in file_ext:
            self.widgets.ff_file_to_load.add_pattern(ext)

    # search for and set up user requested message system.
    # status displays on the statusbat and requires no acknowledge.
    # dialog displays a GTK dialog box with yes or no buttons
    # okdialog displays a GTK dialog box with an ok button
    # dialogs require an answer before focus is sent back to main screen
    def _init_user_messages(self):
        user_messages = self.get_ini_info.get_user_messages()
        if not user_messages:
            return
        for message in user_messages:
            if message[1] == "status":
                pin = hal_glib.GPin(self.halcomp.newpin("messages." + message[2], hal.HAL_BIT, hal.HAL_IN))
                pin.connect("value_changed", self._show_user_message, message)
            elif message[1] == "okdialog":
                pin = hal_glib.GPin(self.halcomp.newpin("messages." + message[2], hal.HAL_BIT, hal.HAL_IN))
                pin.connect("value_changed", self._show_user_message, message)
                pin = hal_glib.GPin(
                    self.halcomp.newpin("messages." + message[2] + "-waiting", hal.HAL_BIT, hal.HAL_OUT))
            elif message[1] == "yesnodialog":
                pin = hal_glib.GPin(self.halcomp.newpin("messages." + message[2], hal.HAL_BIT, hal.HAL_IN))
                pin.connect("value_changed", self._show_user_message, message)
                pin = hal_glib.GPin(
                    self.halcomp.newpin("messages." + message[2] + "-waiting", hal.HAL_BIT, hal.HAL_OUT))
                pin = hal_glib.GPin(
                    self.halcomp.newpin("messages." + message[2] + "-response", hal.HAL_BIT, hal.HAL_OUT))
            else:
                LOG.error(_("Message type {0} not supported").format(message[1]))

    def _show_user_message(self, pin, message):
        if message[1] == "status":
            if pin.get():
                self._show_error((0, message[0]))
        elif message[1] == "okdialog":
            self.halcomp["messages." + message[2] + "-waiting"] = 0
            if pin.get():
                self.halcomp["messages." + message[2] + "-waiting"] = 1
                title = "Pin " + message[2] + " message"
                response = self.dialogs.show_user_message(self, message[0], title)
                self.halcomp["messages." + message[2] + "-waiting"] = 0
        elif message[1] == "yesnodialog":
            if pin.get():
                self.halcomp["messages." + message[2] + "-waiting"] = 1
                self.halcomp["messages." + message[2] + "-response"] = 0
                title = "Pin " + message[2] + " message"
                response = self.dialogs.yesno_dialog(self, message[0], title)
                self.halcomp["messages." + message[2] + "-waiting"] = 0
                self.halcomp["messages." + message[2] + "-response"] = response
            else:
                self.halcomp["messages." + message[2] + "-waiting"] = 0
        else:
            LOG.error(_("Message type {0} not supported").format(message[1]))

    def _show_offset_tab(self, state):
        page = self.widgets.ntb_preview.get_nth_page(1)
        if page.get_visible() and state or not page.get_visible() and not state:
            return
        if state:
            page.show()
            self.widgets.ntb_preview.set_property("show-tabs", state)
            self.widgets.ntb_preview.set_current_page(1)
            self.widgets.offsetpage1.mark_active((self.system_list[self.stat.g5x_index]).lower())
            if self.widgets.chk_use_kb_on_offset.get_active():
                self.widgets.ntb_info.set_current_page(1)
        else:
            names = self.widgets.offsetpage1.get_names()
            for system, name in names:
                system_name = "system_name_{0}".format(system).lower()
                self.prefs.putpref(system_name, name)
            page.hide()

            self.touch_button_dic["edit_offsets"].set_active(False)
            self.widgets.ntb_preview.set_current_page(0)
            self.widgets.ntb_info.set_current_page(0)
            if self.widgets.ntb_preview.get_n_pages() <= 4:  # else user tabs are available
                self.widgets.ntb_preview.set_property("show-tabs", state)

    def _show_tooledit_tab(self, state):
        page = self.widgets.ntb_preview.get_nth_page(2)
        if page.get_visible() and state or not page.get_visible() and not state:
            return
        if state:
            page.show()
            self.widgets.ntb_preview.set_property("show-tabs", not state)
            self.widgets.vbx_jog.hide()
            self.widgets.ntb_preview.set_current_page(2)
            self.widgets.tooledit1.set_selected_tool(self.stat.tool_in_spindle)
            if self.widgets.chk_use_kb_on_tooledit.get_active():
                self.widgets.ntb_info.set_current_page(1)
        else:
            page.hide()
            if self.widgets.ntb_preview.get_n_pages() > 4:  # user tabs are available
                self.widgets.ntb_preview.set_property("show-tabs", not state)
            self.widgets.vbx_jog.show()
            self.widgets.ntb_preview.set_current_page(0)
            self.widgets.ntb_info.set_current_page(0)

    def _show_iconview_tab(self, state):
        page = self.widgets.ntb_preview.get_nth_page(3)
        if page.get_visible() and state or not page.get_visible() and not state:
            return
        if state:
            page.show()
            self.widgets.ntb_preview.set_property("show-tabs", not state)
            self.widgets.ntb_preview.set_current_page(3)
            if self.widgets.chk_use_kb_on_file_selection.get_active():
                self.widgets.box_info.show()
                self.widgets.ntb_info.set_current_page(1)
        else:
            page.hide()
            if self.widgets.ntb_preview.get_n_pages() > 4:  # user tabs are available
                self.widgets.ntb_preview.set_property("show-tabs", not state)
            self.widgets.ntb_preview.set_current_page(0)
            self.widgets.ntb_info.set_current_page(0)

    # every 100 milli seconds this gets called
    # check linuxcnc for status, error and then update the readout
    def _periodic(self):
        # we put the poll command in a try, so if the linuxcnc pid is killed
        # from an external command, we also quit the GUI
        try:
            self.stat.poll()
        except:
            raise SystemExit("gmoccapy can not poll linuxcnc status any more")

        error = self.error_channel.poll()
        if error:
            self._show_error(error)

        if self.gcodes != self.stat.gcodes:
            self._update_active_gcodes()
        if self.mcodes != self.stat.mcodes:
            self._update_active_mcodes()

        if self.lathe_mode:
            if "G8" in self.active_gcodes and self.diameter_mode:
                self._switch_to_g7(False)
            elif "G7" in self.active_gcodes and not self.diameter_mode:
                self._switch_to_g7(True)

        self._update_vel()
        self._update_coolant()
        self._update_spindle()
        self._update_halui_pin()
        self._update_vc()

        self.widgets.lbl_time.set_label(strftime("%H:%M:%S") + "\n" + strftime("%d.%m.%Y"))

        # keep the timer running
        return True

    # every 1 second this gets called
    def _periodic_1s(self):
        if hal.get_value('halui.program.is-running'):
            self.elapsed_time_run += 1
            self._update_progressbar_text()
        return True


    def _show_error(self, error):
        kind, text = error
        # print(kind,text)
        if kind in (linuxcnc.NML_ERROR, linuxcnc.OPERATOR_ERROR):
            icon = ALERT_ICON
            self.halcomp["error"] = True
        elif kind in (linuxcnc.NML_TEXT, linuxcnc.OPERATOR_TEXT):
            icon = INFO_ICON
        elif kind in (linuxcnc.NML_DISPLAY, linuxcnc.OPERATOR_DISPLAY):
            icon = INFO_ICON
        else:
            icon = ALERT_ICON
        if text == "" or text == None:
            text = _("Unknown error type and no error text given")
        self.notification.add_message(text, icon)

        if self.audio_active:
            if kind == 1 or kind == 11:
                self._on_play_sound(None, "error")
            else:
                self._on_play_sound(None, "alert")

    def on_gremlin_gcode_error(self, widget, errortext):
        self.gcodeerror = errortext
        self.dialogs.warning_dialog(self, _("Important Warning"), errortext)


# =========================================================
# button handlers Start

    # toggle emergency button
    def on_tbtn_estop_toggled(self, widget, data=None):
        if widget.get_active():  # estop is active, open circuit
            self.command.state(linuxcnc.STATE_ESTOP)
            self.command.wait_complete()
            self.stat.poll()
            if self.stat.task_state == linuxcnc.STATE_ESTOP_RESET:
                widget.set_active(False)
        else:  # estop circuit is fine
            self.command.state(linuxcnc.STATE_ESTOP_RESET)
            self.command.wait_complete()
            self.stat.poll()
            if self.stat.task_state == linuxcnc.STATE_ESTOP:
                widget.set_active(True)
                self._show_error((11, _("External ESTOP is set, could not change state!")))

    # toggle machine on / off button
    def on_tbtn_on_toggled(self, widget, data=None):
        if widget.get_active():
            if self.stat.task_state == linuxcnc.STATE_ESTOP:
                widget.set_active(False)
                return
            self.command.state(linuxcnc.STATE_ON)
            self.command.wait_complete()
            self.stat.poll()
            if self.stat.task_state != linuxcnc.STATE_ON:
                widget.set_active(False)
                self._show_error((11, _("Could not switch the machine on, is limit switch activated?")))
                self._update_widgets(False)
                return
            self._update_widgets(True)
        else:
            self.command.state(linuxcnc.STATE_OFF)
            self._update_widgets(False)

    # The mode buttons
    def on_rbt_manual_pressed(self, widget, data=None):
        LOG.debug("mode Manual")
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()

    def on_rbt_mdi_pressed(self, widget, data=None):
        LOG.debug("mode MDI")
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()

    def on_rbt_auto_pressed(self, widget, data=None):
        LOG.debug("mode Auto")
        self.command.mode(linuxcnc.MODE_AUTO)
        self.command.wait_complete()

    # If button exit is clicked, press emergency button before closing the application
    def on_btn_exit_clicked(self, widget, data=None):
        self.widgets.window1.destroy()

# button handlers End
# =========================================================

# =========================================================
# hal status Start

    # use the hal_status widget to control buttons and
    # actions allowed by the user and sensitive widgets
    def on_hal_status_all_homed(self, widget):
        LOG.debug("Hal Status all homed")
        self.all_homed = True
        self.widgets.ntb_button.set_current_page(_BB_MANUAL)
        widgetlist = ["rbt_mdi", "rbt_auto", "btn_index_tool", "btn_change_tool", "btn_select_tool_by_no",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z", "btn_touch", "tbtn_switch_mode"
        ]
        self._sensitize_widgets(widgetlist, True)
        self._set_motion_mode(1)
        if self.widgets.chk_reload_tool.get_active():
            # if there is already a tool in spindle, the user
            # homed the second time, unfortunately we will then
            # not get out of MDI mode any more
            # That happen, because the tool in spindle did not change, so the
            # tool info is not updated and we self.change_tool will not be reset
            if self.stat.tool_in_spindle != 0:
                return
            self.reload_tool()
            self.command.mode(linuxcnc.MODE_MANUAL)

    def on_hal_status_not_all_homed(self, widget, joints):
        LOG.debug("Hal Status not all homed {0}".format(joints))
        self.all_homed = False
        if self.no_force_homing:
            return
        widgetlist = ["rbt_mdi", "rbt_auto", "btn_index_tool", "btn_touch", "btn_change_tool", "btn_select_tool_by_no",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z", "btn_touch", "tbtn_switch_mode"
        ]
        self._sensitize_widgets(widgetlist, False)
        self._set_motion_mode(0)

    def on_hal_status_file_loaded(self, widget, filename):
        widgetlist = ["btn_use_current" ]
        # this test is only necessary, because of remap and toolchange, it will emit a file loaded signal
        if filename:
            fileobject = open(filename, 'r')
            lines = fileobject.readlines()
            fileobject.close()
            self.halcomp["program.length"] = len(lines)

            if len(filename) > 70:
                filename = filename[0:10] + "..." + filename[len(filename) - 50:len(filename)]
            self.widgets.lbl_program.set_text(filename)
            self._sensitize_widgets(widgetlist, True)
        else:
            self.halcomp["program.length"] = 0
            self._sensitize_widgets(widgetlist, False)
            self.widgets.lbl_program.set_text(_("No file loaded"))

    def on_hal_status_line_changed(self, widget, line):
        self.halcomp["program.current-line"] = line
        # this test is only necessary, because of remap and toolchange, it will emit a file loaded signal
        if self.halcomp["program.length"] > 0:
            self.halcomp["program.progress"] = 100.00 * line / self.halcomp["program.length"]
        else:
            self.halcomp["program.progress"] = 0.0

        # The program length used for the progress calculation is decreased here by 1 because
        # the last line doesn't emit a line-changed signal.
        self.progress = line / (self.halcomp["program.length"]-1)
        if self.progress > 1.0: self.progress = 1.0
        self.widgets.progressbar_pgm.set_fraction(self.progress)
        self._update_progressbar_text()

    def _update_progressbar_text(self):
        self.widgets.progressbar_pgm.set_text(f"{self.progress*100:.0f} %  ({self.seconds_to_hms(self.elapsed_time_run)})")

    def on_hal_status_interp_idle(self, widget):
        LOG.debug("IDLE")
        if self.load_tool:
            return

        widgetlist = ["ntb_jog", "btn_from_line", "gcode_view",
                      "tbtn_flood", "tbtn_mist", "rbt_forward", "rbt_reverse", "rbt_stop",
                      "btn_load", "btn_edit", "tbtn_optional_blocks", "btn_reload"
        ]
        if not self.widgets.rbt_hal_unlock.get_active() and not self.user_mode:
            widgetlist.append("tbtn_setup")

        if not self.widgets.tbtn_setup.get_active():
            widgetlist.append("rbt_manual")

        if self.all_homed or self.no_force_homing:
            if not self.widgets.tbtn_setup.get_active():
                widgetlist.append("rbt_mdi")
                widgetlist.append("rbt_auto")
            widgetlist.append("btn_index_tool")
            widgetlist.append("btn_change_tool")
            widgetlist.append("btn_select_tool_by_no")
            widgetlist.append("btn_tool_touchoff_x")
            widgetlist.append("btn_tool_touchoff_z")
            widgetlist.append("btn_touch")

        # This happen because hal_glib does emit the signals in the order that idle is emitted later that estop
        if self.stat.task_state == linuxcnc.STATE_ESTOP or self.stat.task_state == linuxcnc.STATE_OFF:
            self._sensitize_widgets(widgetlist, False)
        else:
            self._sensitize_widgets(widgetlist, True)

        for btn in self.macrobuttons:
            btn.set_sensitive(True)

        if self.onboard:
            self._change_kbd_image("img_macro_menu_keyboard")
        else:
            self._change_kbd_image("img_macro_menu_stop")
            self.macro_dic["keyboard"].set_sensitive(False)

        self.widgets.btn_run.set_sensitive(True)
        self.widgets.btn_stop.set_sensitive(False)

        if self.tool_change:
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self.tool_change = False

        self.halcomp["program.current-line"] = 0
        self.halcomp["program.progress"] = 0.0

    def on_hal_status_interp_run(self, widget):
        LOG.debug("RUN")

        widgetlist = ["rbt_manual", "rbt_mdi", "rbt_auto", "tbtn_setup", "btn_index_tool",
                      "btn_from_line", "btn_change_tool", "btn_select_tool_by_no",
                      "btn_load", "btn_edit", "tbtn_optional_blocks", "rbt_reverse", "rbt_stop", "rbt_forward",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z", "btn_touch", "btn_reload"
        ]
        # in MDI it should be possible to add more commands, even if the interpreter is running
        if self.stat.task_mode != linuxcnc.MODE_MDI:
            widgetlist.append("gcode_view")

        self._sensitize_widgets(widgetlist, False)
        self.widgets.btn_run.set_sensitive(False)
        self.widgets.btn_stop.set_sensitive(True)

        self._change_kbd_image("img_macro_menu_stop")
        self.macro_dic["keyboard"].set_sensitive(True)
        self.elapsed_time_run = 0

    def on_hal_status_tool_in_spindle_changed(self, object, new_tool_no):
        LOG.debug("hal signal tool changed")
        # need to save the tool in spindle as preference, to be able to reload it on startup
        self.prefs.putpref("tool_in_spindle", new_tool_no, int)
        self._update_toolinfo(new_tool_no)

    def on_hal_status_state_estop(self, widget=None):
        self.widgets.tbtn_estop.set_active(True)
        self.widgets.tbtn_estop.set_image(self.widgets.img_emergency)
        self.widgets.tbtn_on.set_image(self.widgets.img_machine_off)
        self.widgets.tbtn_on.set_sensitive(False)
        self.widgets.tbtn_on.set_active(False)
        self.command.mode(linuxcnc.MODE_MANUAL)

    def on_hal_status_state_estop_reset(self, widget=None):
        self.widgets.tbtn_estop.set_active(False)
        self.widgets.tbtn_estop.set_image(self.widgets.img_emergency_off)
        self.widgets.tbtn_on.set_image(self.widgets.img_machine_off)
        self.widgets.tbtn_on.set_sensitive(True)
        self.widgets.ntb_jog.set_sensitive(True)
        self.widgets.ntb_jog_JA.set_sensitive(False)
        self.widgets.vbtb_jog_incr.set_sensitive(False)
        self.widgets.hbox_jog_vel.set_sensitive(False)

        # activate limit override only if limit switch active
        if any(self.stat.limit):
            self.widgets.chk_ignore_limits.set_sensitive(True)
        else:
            self.widgets.chk_ignore_limits.set_sensitive(False)

    def on_hal_status_state_off(self, widget):
        widgetlist = ["rbt_manual", "rbt_mdi", "rbt_auto", "btn_homing", "btn_touch", "btn_tool",
                      "hbox_jog_vel", "ntb_jog_JA", "vbtb_jog_incr", "spc_feed", "btn_feed_100", "rbt_forward", "btn_index_tool",
                      "rbt_reverse", "rbt_stop", "tbtn_flood", "tbtn_mist", "btn_change_tool", "btn_select_tool_by_no",
                      "btn_spindle_100", "spc_rapid", "spc_spindle",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z"
        ]
        self._sensitize_widgets(widgetlist, False)
        if self.widgets.tbtn_on.get_active():
            self.widgets.tbtn_on.set_active(False)
        self.widgets.tbtn_on.set_image(self.widgets.img_machine_off)
        self.widgets.btn_exit.set_sensitive(True)
        self.widgets.ntb_main.set_current_page(0)
        self.widgets.ntb_button.set_current_page(_BB_MANUAL)
        self.widgets.ntb_info.set_current_page(0)
        self.widgets.ntb_jog.set_current_page(0)

    def on_hal_status_state_on(self, widget):
        widgetlist = ["rbt_manual", "btn_homing", "btn_touch", "btn_tool",
                      "ntb_jog", "spc_feed", "btn_feed_100", "rbt_forward",
                      "rbt_reverse", "rbt_stop", "tbtn_flood", "tbtn_mist",
                      "btn_spindle_100", "spc_rapid", "spc_spindle"
        ]
        self._sensitize_widgets(widgetlist, True)
        if not self.widgets.tbtn_on.get_active():
            self.widgets.tbtn_on.set_active(True)
        self.widgets.tbtn_on.set_image(self.widgets.img_machine_on)
        self.widgets.btn_exit.set_sensitive(False)
        if self.widgets.ntb_main.get_current_page() != 0:
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()

        # activate limit override only if limit switch active
        if any(self.stat.limit):
            self.widgets.chk_ignore_limits.set_sensitive(True)
        else:
            self.widgets.chk_ignore_limits.set_sensitive(False)

    def on_hal_status_override_limits_changed(self, object, state, limits_list):
        # object = hal_status from glade file
        # state = true if override_limit is active
        # limits_list = list of joint with active override limits, normally all)
        self.widgets.chk_ignore_limits.set_active(state)

    def on_hal_status_limits_tripped(self, object, state, lst_limits):
        # object = hal_status from glade file
        # state = true if limit has been tripped
        # lst_limits = list of joint limits that has been tripped ([0,0],[0,1],[0,0])
        self.widgets.chk_ignore_limits.set_sensitive(state)
        if state:
            # sensitize ntb_jog when limit tripped
            self.widgets.ntb_jog.set_sensitive(True)
        else:
            # refresh immediately when limit is no longer active
            self.widgets.chk_ignore_limits.set_active(False)

    def on_hal_status_mode_manual(self, widget):
        LOG.debug("MANUAL Mode")
        self.widgets.rbt_manual.set_active(True)
        # if setup page is activated, we must leave here, otherwise the pages will be reset
        if self.widgets.tbtn_setup.get_active():
            return
        # if we are in user tabs, we must reset the button
        if self.widgets.tbtn_user_tabs.get_active():
            self.widgets.tbtn_user_tabs.set_active(False)
        self.widgets.ntb_main.set_current_page(0)
        if self.widgets.ntb_button.get_current_page() != _BB_TOUCH_OFF:
            self.widgets.ntb_button.set_current_page(_BB_MANUAL)
        self.widgets.ntb_info.set_current_page(0)
        self.widgets.ntb_jog.set_current_page(0)

        # if the status changed, we reset the key event, otherwise the key press
        # event will not change, if the user did the last change with keyboard shortcut
        # This is caused, because we record the last key event to avoid multiple key
        # press events by holding down the key. I.e. One press should only advance one increment
        # on incremental jogging.
        self.last_key_event = None, 0

    def on_hal_status_mode_mdi(self, widget):
        LOG.debug("MDI Mode, tool_change = {0}".format(self.tool_change))

        # if the edit offsets button is active, we do not want to change
        # pages, as the user may want to edit several axis values
        if self.touch_button_dic["edit_offsets"].get_active():
            return

        # self.tool_change is set only if the tool change was commanded
        # from tooledit widget/page, so we do not want to switch the
        # screen layout to MDI, but set the manual widgets
        if self.tool_change:
            self.widgets.ntb_main.set_current_page(0)
            self.widgets.ntb_button.set_current_page(_BB_MANUAL)
            self.widgets.ntb_info.set_current_page(0)
            self.widgets.ntb_jog.set_current_page(0)
            return

        # if MDI button is not sensitive, we are not ready for MDI commands
        # so we have to abort external commands and get back to manual mode
        # This will happen mostly, if we are in settings mode, as we do disable the mode button
        if not self.widgets.rbt_mdi.get_sensitive():
            self.command.abort()
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self._show_error((13, _("It is not possible to change to MDI Mode at the moment")))
            return
        else:
            # if we are in user tabs, we must reset the button
            if self.widgets.tbtn_user_tabs.get_active():
                self.widgets.tbtn_user_tabs.set_active(False)
            if self.widgets.chk_use_kb_on_mdi.get_active():
                self.widgets.ntb_info.set_current_page(1)
            else:
                self.widgets.ntb_info.set_current_page(0)
            self.widgets.ntb_main.set_current_page(0)
            self.widgets.ntb_button.set_current_page(_BB_MDI)
            self.widgets.ntb_jog.set_current_page(1)
            self.widgets.hal_mdihistory.entry.grab_focus()
            self.widgets.rbt_mdi.set_active(True)

            # if the status changed, we reset the key event, otherwise the key press
            # event will not change, if the user did the last change with keyboard shortcut
            # This is caused, because we record the last key event to avoid multiple key
            # press events by holding down the key. I.e. One press should only advance one increment
            # on incremental jogging.
            self.last_key_event = None, 0

    def on_hal_status_mode_auto(self, widget):
        LOG.debug("AUTO Mode")
        # if Auto button is not sensitive, we are not ready for AUTO commands
        # so we have to abort external commands and get back to manual mode
        # This will happen mostly, if we are in settings mode, as we do disable the mode button
        if not self.widgets.rbt_auto.get_sensitive():
            self.command.abort()
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self._show_error((13, _("It is not possible to change to Auto Mode at the moment")))
            return
        else:
            # if we are in user tabs, we must reset the button
            if self.widgets.tbtn_user_tabs.get_active():
                self.widgets.tbtn_user_tabs.set_active(False)
            self.widgets.ntb_main.set_current_page(0)
            self.widgets.ntb_button.set_current_page(_BB_AUTO)
            self.widgets.ntb_info.set_current_page(0)
            self.widgets.ntb_jog.set_current_page(2)
            self.widgets.rbt_auto.set_active(True)

            # if the status changed, we reset the key event, otherwise the key press
            # event will not change, if the user did the last change with keyboard shortcut
            # This is caused, because we record the last key event to avoid multiple key
            # press events by holding down the key. I.e. One press should only advance one increment
            # on incremental jogging.
            self.last_key_event = None, 0

    def on_hal_status_motion_mode_changed(self, widget, new_mode):
        LOG.debug("hal status motion mode changed")
        # Motion mode change in identity kinematics makes no sense
        # but we will react on the signal and correct the misbehavior
        # self.stat.motion_mode return
        # Mode 1 = joint ; Mode 2 = MDI ; Mode 3 = teleop
        # so in mode 1 we have to show Joints and in Modes 2 and 3 axis values

        widgetlist = ("rbt_mdi", "rbt_auto")
        if new_mode == 1 and self.stat.kinematics_type != linuxcnc.KINEMATICS_IDENTITY:
            self.widgets.gremlin.set_property("enable_dro", True)
            self.widgets.gremlin.use_joints_mode = True
            self.widgets.tbtn_switch_mode.set_active(True)
            self.widgets.ntb_jog_JA.set_current_page(1)
            state = False
        else:
            if not self.widgets.tbtn_fullsize_preview0.get_active():
                self.widgets.gremlin.set_property("enable_dro", self.enable_gremlin_dro)
            self.widgets.gremlin.use_joints_mode = False
            self.widgets.tbtn_switch_mode.set_active(False)
            self.widgets.ntb_jog_JA.set_current_page(0)
            state = True
        if self.stat.task_state != linuxcnc.STATE_ON:
            state = False

        if not self.widgets.tbtn_setup.get_active():
            self._sensitize_widgets(widgetlist, state)

    def on_hal_status_metric_mode_changed(self, widget, metric_units):
        LOG.debug("hal status metric mode changed")
        # set gremlin_units
        self.widgets.gremlin.set_property("metric_units", metric_units)
        # unit switch could be done here if not done in combi_dro/_position()
        # for dro in self.dro_dic:
        #     self.dro_dic[dro].set_property("metric_units", metric_units)

        widgetlist = ["spc_lin_jog_vel"]

        # self.stat.linear_units will return 1.0 for metric and 1/25,4 for imperial
        # display units not equal machine units
        if metric_units != int(self.stat.linear_units):
            # machine units = metric
            if self.stat.linear_units == _MM:
                self.faktor = (1.0 / 25.4)
            # machine units = imperial
            else:
                self.faktor = 25.4
            self.turtle_jog = self.turtle_jog * self.faktor
            self.rabbit_jog = self.rabbit_jog * self.faktor
            self._update_slider( widgetlist )

        else:
            # display units equal machine units would be factor = 1,
            # but if factor not equal 1.0 than we have to reconvert from previous first
            self.turtle_jog = self.turtle_jog / self.faktor
            self.rabbit_jog = self.rabbit_jog / self.faktor
            if self.faktor != 1.0:
                self.faktor = 1 / self.faktor
                self._update_slider(widgetlist)
                self.faktor = 1.0
                self._update_slider(widgetlist)

        if metric_units:
            self.widgets.spc_lin_jog_vel.set_digits(0)
            self.widgets.spc_lin_jog_vel.set_property("unit", _("mm/min"))
        else:
            self.widgets.spc_lin_jog_vel.set_digits(2)
            self.widgets.spc_lin_jog_vel.set_property("unit", _("inch/min"))

# hal status End
# =========================================================

    # There are some settings we can only do if the window is on the screen already
    def on_window1_show(self, widget, data=None):

        # it is time to get the correct estop state and set the button status
        self.stat.poll()
        if self.stat.task_state == linuxcnc.STATE_ESTOP:
            self.widgets.tbtn_estop.set_active(True)
            self.widgets.tbtn_estop.set_image(self.widgets.img_emergency)
            self.widgets.tbtn_on.set_image(self.widgets.img_machine_off)
            self.widgets.tbtn_on.set_sensitive(False)
        else:
            self.widgets.tbtn_estop.set_active(False)
            self.widgets.tbtn_estop.set_image(self.widgets.img_emergency_off)
            self.widgets.tbtn_on.set_sensitive(True)

        # if a file should be loaded, we will do so
        file = self.prefs.getpref("open_file", "", str)
        if file:
            self.widgets.file_to_load_chooser.set_filename(file)
            self.widgets.hal_action_open.load_file(file)

        # check how to start the GUI
        start_as = "rbtn_" + self.prefs.getpref("screen1", "window", str)
        self.widgets[start_as].set_active(True)
        self.widgets.window1.set_decorated(not self.prefs.getpref("hide_titlebar", False, bool))
        self.widgets.chkbtn_hide_titlebar.set_active(not self.widgets.window1.get_decorated())
        self.xpos = int(self.prefs.getpref("x_pos", self.xpos, float))
        self.ypos = int(self.prefs.getpref("y_pos", self.ypos, float))
        self.width = int(self.prefs.getpref("width", self.width, float))
        self.height = int(self.prefs.getpref("height", self.height, float))
        # set the adjustments according to Window position and size
        self.widgets.adj_x_pos.set_value(self.xpos)
        self.widgets.adj_y_pos.set_value(self.ypos)
        self.widgets.adj_width.set_value(self.width)
        self.widgets.adj_height.set_value(self.height)
        if start_as == "rbtn_fullscreen":
            self.widgets.window1.fullscreen()
        elif start_as == "rbtn_maximized":
            self.widgets.window1.maximize()
        else:
            # move and resize the window
            self.widgets.window1.move(self.xpos, self.ypos)
            self.widgets.window1.resize(self.width, self.height)
            
        # keyboard size
        self.widgets.chk_kbd_set_height.set_active(self.kbd_set_height)
        self.widgets.chk_kbd_set_width.set_active(self.kbd_set_width)
        self.widgets.adj_kbd_height.set_value(self.kbd_height)
        self.widgets.adj_kbd_width.set_value(self.kbd_width)
        self._update_kbd_height(self.kbd_set_height)
        self._update_kbd_width(self.kbd_set_width)

        # set initial state of widgets
        self.touch_button_dic["set_active"].set_sensitive(False)

        # set up the hal pin status of tool measurement
        # could not be done prior to this, as the hal pin are not created before
        # the tool measure check, due to the reason we need to know if creation
        # of tool measurement button is needed. So we call the toggle action to
        # set all up and running
        self.on_chk_use_tool_measurement_toggled(self.widgets.chk_use_tool_measurement)

        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()

        self.initialized = True

    # kill keyboard and estop machine before closing
    def on_window1_destroy(self, widget, data=None):
        LOG.debug("estoping / killing gmoccapy")
        self._kill_dynamic_childs()
#        self._kill_keyboard()
        if self.onboard:
            self._kill_keyboard()
        self.command.state(linuxcnc.STATE_OFF)
        self.command.state(linuxcnc.STATE_ESTOP)
        Gtk.main_quit()

    def on_focus_out(self, widget, data=None):
        self.stat.poll()
        if self.stat.enabled and self.stat.task_mode == linuxcnc.MODE_MANUAL and self.stat.current_vel > 0:
            # cancel any joints jogging
            JOGMODE = self._get_jog_mode()
            for jnum in range(self.stat.joints):
                self.command.jog(linuxcnc.JOG_STOP, JOGMODE, jnum)
            LOG.debug("Stopped jogging on focus-out-event")

    # What to do if a macro button has been pushed
    def _on_btn_macro_pressed( self, widget = None, data = None ):
        o_codes = data.split()

        command = str( "O<" + o_codes[0] + "> call" )

        for code in o_codes[1:]:
            parameter = self.dialogs.entry_dialog(self, data=None, header=_("Enter value:"),
                                                  label=_("Set parameter {0} to:").format(code), integer=False)
            if parameter == "ERROR":
                LOG.debug("conversion error")
                self.dialogs.warning_dialog(self, _("Conversion error !"),
                                            ("Please enter only numerical values\nValues have not been applied"))
                return
            elif parameter == "CANCEL":
                return
            else:
                pass
            command = command + " [" + str(parameter) + "] "
# TODO: Should not only clear the plot, but also the loaded program?
        # self.command.program_open("")
        # self.command.reset_interpreter()
        self.widgets.gremlin.clear_live_plotter()
# TODO: End
        self.command.mdi(command)
        for btn in self.macrobuttons:
            btn.set_sensitive(False)
        # we change the widget_image (done by hal status)
        # and use the button to interrupt running macros
        if not self.onboard:
            self.macro_dic["keyboard"].set_sensitive(True)
        self.widgets.ntb_info.set_current_page(0)

# helpers functions start
# =========================================================

    def _change_kbd_image(self, image):
        self.macro_dic["keyboard"].set_image(self.widgets[image])
        if self.onboard:
            self.macro_dic["keyboard"].set_property("tooltip-text", _("This button will show or hide the keyboard"))
        else:
            self.macro_dic["keyboard"].set_property("tooltip-text", _("interrupt running macro"))
        self.macro_dic["keyboard"].show_all()

    def _update_widgets(self, state):
        widgetlist = ["rbt_manual", "btn_homing", "btn_touch", "btn_tool",
                      "hbox_jog_vel", "ntb_jog_JA", "vbtb_jog_incr", "spc_feed", "btn_feed_100", "rbt_forward", "btn_index_tool",
                      "rbt_reverse", "rbt_stop", "tbtn_flood", "tbtn_mist", "btn_change_tool", "btn_select_tool_by_no",
                      "btn_spindle_100", "spc_rapid", "spc_spindle",
                      "btn_tool_touchoff_x", "btn_tool_touchoff_z"
        ]
        self._sensitize_widgets(widgetlist, state)

    def _switch_to_g7(self, state):
        # we do this only if we have a lathe, the check for lathe is done in gmoccapy
        LOG.debug("Switch to G7: {0}".format(state))
        if state:
            self.dro_dic["Combi_DRO_0"].set_property("abs_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_0"].set_property("rel_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_0"].set_property("dtg_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_9"].set_property("abs_color", self._get_RGBA_color(self.abs_color))
            self.dro_dic["Combi_DRO_9"].set_property("rel_color", self._get_RGBA_color(self.rel_color))
            self.dro_dic["Combi_DRO_9"].set_property("dtg_color", self._get_RGBA_color(self.dtg_color))
            self.diameter_mode = True
        else:
            self.dro_dic["Combi_DRO_9"].set_property("abs_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_9"].set_property("rel_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_9"].set_property("dtg_color", self._get_RGBA_color("#F2F1F0"))
            self.dro_dic["Combi_DRO_0"].set_property("abs_color", self._get_RGBA_color(self.abs_color))
            self.dro_dic["Combi_DRO_0"].set_property("rel_color", self._get_RGBA_color(self.rel_color))
            self.dro_dic["Combi_DRO_0"].set_property("dtg_color", self._get_RGBA_color(self.dtg_color))
            self.diameter_mode = False

    def on_key_event(self, widget, event, signal):

        # get the keyname
        keyname = Gdk.keyval_name(event.keyval)

        # estop with F1 should work every time
        # so should also escape abort actions
        if keyname == "F1":  # will estop the machine, but not reset estop!
            self.command.state(linuxcnc.STATE_ESTOP)
            return True
        if keyname == "Escape":
            self.command.abort()
            return True

        # change between teleop and world mode
        if keyname == "F12" or keyname == "$":
            if self.stat.task_mode != linuxcnc.MODE_MANUAL:
                return True
            # only change mode pressing the key, not releasing it
            if signal:
                # No mode switch to joints on Identity kinematics
                if self.stat.kinematics_type != linuxcnc.KINEMATICS_IDENTITY:
                    # Mode 1 = joint ; Mode 3 = teleop
                    if self.stat.motion_mode != 1:
                        self._set_motion_mode(0) # set joint mode
                    else:
                        self._set_motion_mode(1) # set teleop mode
            return True

        # This will avoid executing the key press event several times caused by keyboard auto repeat
        if self.last_key_event[0] == keyname and self.last_key_event[1] == signal:
            return True

        try:
            if keyname == "F2" and signal:
                # only turn on if no estop!
                if self.widgets.tbtn_estop.get_active():
                    return True
                self.widgets.tbtn_on.emit("clicked")
                return True
        except:
            pass

        if keyname == "space" and signal:
            if event.state & Gdk.ModifierType.CONTROL_MASK:  # only do it when control is hold down
                self.notification.del_message(-1)
                self.widgets.window1.grab_focus()
                return

        if keyname == "Super_L" and signal:  # Left Windows
            self.notification.del_last()
            self.widgets.window1.grab_focus()
            return

        # if the user do not want to use keyboard shortcuts, we leave here
        # in this case we do not return true, otherwise entering code in MDI history
        # and the integrated editor will not work
        if not self.widgets.chk_use_kb_shortcuts.get_active():
            LOG.debug("Settings say: do not use keyboard shortcuts, abort")
            return

        # Only in MDI mode the RETURN key should execute a command
        if keyname == "Return" and signal and self.stat.task_mode == linuxcnc.MODE_MDI:
            # LOG.debug("Got enter in MDI")
            self.widgets.hal_mdihistory.submit()
            self.widgets.hal_mdihistory.entry.grab_focus()
            # we need to leave here, otherwise the check for jogging
            # only allowed in manual mode will finish the sub
            return True

        # mode change are only allowed if the interpreter is idle, like mode switching
        if keyname == "F3" or keyname == "F5":
            if self.stat.interp_state != linuxcnc.INTERP_IDLE:
                if signal: # Otherwise the message will be shown twice
                    self._show_error((13, _("Mode change is only allowed if the interpreter is idle!")))
                return
            else:
                # F3 change to manual mode
                if keyname == "F3" and signal:
                    self.command.mode(linuxcnc.MODE_MANUAL)
                    self.command.wait_complete()
                    # we need to leave here, otherwise the check for jogging
                    # only allowed in manual mode will finish the sub
                    self.last_key_event = keyname, signal
                    return True

                # F5 should change to mdi mode
                if keyname == "F5" and signal:
                    self.command.mode(linuxcnc.MODE_MDI)
                    self.command.wait_complete()
                    # we need to leave here, otherwise the check for jogging
                    # only allowed in manual mode will finish the sub
                    self.last_key_event = keyname, signal
                    return True

        # in AUTO Mode we will allow the following key shortcuts
        # R = run program
        # P = pause program
        # S = resume program
        if self.stat.task_mode == linuxcnc.MODE_AUTO:
            # if we are in edit mode do not start a program!
            if self.widgets.ntb_button.get_current_page() == _BB_EDIT:
                return

            # all makes only sense, if a program is loaded,
            # if so, the button use current is sensitive
            if not self.widgets.btn_use_current.get_sensitive():
                return

            if (keyname == "R" or keyname == "r") and self.stat.interp_state == linuxcnc.INTERP_IDLE:
                if event.state & Gdk.ModifierType.CONTROL_MASK:
                    LOG.debug("R und Control gedrückt")
                    self.widgets.hal_action_reload.emit("activate")
                else:
                    self.command.auto(linuxcnc.AUTO_RUN,0)

            if (keyname == "p" or keyname == "P") and self.widgets.tbtn_pause.get_sensitive():
                self.command.auto(linuxcnc.AUTO_PAUSE)

            if (keyname == "S" or keyname == "s"):
                self.command.auto(linuxcnc.AUTO_RESUME)
                if self.widgets.tbtn_pause.get_active():
                    self.widgets.tbtn_pause.set_active(False)

        # Only in manual mode jogging with keyboard is allowed
        # in this case we do not return true, otherwise entering code in MDI history
        # and the integrated editor will not work
        # we also check if we are in settings or user page
        if self.stat.task_mode != linuxcnc.MODE_MANUAL or not self.widgets.ntb_main.get_current_page() == 0:
            return

        # This is just to avoid a terminal message, that this keys are not implemented:
        if (keyname == "R" or keyname == "r" or
            keyname == "p" or keyname == "P" or
            keyname == "S" or keyname == "s"):
            return

        # offset page is active, so keys must go through
        if self.widgets.ntb_preview.get_current_page() == 1:
            return

        # tooledit page is active, so keys must go through
        if self.widgets.ntb_preview.get_current_page() == 2:
            return
            
        if (self.widgets.ntb_preview.get_current_page() == 4 and
            keyname in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
            'comma', 'period', 'BackSpace', 'Return']):
            #user Tab, pass numbers through
            return False
            

        # take care of different key handling for lathe operation
        if self.lathe_mode:
            if keyname == "Page_Up" or keyname == "Page_Down" or keyname == "KP_Page_Up" or keyname == "KP_Page_Down":
                return

        if event.state & Gdk.ModifierType.SHIFT_MASK:  # SHIFT is hold down, fast jogging active
            fast = True
        else:
            fast = False

        if keyname == "Up" or keyname == "KP_Up":
            if self.lathe_mode:
                if self.backtool_lathe:
                    button_name = "x+"
                else:
                    button_name = "x-"
            else:
                button_name = "y+"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "Down" or keyname == "KP_Down":
            if self.lathe_mode:
                if self.backtool_lathe:
                    button_name = "x-"
                else:
                    button_name = "x+"
            else:
                button_name = "y-"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "Left" or keyname == "KP_Left":
            if self.lathe_mode:
                button_name = "z-"
            else:
                button_name = "x-"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "Right" or keyname == "KP_Right":
            if self.lathe_mode:
                button_name = "z+"
            else:
                button_name = "x+"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "Page_Up" or keyname == "KP_Page_Up":
            button_name = "z+"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "Page_Down" or keyname == "KP_Page_Down":
            button_name = "z-"
            if signal:
                self._on_btn_jog_pressed(None, button_name, fast)
            else:
                self._on_btn_jog_released(None, button_name)
        elif keyname == "I" or keyname == "i":
            if signal:
                if self.stat.state != 1:  # still moving
                    return
                # The active button name is hold in self.active_increment
                LOG.debug(self.active_increment)
                rbt = int(self.active_increment.split("_")[1])
                if keyname == "I":
                    # so lets increment it by one
                    rbt += 1
                    # we check if we are still in the allowed limit
                    if rbt > len(self.jog_increments) - 1:  # beginning from zero
                        rbt = 0
                else:  # must be "i"
                    # so lets reduce it by one
                    rbt -= 1
                    # we check if we are still in the allowed limit
                    if rbt < 0:
                        rbt = len(self.jog_increments) - 1  # beginning from zero
                # we set the corresponding button active
                self.incr_rbt_dic["rbt_{0}".format(rbt)].set_active(True)
                # and we have to update all pin and variables
                self._jog_increment_changed(self.incr_rbt_dic["rbt_{0}".format(rbt)])
        else:
            LOG.info("This key has not been implemented yet")
            LOG.info("Key {0} ({1:d}) was pressed {2} {3}".format(keyname, event.keyval, signal, self.last_key_event))
        self.last_key_event = keyname, signal
        return True

    # Notification stuff.
    def _init_notification(self):
        start_as = "rbtn_" + self.prefs.getpref("screen1", "window", str)
        #TODO probably xpos and ypos should be added to notification's properties 'x_pos' and 'y_pos*.
        # But then the behavior should be defined what should happen if the window is moved.
        xpos, ypos = self.widgets.window1.get_position()
        self.notification.set_property('x_pos', self.widgets.adj_x_pos_popup.get_value())
        self.notification.set_property('y_pos', self.widgets.adj_y_pos_popup.get_value())
        self.notification.set_property('message_width', self.widgets.adj_width_popup.get_value())
        if int(self.widgets.adj_max_messages.get_value()) != 0:
            self.notification.set_property('max_messages', self.widgets.adj_max_messages.get_value())
        self.notification.set_property('use_frames', self.widgets.chk_use_frames.get_active())
        self.notification.set_property('font', self.widgets.fontbutton_popup.get_font())
        self.notification.set_property('icon_size', 24)
        self.notification.set_property('top_to_bottom', True)
        # Append these two directories to the icon theme search path
        self.notification.set_property('icon_theme_path', ICON_THEME_DIR)
        self.notification.set_property('icon_theme_path', USER_ICON_THEME_DIR)

    def _from_internal_linear_unit(self, v, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4
        return v * lu

    def _parse_increment(self, btn_name):
        LOG.debug("parse_jogincrement")
        if self.incr_rbt_dic[btn_name] == self.incr_rbt_dic["rbt_0"]:
            jogincr = "0"
        else:
            jogincr = self.incr_rbt_dic[btn_name].get_label()

        if jogincr.endswith("mm"):
            scale = self._from_internal_linear_unit(1 / 25.4)
        elif jogincr.endswith("cm"):
            scale = self._from_internal_linear_unit(10 / 25.4)
        elif jogincr.endswith("um"):
            scale = self._from_internal_linear_unit(.001 / 25.4)
        elif jogincr.endswith("in") or jogincr.endswith("inch"):
            scale = self._from_internal_linear_unit(1.)
        elif jogincr.endswith("mil"):
            scale = self._from_internal_linear_unit(.001)
        else:
            scale = 1
        jogincr = jogincr.rstrip(" inchmuil°degr")
        if "/" in jogincr:
            p, q = jogincr.split("/")
            jogincr = float(p) / float(q)
        else:
            jogincr = float(jogincr)
        return jogincr * scale

    def show_try_errors(self):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        formatted_lines = traceback.format_exc().splitlines()
        LOG.error(_("{0}").format(formatted_lines[0]))
        traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
        LOG.debug(formatted_lines[-1])

    def _sensitize_widgets(self, widgetlist, value):
        for name in widgetlist:
            try:
                self.widgets[name].set_sensitive(value)
            except Exception as e:
                LOG.debug("No widget named {0} to sensitize".format(name))
                traceback.print_exc()

    def _update_active_gcodes(self):
        # active G-codes
        active_codes = []
        temp = []
        for code in sorted(self.stat.gcodes[1:]):
            if code == -1:
                continue
            if code % 10 == 0:
                temp.append("{0}".format(code // 10))
            else:
                temp.append("{0}.{1}".format(code // 10, code % 10))
        for num, code in enumerate(temp):
            if num == 8:
                active_codes.append("\n")
            active_codes.append("G" + code)
        self.active_gcodes = active_codes
        self.gcodes = self.stat.gcodes
        self.widgets.active_gcodes_label.set_label(" ".join(self.active_gcodes))

    def _update_active_mcodes(self):
        # M-codes
        active_codes = []
        temp = []
        for code in sorted(self.stat.mcodes[1:]):
            if code == -1:
                continue
            temp.append("{0}".format(code))
        for code in (temp):
            active_codes.append("M" + code)
        self.active_mcodes = active_codes
        self.mcodes = self.stat.mcodes
        self.widgets.active_mcodes_label.set_label(" ".join(self.active_mcodes))

    # Update the velocity labels
    def _update_vel(self):
        # self.stat.program_units will return 1 for inch, 2 for mm and 3 for cm
        real_feed = float(self.stat.settings[1] * self.stat.feedrate)
        if self.stat.program_units != 1:
            self.widgets.lbl_current_vel.set_text("{0:d}".format(int(self.stat.current_vel * 60.0 * self.faktor)))
            if "G95" in self.active_gcodes:
                feed_str = "{0:d}".format(int(self.stat.settings[1]))
                real_feed_str = "F  {0:.2f}".format(real_feed)
            else:
                feed_str = "{0:d}".format(int(self.stat.settings[1]))
                real_feed_str = "F  {0:d}".format(int(real_feed))
        else:
            self.widgets.lbl_current_vel.set_text("{0:.2f}".format(self.stat.current_vel * 60.0 * self.faktor))
            if "G95" in self.active_gcodes:
                feed_str = "{0:.4f}".format(self.stat.settings[1])
                real_feed_str = "F {0:.4f}".format(real_feed)
            else:
                feed_str = "{0:.3f}".format(self.stat.settings[1])
                real_feed_str = "F {0:.3f}".format(real_feed)

        # converting 0.0 to string brings nothing, so the string is empty
        # happens only on start up
        if not real_feed:
            real_feed_str = "F  0"

        self.widgets.lbl_active_feed.set_label(feed_str)
        self.widgets.lbl_feed_act.set_text(real_feed_str)

    def _update_coolant(self):
        if self.stat.flood:
            if not self.widgets.tbtn_flood.get_active():
                self.widgets.tbtn_flood.set_active(True)
                self.widgets.tbtn_flood.set_image(self.widgets.img_coolant_on)
        else:
            if self.widgets.tbtn_flood.get_active():
                self.widgets.tbtn_flood.set_active(False)
                self.widgets.tbtn_flood.set_image(self.widgets.img_coolant_off)
        if self.stat.mist:
            if not self.widgets.tbtn_mist.get_active():
                self.widgets.tbtn_mist.set_active(True)
                self.widgets.tbtn_mist.set_image(self.widgets.img_mist_on)
        else:
            if self.widgets.tbtn_mist.get_active():
                self.widgets.tbtn_mist.set_active(False)
                self.widgets.tbtn_mist.set_image(self.widgets.img_mist_off)

    def _update_halui_pin(self):
        if self.spindle_override != self.stat.spindle[0]['override']:
            self.initialized = False
            self.widgets.spc_spindle.set_value(self.stat.spindle[0]['override'] * 100)
            self.spindle_override = self.stat.spindle[0]['override']
            self.initialized = True
        if self.feed_override != self.stat.feedrate:
            self.initialized = False
            self.widgets.spc_feed.set_value(self.stat.feedrate * 100)
            self.feed_override = self.stat.feedrate
            self.initialized = True
        if self.rapidrate != self.stat.rapidrate:
            self.initialized = False
            self.widgets.spc_rapid.set_value(self.stat.rapidrate * 100)
            self.rapidrate = self.stat.rapidrate
            self.initialized = True

    def _update_slider(self, widgetlist):
        # update scales and sliders, this must happen if sliders shows units
        for widget in widgetlist:
            value = self.widgets[widget].get_value()
            min = self.widgets[widget].get_property("min")
            max = self.widgets[widget].get_property("max")

            self.widgets[widget].set_property("min", min * self.faktor)
            self.widgets[widget].set_property("max", max * self.faktor)
            self.widgets[widget].set_value(value * self.faktor)

        self.scale_jog_vel = self.scale_jog_vel * self.faktor

        if "spc_lin_jog_vel" in widgetlist:
            if self.widgets.tbtn_turtle_jog.get_active():
                self.turtle_jog = self.turtle_jog * self.faktor
            else:
                self.rabbit_jog = self.rabbit_jog * self.faktor

    def _change_dro_color(self, property, color):
        LOG.debug(property)
        LOG.debug("RGBA: {0} {1} {2} {3}".format(color.red, color.green, color.blue, color.alpha))
        for dro in self.dro_dic:
            self.dro_dic[dro].set_property(property, color)

# ToDo: Do w need this???
#         if self.lathe_mode:
#             # check if G7 or G8 is active
#             # this is set on purpose wrong, because we want the periodic
#             # to update the state correctly
#             if "G7" in self.active_gcodes:
#                 self.diameter_mode = False
#             else:
#                 self.diameter_mode = True

    def _update_toolinfo(self, tool):
        LOG.debug("Tool is now {0}".format(tool))
        if "G43" in self.active_gcodes:
            LOG.debug("G43 is active")
        else:
            LOG.debug("G43 is not active")

        toolinfo = self.widgets.tooledit1.get_toolinfo(tool)
        if toolinfo:
            # Doku
            # toolinfo[0] = cell toggle
            # toolinfo[1] = tool number
            # toolinfo[2] = pocket number
            # toolinfo[3] = X offset
            # toolinfo[4] = Y offset
            # toolinfo[5] = Z offset
            # toolinfo[6] = A offset
            # toolinfo[7] = B offset
            # toolinfo[8] = C offset
            # toolinfo[9] = U offset
            # toolinfo[10] = V offset
            # toolinfo[11] = W offset
            # toolinfo[12] = tool diameter
            # toolinfo[13] = frontangle
            # toolinfo[14] = backangle
            # toolinfo[15] = tool orientation
            # toolinfo[16] = tool info
            self.widgets.lbl_tool_no.set_text(str(toolinfo[1]))
            self.widgets.lbl_tool_dia.set_text(toolinfo[12])
            self.halcomp["tool-diameter"] = float(locale.atof(toolinfo[12]))
            self.widgets.lbl_tool_name.set_text(toolinfo[16])

        # we do not allow touch off with no tool mounted, so we set the
        # corresponding widgets unsensitized and set the description accordingly
        if tool <= 0:
            self.widgets.lbl_tool_no.set_text("0")
            self.widgets.lbl_tool_dia.set_text("0")
            self.widgets.lbl_tool_name.set_text(_("No tool description available"))
            self.widgets.btn_tool_touchoff_x.set_sensitive(False)
            self.widgets.btn_tool_touchoff_z.set_sensitive(False)
        else:
            self.widgets.btn_tool_touchoff_x.set_sensitive(True)
            self.widgets.btn_tool_touchoff_z.set_sensitive(True)

        if self.load_tool:
            self.load_tool = False
            self.on_hal_status_interp_idle(None)
            return

    def _set_enable_tooltips(self, value):
        LOG.debug("_set_enable_tooltips = {0}".format(value))
        # this will hide the tooltips from the glade file widgets,
        # but not from the ones we created dynamically
        for widget in self.widgets_with_tooltips:
            widget.set_has_tooltip(value)
        # the dynamically created widgets are in ordered in dictionaries
        # self.joints_button_dic (only in non trivial kinematics)
        # self.ref_button_dic
        # self.touch_button_dic
        # self.jog_button_dic
        # self.macro_dic
        dictionaries = [self.ref_button_dic, self.touch_button_dic,
                        self.jog_button_dic, self.macro_dic]
        if not self.trivial_kinematics:
            dictionaries.append(self.joints_button_dic)
        for dict in dictionaries:
            for widget in dict:
                dict[widget].set_has_tooltip(value)

# helpers functions end
# =========================================================

    def on_adj_dro_digits_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.dro_digits = int(widget.get_value())
        self.prefs.putpref("dro_digits", self.dro_digits, int)
        if self.stat.program_units != 1:
            format_string_mm = "%" + str(13 - self.dro_digits) + "." + str(self.dro_digits) + "f"
            format_string_inch = "%" + str(13 - self.dro_digits - 1) + "." + str(self.dro_digits + 1) + "f"
        else:
            format_string_inch = "%" + str(13 - self.dro_digits) + "." + str(self.dro_digits) + "f"
            format_string_mm = "%" + str(13 - self.dro_digits + 1) + "." + str(self.dro_digits - 1) + "f"

        for dro in self.dro_dic:
            self.dro_dic[dro].set_property("mm_text_template", format_string_mm)
            self.dro_dic[dro].set_property("imperial_text_template", format_string_inch)

    def on_chk_toggle_readout_toggled(self, widget, data=None):
        state = widget.get_active()
        self.prefs.putpref("toggle_readout", state)
        self.toggle_readout = state
        for dro in self.dro_dic:
            self.dro_dic[dro].set_property("toggle_readout", state)

    def _on_DRO_clicked(self, widget, joint, order):
        for dro in self.dro_dic:
            self.dro_dic[dro].set_order(order)
        return

    def _on_DRO_axis_clicked(self, widget, axisletter):
        if axisletter == "r" or axisletter == "d":
            axisletter = "x"
        self._on_btn_set_value_clicked(None, data=axisletter)

    def _offset_changed(self, pin, tooloffset):
        joint = None
        for axis in ("x", "z"):
            if axis in self.axis_list:
                joint = self._get_joint_from_joint_axis_dic(axis)
                break
            else:
                continue

        # no X or Z axis in config, so we can not apply offsets
        if joint is None:
            self.widgets.lbl_tool_offset_z.hide()
            self.widgets.lbl_tool_offset_x.hide()
            return

        dro = self.dro_dic["Combi_DRO_{0}".format(joint)]

        if dro.machine_units == _MM:
            self.widgets.lbl_tool_offset_z.set_text("{0:.3f}".format(self.halcomp["tooloffset-z"]))
            self.widgets.lbl_tool_offset_x.set_text("{0:.3f}".format(self.halcomp["tooloffset-x"]))
        else:
            self.widgets.lbl_tool_offset_z.set_text("{0:.4f}".format(self.halcomp["tooloffset-z"]))
            self.widgets.lbl_tool_offset_x.set_text("{0:.4f}".format(self.halcomp["tooloffset-x"]))

    def on_offsetpage1_selection_changed(self, widget, system, name):
        if system not in self.system_list[1:] or self.touch_button_dic["edit_offsets"].get_active():
            self.touch_button_dic["set_active"].set_sensitive(False)
        else:
            self.touch_button_dic["set_active"].set_sensitive(True)

    def on_adj_x_pos_popup_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.prefs.putpref("x_pos_popup", widget.get_value(), float)
        self._init_notification()

    def on_adj_y_pos_popup_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.prefs.putpref("y_pos_popup", widget.get_value(), float)
        self._init_notification()

    def on_adj_width_popup_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.prefs.putpref("width_popup", widget.get_value(), float)
        self._init_notification()

    def on_adj_max_messages_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.prefs.putpref("max_messages", widget.get_value(), float)
        self._init_notification()

    def on_chk_use_frames_toggled(self, widget, data=None):
        if not self.initialized:
            return
        self.prefs.putpref("use_frames", widget.get_active())
        self._init_notification()

    def on_fontbutton_popup_font_set(self, font):
        self.prefs.putpref("message_font", self.widgets.fontbutton_popup.get_font())
        self._init_notification()

    def on_btn_launch_test_message_pressed(self, widget=None, data=None):
        index = len(self.notification.messages)
        text = _("Halo, welcome to the test message {0}").format(index)
        self._show_error((13, text))

    def on_chk_turtle_jog_toggled(self, widget, data=None):
        state = widget.get_active()
        self.prefs.putpref("hide_turtle_jog_button", state)
        self.widgets.grid_turtle_jog_factor.set_sensitive(not state)
        if state:
            self.widgets.tbtn_turtle_jog.hide()
        else:
            self.widgets.tbtn_turtle_jog.show()
            self.turtle_jog_factor = self.prefs.getpref('turtle_jog_factor', 20, int)
            self.widgets.adj_turtle_jog_factor.configure(self.turtle_jog_factor, 1,
                                                         100, 1, 0, 0)
            self.turtle_jog = self.jog_rate_max / self.turtle_jog_factor

    def on_adj_turtle_jog_factor_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        self.turtle_jog_factor = int(widget.get_value())
        self.prefs.putpref("turtle_jog_factor", self.turtle_jog_factor, int)
        self.turtle_jog = self.rabbit_jog / self.turtle_jog_factor
        if self.widgets.tbtn_turtle_jog.get_active():
            self.widgets.spc_lin_jog_vel.set_property("min", 0)
            self.widgets.spc_lin_jog_vel.set_property("max", self.jog_rate_max / self.turtle_jog_factor)
            self.widgets.spc_lin_jog_vel.set_value(self.turtle_jog)

    def on_tbtn_turtle_jog_toggled( self, widget, data = None ):
        # due to imperial and metric options we have to get first the values of the widget
        max = self.widgets.spc_lin_jog_vel.max
        min = self.widgets.spc_lin_jog_vel.min
        value = self.widgets.spc_lin_jog_vel.get_value()

        if widget.get_active():
            self.rabbit_jog = value
            widget.set_image( self.widgets.img_turtle_jog )
            self.widgets.spc_lin_jog_vel.set_property("min", min)
            self.widgets.spc_lin_jog_vel.set_property("max", max / self.turtle_jog_factor)
            self.widgets.spc_lin_jog_vel.set_value(self.turtle_jog)
        else:
            self.turtle_jog = value
            widget.set_image( self.widgets.img_rabbit_jog )
            self.widgets.spc_lin_jog_vel.set_property("min", min)
            self.widgets.spc_lin_jog_vel.set_property("max", max * self.turtle_jog_factor)
            self.widgets.spc_lin_jog_vel.set_value(self.rabbit_jog)

    def _on_pin_turtle_jog(self, pin):
        self.widgets.tbtn_turtle_jog.set_active(pin.get())

    # use the current loaded file to be loaded on start up
    def on_btn_use_current_clicked(self, widget, data=None):
        if self.stat.file:
            self.widgets.file_to_load_chooser.set_filename(self.stat.file)
            self.prefs.putpref("open_file", self.stat.file)

    # Clear the status to load a file on start up, so there will not be loaded a program
    # on the next start of the GUI
    def on_btn_none_clicked(self, widget, data=None):
        self.widgets.file_to_load_chooser.set_filename(" ")
        self.prefs.putpref("open_file", " ")

    def on_ntb_main_switch_page(self, widget, page, page_num, data=None):
        if self.widgets.tbtn_setup.get_active():
            if page_num != 1:  # setup page is active,
                self.widgets.tbtn_setup.set_active(False)

    def on_rbt_manual_toggled(self, widget):
        widget.set_image(self.widgets["img_manual_on" if widget.get_active() else "img_manual"])

    def on_rbt_mdi_toggled(self, widget):
        widget.set_image(self.widgets["img_mdi_on" if widget.get_active() else "img_mdi"])

    def on_rbt_auto_toggled(self, widget):
        widget.set_image(self.widgets["img_auto_on" if widget.get_active() else "img_auto"])

    def on_tbtn_setup_toggled(self, widget, data=None):
        # first we set to manual mode, as we do not allow changing settings in other modes
        # otherwise external halui commands could start a program while we are in settings
        self.stat.poll()
        if widget.get_active():
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self.widgets.rbt_manual.set_active(True)
            # save the mode to restore when leaving the settings page
            self.last_mode = self.stat.task_mode
            self.user_tab_enabled = self.widgets.tbtn_user_tabs.get_sensitive()
            # deactivate the mode buttons, so changing modes is not possible while we are in settings mode
            self.widgets.rbt_manual.set_sensitive(False)
            self.widgets.rbt_mdi.set_sensitive(False)
            self.widgets.rbt_auto.set_sensitive(False)
            self.widgets.tbtn_user_tabs.set_sensitive(False)
            code = False
            # here the user don"t want an unlock code
            if self.widgets.rbt_no_unlock.get_active():
                code = True
            # if hal pin is true, we are allowed to enter settings, this may be
            # realized using a key switch
            if self.widgets.rbt_hal_unlock.get_active() and self.halcomp["unlock-settings"]:
                code = True
            # else we ask for the code using the system.dialog
            if self.widgets.rbt_use_unlock.get_active():
                if self.dialogs.system_dialog(self):
                    code = True
            # Lets see if the user has the right to enter settings
            if code:
                self.widgets.ntb_main.set_current_page(1)
                self.widgets.ntb_setup.set_current_page(0)
                self.widgets.ntb_button.set_current_page(_BB_SETUP)
                widget.set_image(self.widgets.img_settings_on)
            else:
                if self.widgets.rbt_hal_unlock.get_active():
                    message = _("Hal Pin is low, Access denied")
                else:
                    message = _("wrong code entered, Access denied")
                self.dialogs.warning_dialog(self, _("Just to warn you"), message)
                self.widgets.tbtn_setup.set_active(False)
                widget.set_image(self.widgets.img_settings)
        else:
            # check which button should be sensitive, depending on the state of the machine
            if self.stat.task_state == linuxcnc.STATE_ESTOP:
                # estopped no mode available
                self.widgets.rbt_manual.set_sensitive(False)
                self.widgets.rbt_mdi.set_sensitive(False)
                self.widgets.rbt_auto.set_sensitive(False)
            if (self.stat.task_state == linuxcnc.STATE_ON) and not self.all_homed:
                # machine on, but not homed, only manual allowed
                self.widgets.rbt_manual.set_sensitive(True)
                self.widgets.rbt_mdi.set_sensitive(False)
                self.widgets.rbt_auto.set_sensitive(False)
            if (self.stat.task_state == linuxcnc.STATE_ON) and (self.all_homed or self.no_force_homing):
                # all OK, make all modes available
                self.widgets.rbt_manual.set_sensitive(True)
                self.widgets.rbt_mdi.set_sensitive(True)
                self.widgets.rbt_auto.set_sensitive(True)

            if self.user_tab_enabled:
                self.widgets.tbtn_user_tabs.set_sensitive(True)
            # if user tab was open before, switch back
            if self.widgets.tbtn_user_tabs.get_active():
                self.widgets.ntb_main.set_current_page(2)
            # if task_mode didn't change, enable tab "Manual Mode"
            elif self.last_mode == self.stat.task_mode:
                # this is needed here, because we do not
                # change mode, so on_hal_status_manual will not be called
                self.widgets.ntb_main.set_current_page(0)
                self.widgets.ntb_button.set_current_page(_BB_MANUAL)
                self.widgets.ntb_info.set_current_page(0)
                self.widgets.ntb_jog.set_current_page(0)
            else:
                # restore mode
                self.command.mode(self.last_mode)
                self.command.wait_complete()

            widget.set_image(self.widgets.img_settings)

    # Show or hide the user tabs
    def on_tbtn_user_tabs_toggled(self, widget, data=None):
        if widget.get_active():
            self.widgets.ntb_main.set_current_page(2)
            self.widgets.tbtn_fullsize_preview0.set_sensitive(False)
            widget.set_image(self.widgets.img_user_tabs_on)
        else:
            self.widgets.ntb_main.set_current_page(0)
            self.widgets.tbtn_fullsize_preview0.set_sensitive(True)
            widget.set_image(self.widgets.img_user_tabs)

# =========================================================
# The homing functions
    def on_btn_homing_clicked(self, widget, data=None):
        self.widgets.ntb_button.set_current_page(_BB_HOME)

    def on_btn_home_all_clicked(self, widget, data=None):
        self._set_motion_mode(0)
        # home -1 means all
        self.command.home(-1)

    def _on_btn_unhome_clicked(self, widget):
        self._set_motion_mode(0)
        # -1 for all
        self.command.unhome(-1)

    def _on_btn_home_back_clicked(self, widget):
        self.widgets.ntb_button.set_current_page(_BB_MANUAL)
        self.widgets.ntb_main.set_current_page(0)
        self.widgets.ntb_preview.set_current_page(0)

    def _on_btn_home_clicked(self, widget):
        # home axis or joint?
        LOG.debug("on button home clicked = {0}".format(widget.get_property("name")))
        if "axis" in widget.get_property("name"):
            value = widget.get_property("name")[-1]
            # now get the joint from directory by the value
            joint_or_axis = self._get_joint_from_joint_axis_dic(value)
        elif "joint" in widget.get_property("name"):
            joint_or_axis = int(widget.get_property("name")[-1])
        elif "all" in widget.get_property("name"):
            joint_or_axis = -1

        self._set_motion_mode(0)
        self.command.home(joint_or_axis)

    def _unhome_signal(self, object, joint):
        self._set_motion_mode(0)
        self.all_homed = False
        # -1 for all
        self.command.unhome(joint)

    def _set_motion_mode(self, state):
        # 1:teleop, 0: joint
        self.command.teleop_enable(state)
        self.command.wait_complete()

# The homing functions
# =========================================================

    def _ignore_limits(self, pin):
        self.widgets.chk_ignore_limits.set_active(pin.get())

    def on_chk_ignore_limits_toggled(self, widget, data=None):
        if self.widgets.chk_ignore_limits.get_active():
           self.command.override_limits()

    def on_tbtn_fullsize_preview_toggled(self, widget, data=None):
        state = widget.get_active()

        # "synchronize" all fullscreen toggle buttons
        for tbtn_fullscreen, num in [(self.widgets[f"tbtn_fullsize_preview{n}"], n) for n in range(2)]:
            if tbtn_fullscreen.get_active() is not state:
                with(tbtn_fullscreen.handler_block(icon_theme_helper.find_handler_id_by_signal(tbtn_fullscreen, "toggled"))):
                    tbtn_fullscreen.set_active(state)
            tbtn_fullscreen.set_image(self.widgets[f"img_fullsize_preview{num}_" + ("close" if state else "open")])

        if state:
            self.widgets.box_info.hide()
            self.widgets.vbx_jog.hide()
            dro = self.dro_dic[list(self.dro_dic.keys())[0]]
            self.widgets.gremlin.set_property("metric_units", dro.metric_units)
            self.widgets.gremlin.set_property("enable_dro", True)
            if self.lathe_mode:
                self.widgets.gremlin.set_property("show_lathe_radius", not self.diameter_mode)
        else:
            self.widgets.box_info.show()
            self.widgets.vbx_jog.show()
            if not self.widgets.chk_show_dro.get_active():
                self.widgets.gremlin.set_property("enable_dro", self.enable_gremlin_dro)

# =========================================================
# this are hal-tools copied from gsreen function
    def on_btn_show_hal_clicked(self, widget, data=None):
        p = os.popen("tclsh {0}/bin/halshow.tcl &".format(TCLPATH))

    def on_btn_calibration_clicked(self, widget, data=None):
        p = os.popen("tclsh {0}/bin/emccalib.tcl -- -ini {1} > /dev/null &".format(TCLPATH, sys.argv[2]), "w")

    def on_btn_hal_meter_clicked(self, widget, data=None):
        p = os.popen("halmeter &")

    def on_btn_status_clicked(self, widget, data=None):
        p = os.popen("linuxcnctop  > /dev/null &", "w")

    def on_btn_hal_scope_clicked(self, widget, data=None):
        p = os.popen("halscope  > /dev/null &", "w")

    def on_btn_classicladder_clicked(self, widget, data=None):
        if hal.component_exists("classicladder_rt"):
            p = os.popen("classicladder  &", "w")
        else:
            self.dialogs.warning_dialog(self, _("INFO:"),
                                   _("Classicladder real-time component not detected"))

# =========================================================
# spindle stuff

    def _update_spindle(self):
        if self.stat.spindle[0]['direction'] > 0:
            self.widgets.rbt_forward.set_active(True)
        elif self.stat.spindle[0]['direction'] < 0:
            self.widgets.rbt_reverse.set_active(True)
        elif not self.widgets.rbt_stop.get_active():
            self.widgets.rbt_stop.set_active(True)

        # this is needed, because otherwise a command S0 would not set active btn_stop
        if not abs(self.stat.spindle[0]['speed']):
            self.widgets.rbt_stop.set_active(True)
            return

        # set the speed label in active code frame
        if self.stat.spindle[0]['speed'] == 0:
            speed = self.stat.settings[2]
        else:
            speed = self.stat.spindle[0]['speed']
        self.widgets.active_speed_label.set_label("{0:.0f}".format(abs(speed)))
        self.widgets.lbl_spindle_act.set_text("S {0}".format(int(round(speed * self.spindle_override))))

    def _update_vc(self):
        if self.stat.spindle[0]['direction'] != 0:
            if self.stat.spindle[0]['speed'] == 0:
                speed = self.stat.settings[2]
            else:
                speed = self.stat.spindle[0]['speed']

            if not self.lathe_mode:
                diameter = self.halcomp["tool-diameter"]
            else:
                diameter = int(self.dro_dic["Combi_DRO_0"].get_position()[1] * 2)
                speed = self.widgets.spindle_feedback_bar.value
            vc = abs(int(speed * self.spindle_override) * diameter * 3.14 / 1000)
        else:
            vc = 0
        if vc >= 100:
            text = "Vc= {0:d}".format(int(vc))
        elif vc >= 10:
            text = "Vc= {0:2.1f}".format(vc)
        else:
            text = "Vc= {0:.2f}".format(vc)
        self.widgets.lbl_vc.set_text(text)
        
    def seconds_to_hms(self, seconds):
        seconds = int(seconds)
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        seconds = seconds % 60
        if hours < 1:
            if minutes < 1:
                return f"{seconds:02} s"
            else:
                return f"{minutes:02}:{seconds:02} min"
        else:
            return f"{hours:02}:{minutes:02}:{seconds:02} h"

    # This extracts the time from a string like "104 Seconds" or "2.4 Minutes"
    def parse_time_string(self, time_string):
        if "Minutes" in time_string:
            return self.seconds_to_hms(float(time_string.split("Minutes")[0])*60)
        elif "Seconds" in time_string:
            return self.seconds_to_hms(time_string.split("Seconds")[0])
        else:
            return ""
        
    def on_gcode_properties(self, widget, data):
        # print("G-code properties:", data)
        self.widgets.lbl_gcode_size.set_text(data['size'])
        self.widgets.lbl_gcode_g0.set_text(data['g0'])
        self.widgets.lbl_gcode_g1.set_text(data['g1'])
        self.widgets.lbl_gcode_run.set_text(f"{self.parse_time_string(data['run'])}")
        # self.widgets.lbl_gcode_toollist.set_text(data['toollist'])
        self.widgets.lbl_gcode_x.set_text(data['x'])
        self.widgets.lbl_gcode_y.set_text(data['y'])
        self.widgets.lbl_gcode_z.set_text(data['z'])
        # alternative way to calculate program length
        # self.halcomp["program.length"] = int(data['size'].split("\n")[1].split(" ")[0] )
    
    def on_ntb_tool_code_info_switch_page(self, widget, page, page_num):
        self.prefs.putpref("info_tab_page", page_num, int)

    def on_rbt_forward_clicked(self, widget, data=None):
        if widget.get_active():
            widget.set_image(self.widgets.img_spindle_forward_on)
            self._set_spindle("forward")
        else:
            widget.set_image(self.widgets.img_spindle_forward)
        # Toggling the sensitive property is important here! See the commit description.
        widget.set_sensitive(not widget.get_sensitive())
        widget.set_sensitive(not widget.get_sensitive())

    def on_rbt_reverse_clicked(self, widget, data=None):
        if widget.get_active():
            widget.set_image(self.widgets.img_spindle_reverse_on)
            self._set_spindle("reverse")
        else:
            widget.set_image(self.widgets.img_spindle_reverse)
        # Toggling the sensitive property is important here! See the commit description.
        widget.set_sensitive(not widget.get_sensitive())
        widget.set_sensitive(not widget.get_sensitive())

    def on_rbt_stop_clicked(self, widget, data=None):
        if widget.get_active():
            widget.set_image(self.widgets.img_spindle_stop_on)
            self._set_spindle("stop")
        else:
            widget.set_image(self.widgets.img_spindle_stop)
        # Toggling the sensitive property is important here! See the commit description.
        widget.set_sensitive(not widget.get_sensitive())
        widget.set_sensitive(not widget.get_sensitive())

    def _set_spindle(self, command):
        # if we are in estop state, we will have to leave here, otherwise
        # we get an error, that switching spindle off is not allowed with estop
        if self.stat.task_state == linuxcnc.STATE_ESTOP:
            return

        # if we do not check this, we will get an error in auto mode and sub
        # calls from MDI containing i.e. G96 would not run, as the speed will
        # be set to the commanded value due to the next code part
        if self.stat.task_mode != linuxcnc.MODE_MANUAL:
            if self.stat.interp_state == linuxcnc.INTERP_READING or self.stat.interp_state == linuxcnc.INTERP_WAITING:
                return

        rpm = self._check_spindle_range()
        # as the commanded value will be multiplied with speed override,
        # we take care of that but we have to check for speed override
        # to not be zero to avoid division by zero error
        try:
            rpm_out = rpm / self.stat.spindle[0]['override']
        except:
            rpm_out = 0
        self.widgets.lbl_spindle_act.set_label("S {0}".format(int(rpm)))

        if command == "stop":
            # documentation of self.command.spindle()
            # linuxcnc.spindle(direction, speed, spindle=0)
            self.command.spindle(0)
            self.widgets.lbl_spindle_act.set_label("S 0")
        elif command == "forward":
            self.command.spindle(1, rpm_out)
        elif command == "reverse":
            self.command.spindle(-1, rpm_out)
        else:
            LOG.debug("Something went wrong, we have an unknown spindle widget {0}".format(command))

    def _check_spindle_range(self):
        rpm = (self.stat.settings[2])
        if rpm == 0:
            rpm = self.spindle_start_rpm

        spindle_override = self.widgets.spc_spindle.get_value() / 100
        real_spindle_speed = rpm * spindle_override

        if real_spindle_speed > self.max_spindle_rev:
            real_spindle_speed = self.max_spindle_rev
        elif real_spindle_speed < self.min_spindle_rev:
            real_spindle_speed = self.min_spindle_rev
        return real_spindle_speed

    def on_btn_spindle_100_clicked(self, widget, data=None):
        self.widgets.spc_spindle.set_value(100)

    def on_spc_spindle_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        # this is in a try except, because on initializing the window the values are still zero
        # so we would get an division / zero error
        spindle_speed_out = 0
        widget_value = widget.get_value()
        try:
            # get the current spindle speed
            if not abs(self.stat.settings[2]):
                if self.widgets.rbt_forward.get_active() or self.widgets.rbt_reverse.get_active():
                    speed = self.stat.spindle[0]['speed']
                else:
                    speed = 0
            else:
                speed = abs(self.stat.spindle[0]['speed'])
            spindle_override_in = widget_value / 100
            spindle_speed_out = speed * spindle_override_in

            # back calculation of override value if speed limit is hit
            if abs(spindle_speed_out) > self.max_spindle_rev:
                spindle_override_command = spindle_override_in * self.max_spindle_rev / abs(spindle_speed_out)
            elif abs(spindle_speed_out) < self.min_spindle_rev:
                spindle_override_command = spindle_override_in * self.min_spindle_rev / abs(spindle_speed_out)
            else:
                spindle_override_command = spindle_override_in

            widget.set_value(spindle_override_command * 100.0)
            self.spindle_override_in = spindle_override_command
            self.command.spindleoverride(spindle_override_command)
        except:
            pass

    def on_adj_start_spindle_RPM_value_changed(self, widget, data=None):
        self.spindle_start_rpm = widget.get_value()
        self.prefs.putpref("spindle_start_rpm", self.spindle_start_rpm, float)

    def on_adj_spindle_bar_min_value_changed(self, widget, data=None):
        self.min_spindle_rev = widget.get_value()
        self.prefs.putpref("spindle_bar_min", self.min_spindle_rev, float)
        self.widgets.spindle_feedback_bar.set_property("min", self.min_spindle_rev)

    def on_adj_spindle_bar_max_value_changed(self, widget, data=None):
        self.max_spindle_rev = widget.get_value()
        self.prefs.putpref("spindle_bar_max", self.max_spindle_rev, float)
        self.widgets.spindle_feedback_bar.set_property("max", self.max_spindle_rev)

# =========================================================
# Coolant an mist coolant button
    def on_tbtn_flood_toggled(self, widget, data=None):
        if self.stat.flood and self.widgets.tbtn_flood.get_active():
            return
        elif not self.stat.flood and not self.widgets.tbtn_flood.get_active():
            return
        elif self.widgets.tbtn_flood.get_active():
            self.widgets.tbtn_flood.set_image(self.widgets.img_coolant_on)
            self.command.flood(linuxcnc.FLOOD_ON)
        else:
            self.widgets.tbtn_flood.set_image(self.widgets.img_coolant_off)
            self.command.flood(linuxcnc.FLOOD_OFF)

    def on_tbtn_mist_toggled(self, widget, data=None):
        if self.stat.mist and self.widgets.tbtn_mist.get_active():
            return
        elif not self.stat.mist and not self.widgets.tbtn_mist.get_active():
            return
        elif self.widgets.tbtn_mist.get_active():
            self.widgets.tbtn_mist.set_image(self.widgets.img_mist_on)
            self.command.mist(linuxcnc.MIST_ON)
        else:
            self.widgets.tbtn_mist.set_image(self.widgets.img_mist_off)
            self.command.mist(linuxcnc.MIST_OFF)

# =========================================================
# feed stuff
    def on_spc_feed_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = widget.get_value() / 100
        self.feed_override = value
        self.command.feedrate(value)

    def on_btn_feed_100_clicked(self, widget, data=None):
        self.widgets.spc_feed.set_value(100)

    def on_spc_rapid_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = widget.get_value() / 100
        self.rapidrate = value
        self.command.rapidrate(value)

    # this are the MDI thinks we need
    def on_btn_delete_clicked(self, widget, data=None):
        message = _("Do you really want to delete the MDI history?\n")
        message += _("This will not delete the MDI History file, but will\n")
        message += _("delete the listbox entries for this session.")
        result = self.dialogs.yesno_dialog(self, message, _("Attention!!"))
        if result:
            self.widgets.hal_mdihistory.model.clear()

    def on_btn_show_kbd_clicked(self, widget):
        #print("show Keyboard clicked", self.widgets.key_box.get_children())
        #print(widget)
        #print(widget.name)
        #print(widget.get_children()[0])
        #print(widget.get_children()[0].get_property("file"))

        # special case if we are in mdi mode
        if self.widgets.ntb_button.get_current_page() == _BB_MDI and self.stat.interp_state != linuxcnc.INTERP_IDLE:
            self.command.abort()
            self.command.wait_complete()
            for pos in self.macro_dic:
                self.macro_dic[pos].set_sensitive(True)
            if self.onboard:
                self._change_kbd_image("img_macro_menu_keyboard")
                #self.socket.show_all()  # This is needed, because after a rezise the keyboard is not visible for unknown reasons
            else:
                self.macro_dic["keyboard"].set_sensitive(False)
        elif self.widgets.ntb_info.get_current_page() == 1:
            self.widgets.ntb_info.set_current_page(0)
        else:
            self.widgets.ntb_info.set_current_page(1)

        # special case if we are in edit mode
        if self.widgets.ntb_button.get_current_page() == _BB_EDIT:
            if self.widgets.ntb_info.get_visible():
                self.widgets.ntb_info.hide()
            else:
                self.widgets.ntb_info.show()

    def on_ntb_info_switch_page(self, widget, page, page_num, data=None):
        if self.stat.task_mode == linuxcnc.MODE_MDI:
            self.widgets.hal_mdihistory.entry.grab_focus()
        elif self.stat.task_mode == linuxcnc.MODE_AUTO:
            self.widgets.gcode_view.grab_focus()
        # Change keyboard icon on keyboard-show buttons
        shown = page_num == 1
        self.widgets.btn_keyb.set_image(
            self.widgets["img_edit_menu_keyboard_hide" if shown else "img_edit_menu_keyboard"]
        )
        if self.onboard:
            self.widgets.btn_macro_menu_toggle_keyboard.set_image(
                self.widgets["img_macro_menu_keyboard_hide" if shown else "img_macro_menu_keyboard"]
            )

    # Three back buttons to be able to leave notebook pages
    # All use the same callback offset
    def on_btn_back_clicked(self, widget, data=None):
        if self.widgets.ntb_button.get_current_page() == _BB_EDIT:  # edit mode, go back to auto_buttons
            if self.file_changed:
                message = _("Exit and discard changes?")
                result = self.dialogs.yesno_dialog(self, message, _("Attention!"))
                if not result: # user says no, he want to save
                    return
            self.widgets.ntb_button.set_current_page(_BB_AUTO)
            if self.widgets.tbtn_fullsize_preview0.get_active():
                self.widgets.vbx_jog.set_visible(False)
        elif self.widgets.ntb_button.get_current_page() == _BB_LOAD_FILE:  # File selection mode
            self.widgets.ntb_button.set_current_page(_BB_AUTO)
            self.widgets.tbtn_fullsize_preview0.set_active(False)
            self.on_tbtn_fullsize_preview_toggled(self.widgets.tbtn_fullsize_preview0)
        else:  # else we go to main button on manual
            self.widgets.ntb_button.set_current_page(_BB_MANUAL)
            self.widgets.ntb_main.set_current_page(0)
            self.widgets.ntb_preview.set_current_page(0)
        # re-enable mode buttons, when returning from edit or file selection
        self.widgets.rbt_manual.set_sensitive(True)
        self.widgets.rbt_mdi.set_sensitive(True)
        self.widgets.tbtn_setup.set_sensitive(True)

    # The offset settings, set to zero
    def on_btn_touch_clicked(self, widget, data=None):
        self.widgets.ntb_button.set_current_page(_BB_TOUCH_OFF)
        self._show_offset_tab(True)
        if self.widgets.rbtn_show_preview.get_active():
            self.widgets.ntb_preview.set_current_page(0)

    def on_tbtn_edit_offsets_toggled(self, widget, data=None):
        state = widget.get_active()
        self.widgets.offsetpage1.edit_button.set_active(state)

        if self.widgets.rbtn_show_preview.get_active() and not state:
            self.widgets.ntb_preview.set_current_page(0)
        else:
            self.widgets.ntb_preview.set_current_page(1)

        # de-sensitize other buttons
        widgetlist = ["ntb_jog", "rbt_mdi","rbt_auto","tbtn_setup"]
        if self.user_tabs_enabled:
            widgetlist.append("tbtn_user_tabs")
        self._sensitize_widgets( widgetlist, not state )
        for element in self.touch_button_dic:
            if self.touch_button_dic[element].get_property("name") in ["edit_offsets", "touch_back"]:
                continue
            self.touch_button_dic[element].set_sensitive(not state)

        # if no system is selected, set the "set active" button not sensitive
        system, name = self.widgets.offsetpage1.get_selected()
        if not system:
            self.touch_button_dic["set_active"].set_sensitive(False)

        if not state:  # we must switch back to manual mode, otherwise jogging is not possible
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()

        # show virtual keyboard?
        if state and self.widgets.chk_use_kb_on_offset.get_active():
            self.widgets.ntb_info.set_current_page(1)
            self.widgets.ntb_preview.set_current_page(1)

    def on_btn_zero_g92_clicked(self, widget, data=None):
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi("G92.1")
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.widgets.btn_touch.emit("clicked")

    def _on_btn_set_value_clicked(self, widget, data=None):
        if not self.stat.task_state == linuxcnc.STATE_ON or not (self.all_homed or self.no_force_homing):
            return

        if widget:
            axis = widget.get_property("name")[-1]
        else:
            axis = data

        LOG.debug("touch button clicked {0}".format(axis.upper()))

        if self.lathe_mode and axis =="x":
            if self.diameter_mode:
                preset = self.prefs.getpref("diameter offset_axis_{0}".format(axis), 0, float)
                offset = self.dialogs.entry_dialog(self, data=preset, header=_("Enter value for diameter"),
                                                   label=_("Set diameter to:"), integer=False)
            else:
                preset = self.prefs.getpref("radius offset_axis_{0}".format(axis), 0, float)
                offset = self.dialogs.entry_dialog(self, data=preset, header=_("Enter value for radius"),
                                                   label=_("Set radius to:"), integer=False)
        else:
            preset = self.prefs.getpref("offset_axis_{0}".format(axis), 0, float)
            offset = self.dialogs.entry_dialog(self, data=preset, header=_("Enter value for axis {0}").format(axis.upper()),
                                               label=_("Set axis {0} to:").format(axis.upper()), integer=False)
        if offset == "CANCEL":
            return
        elif offset == "ERROR":
            LOG.debug("Conversion error in btn_set_value")
            self.dialogs.warning_dialog(self, _("Conversion error in btn_set_value!"),
                                   _("Please enter only numerical values. Values have not been applied"))
        else:
            self.command.mode(linuxcnc.MODE_MDI)
            self.command.wait_complete()
            command = "G10 L20 P0 {0}{1:f}".format(axis, offset)
            self.command.mdi(command)
            self.widgets.hal_action_reload.emit("activate")
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self.prefs.putpref("offset_axis_{0}".format(axis), offset, float)

    def _on_btn_set_selected_clicked(self, widget, data=None):
        system, name = self.widgets.offsetpage1.get_selected()
        if not system:
            message = _("You did not select a system to be changed to, so nothing will be changed")
            self.dialogs.warning_dialog(self, _("Important Warning!"), message)
            return
        if system == self.system_list[self.stat.g5x_index]:
            return
        else:
            self.command.mode(linuxcnc.MODE_MDI)
            self.command.wait_complete()
            self.command.mdi(system)
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()

    def on_spbtn_probe_height_value_changed(self, widget, data=None):
        self.halcomp["probeheight"] = widget.get_value()
        self.prefs.putpref("probeheight", widget.get_value(), float)

    def on_spbtn_search_vel_value_changed(self, widget, data=None):
        self.halcomp["searchvel"] = widget.get_value()
        self.prefs.putpref("searchvel", widget.get_value(), float)

    def on_spbtn_probe_vel_value_changed(self, widget, data=None):
        self.halcomp["probevel"] = widget.get_value()
        self.prefs.putpref("probevel", widget.get_value(), float)

    def on_chk_use_tool_measurement_toggled(self, widget, data=None):
        if widget.get_active():
            self.widgets.eb_blockheight_label.show()
            self.widgets.frm_probe_pos.set_sensitive(True)
            self.widgets.frm_probe_vel.set_sensitive(True)
            self.halcomp["toolmeasurement"] = True
            self.halcomp["searchvel"] = self.widgets.spbtn_search_vel.get_value()
            self.halcomp["probevel"] = self.widgets.spbtn_probe_vel.get_value()
            self.halcomp["probeheight"] = self.widgets.spbtn_probe_height.get_value()
        else:
            self.widgets.eb_blockheight_label.hide()
            self.widgets.frm_probe_pos.set_sensitive(False)
            self.widgets.frm_probe_vel.set_sensitive(False)
            self.halcomp["toolmeasurement"] = False
            self.halcomp["searchvel"] = 0.0
            self.halcomp["probevel"] = 0.0
            self.halcomp["probeheight"] = 0.0
        self.prefs.putpref("use_toolmeasurement", widget.get_active())

    def on_chk_reload_tool_toggled(self, widget, data=None):
        state = widget.get_active()
        self.reload_tool_enabled = state
        self.prefs.putpref("reload_tool", state)

    def on_btn_block_height_clicked(self, widget, data=None):
        probeheight = self.widgets.spbtn_probe_height.get_value()
        preset = self.prefs.getpref("blockheight", 0.0, float)
        blockheight = self.dialogs.entry_dialog(self, data=preset, header=_("Enter the block height"),
                                                label=_("Block height measured from base table"), integer=False)

        if blockheight == "CANCEL" or blockheight == "ERROR":
            return
        if blockheight != False or blockheight == 0:
            self.halcomp["blockheight"] = blockheight
            self.halcomp["probeheight"] = probeheight
            self.prefs.putpref("blockheight", blockheight, float)
            self.prefs.putpref("probeheight", probeheight, float)
        else:
            self.prefs.putpref("blockheight", 0.0, float)
            self.prefs.putpref("probeheight", 0.0, float)
            self.dialogs.warning_dialog(self, _("Conversion error in btn_block_height!"),
                                        _("Please enter only numerical values\nValues have not been applied"))

        # set coordinate system to new origin
        origin = self.get_ini_info.get_axis_2_min_limit() + blockheight
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi("G10 L2 P0 Z{0}".format(origin))
        self.widgets.hal_action_reload.emit("activate")
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()

    # choose a theme to apply
    def on_theme_choice_changed(self, widget):
        active = widget.get_active_iter()
        if active is None:
            return
        theme = widget.get_model()[active][0]
        self.prefs.putpref("gtk_theme", theme)
        if theme == "Follow System Theme":
            theme = self.default_theme
        settings = Gtk.Settings.get_default()
        settings.set_property("gtk-theme-name", theme)

    def _set_icon_theme(self, name):
        LOG.debug(f"Setting icon theme '{name}'")
        if name is None or name == "none":
            # Switching to none required a restart (skip entire icon theme stuff)
            message = "Change to no icon theme requires a restart to take effect."
            self.dialogs.warning_dialog(self, _("Just to warn you"), message)
        else:
            self.icon_theme.set_custom_theme(name)
            self.notification.set_property('icon_theme_name', name)
            icon_configs = [
                # widget, named_icon, size
                ("img_emergency", "main_switch_on", 48),
                ("img_emergency_off", "main_switch_off", 48),
                ("img_machine_on", "power_on", 48),
                ("img_machine_off", "power_off", 48),
                # mode buttons
                ("img_manual", "mode_manual_inactive", 48),
                ("img_manual_on", "mode_manual_active", 48),
                ("img_mdi", "mode_mdi_inactive", 48),
                ("img_mdi_on", "mode_mdi_active", 48),
                ("img_auto", "mode_auto_inactive", 48),
                ("img_auto_on", "mode_auto_active", 48),
                ("img_settings", "mode_settings_inactive", 48),
                ("img_settings_on", "mode_settings_active", 48),
                ("img_user_tabs", "mode_user_tabs_inactive", 48),
                ("img_user_tabs_on", "mode_user_tabs_active", 48),
                # gremlin controls
                ("img_view_p", "tool_axis_p", 24),
                ("img_view_x", "tool_axis_x", 24),
                ("img_view_y", "tool_axis_y", 24),
                ("img_view_y2", "tool_axis_y_inv", 24),
                ("img_view_z", "tool_axis_z", 24),
                ("img_zoom_in", "zoom_in", 24),
                ("img_zoom_out", "zoom_out", 24),
                ("img_tool_clear", "clear", 24),
                ("img_tool_path", "toolpath", 24),
                ("img_dimensions", "dimensions", 24),
                # coolant
                ("img_coolant_on",  "coolant_flood_active",   48),
                ("img_coolant_off", "coolant_flood_inactive", 48),
                ("img_mist_on",     "coolant_mist_active",    48),
                ("img_mist_off",    "coolant_mist_inactive",  48),
                # spindle
                ("img_spindle_forward", "spindle_right", 48),
                ("img_spindle_forward_on", "spindle_right_on", 48),
                ("img_spindle_reverse", "spindle_left", 48),
                ("img_spindle_reverse_on", "spindle_left_on", 48),
                ("img_spindle_stop", "spindle_stop", 48),
                ("img_spindle_stop_on", "spindle_stop_on", 48),
                # jog
                ("img_rabbit_jog", "jog_speed_fast", 32),
                ("img_turtle_jog", "jog_speed_slow", 32),
                # fullscreen
                ("img_fullsize_preview0_open",  "fullscreen_open",  48),
                ("img_fullsize_preview0_close", "fullscreen_close", 48),
                ("img_fullsize_preview1_open",  "fullscreen_open",  48),
                ("img_fullsize_preview1_close", "fullscreen_close", 48),
                # ref
                ("img_ref_menu",            "ref_all",          48),
                ("img_ref_menu_close",      "back_to_app",      48),
                ("img_ref_paginate_next",   "chevron_right",    32),
                ("img_ref_paginate_prev",   "chevron_left",     32),
                ("img_ref_all",             "ref_all",          48),
                ("img_unref_all",           "unref_all",        48),
                ("img_ref_x", "ref_x", 48),
                ("img_ref_y", "ref_y", 48),
                ("img_ref_z", "ref_z", 48),
                ("img_ref_a", "ref_a", 48),
                ("img_ref_b", "ref_b", 48),
                ("img_ref_c", "ref_c", 48),
                ("img_ref_u", "ref_u", 48),
                ("img_ref_v", "ref_v", 48),
                ("img_ref_w", "ref_w", 48),
                ("img_ref_0", "ref_0", 48),
                ("img_ref_1", "ref_1", 48),
                ("img_ref_2", "ref_2", 48),
                ("img_ref_3", "ref_3", 48),
                ("img_ref_4", "ref_4", 48),
                ("img_ref_5", "ref_5", 48),
                ("img_ref_6", "ref_6", 48),
                ("img_ref_7", "ref_7", 48),
                # touch off
                ("img_touch_off",           "touch_off", 48),
                ("img_touch_menu_close",    "back_to_app",  48),
                ("img_touch_paginate_next",   "chevron_right",    32),
                ("img_touch_paginate_prev",   "chevron_left",     32),
                ("img_touch_x", "touch_x", 48),
                ("img_touch_y", "touch_y", 48),
                ("img_touch_z", "touch_z", 48),
                ("img_touch_a", "touch_a", 48),
                ("img_touch_b", "touch_b", 48),
                ("img_touch_c", "touch_c", 48),
                ("img_touch_u", "touch_u", 48),
                ("img_touch_v", "touch_v", 48),
                ("img_touch_w", "touch_w", 48),
                # tool settings
                ("img_tools",       "hsk_mill_tool",        48),
                ("img_toolchange",  "mill_tool_change",     48),
                ("img_back_tool",   "back_to_app",          48),
                ("img_tool_by_no",  "mill_tool_change_num", 48),
                ("img_index_tool",  "mill_tool_set_num",    48),
                # auto mode buttons
                ("img_open",            "open_file",        32),
                ("img_reload1",         "refresh",          32),
                ("img_run",             "play",             32),
                ("img_stop",            "stop",             32),
                ("img_pause",           "pause",            32),
                ("img_pause_active",    "pause_active",     32),
                ("img_step",            "step",             32),
                ("img_run_from",        "run_from_line",    48),
                ("img_editor",          "edit_code",        32),
                ("img_skip_optional_active",    "skip_optional_active",     32),
                ("img_skip_optional_inactive",  "skip_optional_inactive",   32),
                # load file buttons
                ("img_home",            "home_folder",          32),
                ("img_dir_up",          "chevron_up",           32),
                ("img_sel_prev",        "chevron_left",         32),
                ("img_sel_next",        "chevron_right",        32),
                ("img_jump_to",         "user_defined_folder",  32),
                ("img_select",          "select_file",          32),
                ("img_back_file_load",  "back_to_app",          48),
                # edit file menu
                ("img_edit_menu_reload",        "refresh",          32),
                ("img_edit_menu_save",          "save",             32),
                ("img_edit_menu_save_as",       "save_as",          32),
                ("img_edit_menu_new",           "new_document",     32),
                ("img_edit_menu_keyboard",      "keyboard",         32),
                ("img_edit_menu_keyboard_hide", "keyboard_hide",    32),
                ("img_edit_menu_close",         "back_to_app",      48),
                # macro menu
                ("img_macro_menu_keyboard",         "keyboard",         32),
                ("img_macro_menu_keyboard_hide",    "keyboard_hide",    32),
                ("img_macro_menu_stop",             "stop",             32),
                # misc
                ("img_close", "logout", 48),
            ]

            # default style context see TODO below
            default_style = Gtk.Button().get_style_context()

            failed_icons = 0
            for widget_name, icon_name, size in icon_configs:
                try:
                    image = self.widgets[widget_name]
                    # TODO: That's kind a problem, as not every image has (yet) a parent (e.g. for toggle button only one
                    #  image is assigned at a time) and the default_style is maybe to inaccurate in terms of overridden
                    #  style attributes used by the icon loading mechanism (fg, succcss, warning and error colors)
                    # style = image.get_parent().get_style_context() if image.get_parent() else default_style
                    pixbuf = icon_theme_helper.load_symbolic_from_icon_theme(self.icon_theme, icon_name, size, default_style)
                    image.set_from_pixbuf(pixbuf)
                    image.set_size_request(size, size)
                except BaseException as err:
                    LOG.warning(f"Failed to change icon for <{widget_name}> to '{icon_name}': {str(err)}")
                    failed_icons += 1

            if failed_icons > 0:
                LOG.warning(f"{failed_icons} icons failed to load! (Maybe the icon theme is incomplete?)")

    def on_icon_theme_choice_changed(self, widget):
        active = widget.get_active_iter()
        if active is not None:
            name = widget.get_model()[active][1]
            self.prefs.putpref("icon_theme", name)
            self._set_icon_theme(name)

    def _set_sourceview_theme(self, name):
        self.widgets["gcode_view"].set_style_scheme(name)
        buffer = self.widgets["gcode_view"].get_buffer()
        style = buffer.get_style_scheme().get_style('current-line')
        color = style.props.background
        rgba = Gdk.RGBA()
        rgba.parse(color)
        self.widgets["gcode_view"].add_mark_category('motion', rgba.to_string())
        
    def on_sourceview_theme_choice_changed(self, widget):
        active = widget.get_active_iter()
        if active is not None:
            name = widget.get_model()[active][0]
            self.prefs.putpref("gcode_theme", name)
            self._set_sourceview_theme(name)

    def on_rbt_unlock_toggled(self, widget, data=None):
        if widget.get_active():
            if widget == self.widgets.rbt_use_unlock:
                self.prefs.putpref("unlock_way", "use")
            elif widget == self.widgets.rbt_no_unlock:
                self.prefs.putpref("unlock_way", "no")
            else:
                self.prefs.putpref("unlock_way", "hal")

    def on_rbtn_run_from_line_toggled(self, widget, data=None):
        if widget.get_active():
            if widget == self.widgets.rbtn_no_run_from_line:
                self.prefs.putpref("run_from_line", "no_run")
                self.widgets.btn_from_line.set_sensitive(False)
            else:  # widget == self.widgets.rbtn_run_from_line:
                self.prefs.putpref("run_from_line", "run")
                self.widgets.btn_from_line.set_sensitive(True)

    def on_chk_use_kb_on_offset_toggled(self, widget, data=None):
        self.prefs.putpref("show_keyboard_on_offset", widget.get_active())

    def on_chk_use_kb_on_tooledit_toggled(self, widget, data=None):
        self.prefs.putpref("show_keyboard_on_tooledit", widget.get_active())

    def on_chk_use_kb_on_edit_toggled(self, widget, data=None):
        self.prefs.putpref("show_keyboard_on_edit", widget.get_active())

    def on_chk_use_kb_on_mdi_toggled(self, widget, data=None):
        self.prefs.putpref("show_keyboard_on_mdi", widget.get_active())

    def on_chk_use_kb_on_file_selection_toggled(self, widget, data=None):
        self.prefs.putpref("show_keyboard_on_file_selection", widget.get_active())

    def on_chk_use_kb_shortcuts_toggled(self, widget, data=None):
        self.prefs.putpref("use_keyboard_shortcuts", widget.get_active())

    def on_rbtn_show_preview_toggled(self, widget, data=None):
        self.prefs.putpref("show_preview_on_offset", widget.get_active())

    def on_adj_scale_jog_vel_value_changed(self, widget, data=None):
        self.prefs.putpref("scale_jog_vel", widget.get_value(), float)
        self.scale_jog_vel = widget.get_value()

    def on_adj_scale_feed_override_value_changed(self, widget, data=None):
        self.prefs.putpref("scale_feed_override", widget.get_value(), float)
        self.scale_feed_override = widget.get_value()

    def on_adj_scale_rapid_override_value_changed(self, widget, data=None):
        self.prefs.putpref("scale_rapid_override", widget.get_value(), float)
        self.scale_rapid_override = widget.get_value()

    def on_adj_scale_spindle_override_value_changed(self, widget, data=None):
        self.prefs.putpref("scale_spindle_override", widget.get_value(), float)
        self.scale_spindle_override = widget.get_value()

    def on_rbtn_fullscreen_toggled(self, widget):
        if widget.get_active():
            self.widgets.window1.fullscreen()
            self.prefs.putpref("screen1", "fullscreen")
        else:
            self.widgets.window1.unfullscreen()

    def on_rbtn_maximized_toggled(self, widget):
        if widget.get_active():
            self.widgets.window1.maximize()
            self.prefs.putpref("screen1", "maximized")
        else:
            self.widgets.window1.unmaximize()
        self.widgets.chkbtn_hide_titlebar.set_sensitive(widget.get_active())

    def on_rbtn_window_toggled(self, widget):
        self.widgets.grid_main_window.set_sensitive(widget.get_active())
        self.widgets.chkbtn_hide_titlebar.set_sensitive(widget.get_active())
        # we have to check also if the window is active, because the button is toggled the first time
        # before the window is shown
        if widget.get_active() and self.widgets.window1.is_active():
            self.widgets.window1.set_decorated(not self.widgets.chkbtn_hide_titlebar.get_active())
            self.widgets.window1.move(self.xpos, self.ypos)
            self.widgets.window1.resize(self.width, self.height)
            self.prefs.putpref("screen1", "window")

    def on_chkbtn_hide_titlebar_toggled(self, widget):
        self.widgets.window1.set_decorated(not widget.get_active())
        self.widgets.window1.move(self.xpos, self.ypos)
        self.prefs.putpref("hide_titlebar", widget.get_active())

    def on_adj_x_pos_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("x_pos", value, float)
        self.xpos = value
        self.widgets.window1.move(value, self.ypos)

    def on_adj_y_pos_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("y_pos", value, float)
        self.ypos = value
        self.widgets.window1.move(self.xpos, value)

    def on_adj_width_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("width", value, float)
        self.width = value
        self.widgets.window1.resize(value, self.height)

    def on_adj_height_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("height", value, float)
        self.height = value
        self.widgets.window1.resize(self.width, value)

    def on_adj_kbd_height_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("kbd_height", value, int)
        self.kbd_height = value
        self._update_kbd_height(self.kbd_set_height)

    def on_adj_kbd_width_value_changed(self, widget, data=None):
        if not self.initialized:
            return
        value = int(widget.get_value())
        self.prefs.putpref("kbd_width", value, int)
        self.kbd_width = value
        self._update_kbd_width(self.kbd_set_width)

    def _update_kbd_height(self, seth):
        if seth:
            self.widgets.box_kbd_height.set_sensitive(True)
            self.widgets.key_box.set_valign(Gtk.Align.CENTER)
            if self.kbd_set_width:
                self.widgets.key_box.set_size_request(self.kbd_width, self.kbd_height)
            else:
                self.widgets.key_box.set_size_request(-1, self.kbd_height)
        else:
            self.widgets.box_kbd_height.set_sensitive(False)
            self.widgets.key_box.set_valign(Gtk.Align.FILL)
            if self.kbd_set_width:
                self.widgets.key_box.set_size_request(self.kbd_width, -1)
            else:
                self.widgets.key_box.set_size_request(-1, -1)
        self.prefs.putpref("kbd_set_height", seth, bool)
        
    def _update_kbd_width(self, setw):
        if setw:
            self.widgets.box_kbd_width.set_sensitive(True)
            self.widgets.key_box.set_halign(Gtk.Align.CENTER)
            if self.kbd_set_height:
                self.widgets.key_box.set_size_request(self.kbd_width, self.kbd_height)
            else:
                self.widgets.key_box.set_size_request(self.kbd_width, -1)
        else:
            self.widgets.box_kbd_width.set_sensitive(False)
            self.widgets.key_box.set_halign(Gtk.Align.FILL)
            if self.kbd_set_height:
                self.widgets.key_box.set_size_request(-1, self.kbd_height)
            else:
                self.widgets.key_box.set_size_request(-1, -1)
        self.prefs.putpref("kbd_set_width", setw, bool)
        
    def on_chk_kb_set_height_toggled(self, widget):
        self.kbd_set_height = widget.get_active()
        self._update_kbd_height(self.kbd_set_height)

    def on_chk_kb_set_width_toggled(self, widget):
        self.kbd_set_width = widget.get_active()
        self._update_kbd_width(self.kbd_set_width)

    def on_adj_dro_size_value_changed(self, widget, data=None):
        value = int(widget.get_value())
        self.prefs.putpref("dro_size", value, int)
        self.dro_size = value
        self._arrange_dro(attach=False)

    def on_chk_hide_cursor_toggled(self, widget, data=None):
        self.prefs.putpref("hide_cursor", widget.get_active())
        self.hide_cursor = widget.get_active()

        win = self.widgets.window1.get_window()
        if widget.get_active():
            self.cursor = Gdk.Cursor(Gdk.CursorType.BLANK_CURSOR)
        else:
            self.cursor = Gdk.Cursor(Gdk.CursorType.ARROW)
        win.set_cursor(self.cursor)

# Helper may be in an extra file, used also in CombiDRO
    def _convert_color(self, color):
        colortuple = ((int(color.red * 255.0), int(color.green * 255.0), int(color.blue * 255.0)))
        return ('#' + ''.join(f'{i:02X}' for i in colortuple))

    def on_chk_hide_tooltips_toggled(self, widget, data=None):
        LOG.debug("hide tooltips toggled")
        self.hide_tooltips = widget.get_active()
        self.prefs.putpref("hide_tooltips", self.hide_tooltips)
        self._set_enable_tooltips(not self.hide_tooltips)

    def on_rel_colorbutton_color_set(self, widget):
        color = widget.get_rgba()
        self.rel_color = self._convert_color(color)
        self.prefs.putpref('rel_color', self.rel_color)
        self._change_dro_color("rel_color", color)

    def on_abs_colorbutton_color_set(self, widget):
        color = widget.get_rgba()
        self.abs_color = self._convert_color(color)
        self.prefs.putpref('abs_color', self.abs_color)
        self._change_dro_color("abs_color", color)

    def on_dtg_colorbutton_color_set(self, widget):
        color = widget.get_rgba()
        self.dtg_color = self._convert_color(color)
        self.prefs.putpref('dtg_color', self.dtg_color)
        self._change_dro_color("dtg_color", color)

    def on_homed_colorbtn_color_set(self, widget):
        color = widget.get_rgba()
        self.homed_color = self._convert_color(color)
        self.prefs.putpref('homed_color', self.homed_color)
        self._change_dro_color("homed_color", color)

    def on_unhomed_colorbtn_color_set(self, widget):
        color = widget.get_rgba()
        self.unhomed_color = self._convert_color(color)
        self.prefs.putpref('unhomed_color', self.unhomed_color)
        self._change_dro_color("unhomed_color", color)

    def on_file_to_load_chooser_file_set(self, widget):
        self.prefs.putpref("open_file", widget.get_filename())

    def on_jump_to_dir_chooser_file_set(self, widget, data=None):
        path = widget.get_filename()
        self.prefs.putpref("jump_to_dir", path)
        self.widgets.IconFileSelection1.set_property("jump_to_dir", path)

    def on_grid_size_value_changed(self, widget, data=None):
        self.widgets.gremlin.set_property('grid_size', widget.get_value())
        self.prefs.putpref('grid_size', widget.get_value(), float)

    def on_chk_show_dro_toggled(self, widget, data=None):
        state = widget.get_active()
        self.widgets.gremlin.set_property("metric_units", self.dro_dic["Combi_DRO_0"].metric_units)
        self.widgets.gremlin.set_property("enable_dro", state)
        self.prefs.putpref("enable_dro", state)
        self.enable_gremlin_dro = state
        self.widgets.chk_show_offsets.set_sensitive(state)
        self.widgets.chk_show_dtg.set_sensitive(state)

    def on_chk_show_dtg_toggled(self, widget, data=None):
        state = widget.get_active()
        self.widgets.gremlin.set_property("show_dtg", state)
        self.prefs.putpref("show_dtg", state)

    def on_chk_show_offsets_toggled(self, widget, data=None):
        state = widget.get_active()
        self.widgets.gremlin.show_offsets = state
        self.prefs.putpref("show_offsets", state)

    def on_cmb_mouse_button_mode_changed(self, widget):
        index = widget.get_active()
        self.widgets.gremlin.set_property("mouse_btn_mode", index)
        self.prefs.putpref("mouse_btn_mode", index, int)

# =========================================================
# tool stuff
    # This is used to reload the tool in spindle after starting the GUI
    # This is called from the all_homed_signal
    def reload_tool(self):
        tool_to_load = self.prefs.getpref("tool_in_spindle", 0, int)
        if tool_to_load == 0:
            return
        self.load_tool = True
        self.tool_change = True

        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()

        command = "M61 Q {0} G43".format(tool_to_load)
        self.command.mdi(command)
        self.command.wait_complete()

    def on_btn_tool_clicked(self, widget, data=None):
        if self.widgets.tbtn_fullsize_preview0.get_active():
            self.widgets.tbtn_fullsize_preview0.set_active(False)
#            self.widgets.tbtn_fullsize_preview1.set_active(False)
        self.widgets.ntb_button.set_current_page(_BB_TOOL)
        self._show_tooledit_tab(True)

    # Here we create a manual tool change dialog
    def on_tool_change(self, widget):
        LOG.debug("on tool change")
        change = self.halcomp['toolchange-change']
        toolnumber = self.halcomp['toolchange-number']
        if change:
            # if toolnumber = 0 we will get an error because we will not be able to get
            # any tool description, so we avoid that case
            if toolnumber == 0:
                message = _("Please remove the mounted tool and press OK when done")
            else:
                try:
                    tooldescr = self.widgets.tooledit1.get_toolinfo(toolnumber)[16]
                    message = _("Please change to tool\n\n# {0:d}     {1}\n\n then click OK.").format(toolnumber, tooldescr)
                except:
                    message = _("Tool\n\n# {0:d}\n\n not in the tool table!").format(toolnumber)

            result = self.dialogs.warning_dialog(self, message, title=_("Manual Tool change"),\
                confirm_pin = 'toolchange-confirm', active_pin = 'toolchange-change')
            if result:
                self.halcomp["toolchange-changed"] = True
            else:
                LOG.debug("toolchange abort {0} {1}".format(self.stat.tool_in_spindle, self.halcomp['toolchange-number']))
                self.command.abort()
                self.halcomp['toolchange-number'] = self.stat.tool_in_spindle
                self.halcomp['toolchange-change'] = False
                self.halcomp['toolchange-changed'] = True
                message = _("Tool Change has been aborted!\n")
                message += _("The old tool will remain set!")
                self.dialogs.warning_dialog(self, message)
        else:
            self.halcomp['toolchange-changed'] = False

    def on_btn_delete_tool_clicked(self, widget, data=None):
        act_tool = self.stat.tool_in_spindle
        if act_tool == self.widgets.tooledit1.get_selected_tool():
            message = _("You are trying to delete the tool mounted in the spindle\n")
            message += _("This is not allowed, please change tool prior to delete it")
            self.dialogs.warning_dialog(self, _("Warning Tool can not be deleted!"), message)
            return

        self.widgets.tooledit1.delete(None)
        self.widgets.tooledit1.set_selected_tool(act_tool)

    def on_btn_add_tool_clicked(self, widget, data=None):
        self.widgets.tooledit1.add(None)

    def on_btn_reload_tooltable_clicked(self, widget, data=None):
        self.widgets.tooledit1.reload(None)
        self.widgets.tooledit1.set_selected_tool(self.stat.tool_in_spindle)

    def on_btn_apply_tool_changes_clicked(self, widget, data=None):
        self.widgets.tooledit1.save(None)
        self.widgets.tooledit1.set_selected_tool(self.stat.tool_in_spindle)

    def on_btn_tool_touchoff_clicked(self, widget, data=None):
        if not self.widgets.tooledit1.get_selected_tool():
            message = _("No or multiple tools selected in the tool table. ")
            message += _("Please select only one tool in the table!")
            self.dialogs.warning_dialog(self, _("Warning Tool Touch off not possible!"), message)
            return

        if self.widgets.tooledit1.get_selected_tool() != self.stat.tool_in_spindle:
            message = _("You can not touch off a tool, which is not mounted in the spindle! ")
            message += _("Your selection has been reset to the tool in spindle.")
            self.dialogs.warning_dialog(self, _("Warning Tool Touch off not possible!"), message)
            self.widgets.tooledit1.reload(self)
            self.widgets.tooledit1.set_selected_tool(self.stat.tool_in_spindle)
            return

        if "G41" in self.active_gcodes or "G42" in self.active_gcodes:
            message = _("Tool touch off is not possible with cutter radius compensation switched on!\n")
            message += _("Please emit an G40 before tool touch off.")
            self.dialogs.warning_dialog(self, _("Warning Tool Touch off not possible!"), message)
            return

        if widget == self.widgets.btn_tool_touchoff_x:
            axis = "x"
        elif widget == self.widgets.btn_tool_touchoff_z:
            axis = "z"
        else:
            self.dialogs.warning_dialog(self, _("Real big error!"),
                                   _("You managed to come to a place that is not possible in on_btn_tool_touchoff"))
            return

        value = self.dialogs.entry_dialog(self, data=None,
                                     header=_("Enter value for axis {0} to set:").format(axis.upper()),
                                     label=_("Set parameter of tool {0:d} and axis {1} to:").format(self.stat.tool_in_spindle, axis.upper()),
                                     integer=False)

        if value == "ERROR":
            message = _("Conversion error because of wrong entry for touch off axis {0}").format(axis.upper())
            self.dialogs.warning_dialog(self, _("Conversion error !"), message)
            return
        elif value == "CANCEL":
            return
        else:
            command = "G10 L10 P{0} {1}{2}".format(self.stat.tool_in_spindle, axis, value)
            self.command.mode(linuxcnc.MODE_MDI)
            self.command.wait_complete()
            self.command.mdi(command)
            self.command.wait_complete()
            if "G43" in self.active_gcodes:
                self.command.mdi("G43")
                self.command.wait_complete()
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()

    # select a tool entering a number
    def on_btn_select_tool_by_no_clicked(self, widget, data=None):
        value = self.dialogs.entry_dialog(self, data=None, header=_("Enter the tool number as integer "),
                                     label=_("Select the tool to change"), integer=True)
        if value == "ERROR":
            message = _("Conversion error because of wrong entry for tool number.\n")
            message += _("Enter only integer numbers!")
            self.dialogs.warning_dialog(self, _("Conversion error !"), message)
            return
        elif value == "CANCEL":
            return
        elif int(value) == self.stat.tool_in_spindle:
            message = _("Selected tool is already in spindle, no change needed.")
            self.dialogs.warning_dialog(self, _("Important Warning!"), message)
            return
        else:
            self.tool_change = True
            self.command.mode(linuxcnc.MODE_MDI)
            self.command.wait_complete()
            command = "T{0} M6".format(int(value))
            self.command.mdi(command)
            # Next two lines fix issue #3129 caused by GStat missing changes in interpreter mode
            command = "G4 P{0}".format(self.get_ini_info.get_cycle_time()/1000)
            self.command.mdi(command)
            
    # set tool with M61 Q? or with T? M6
    def on_btn_selected_tool_clicked(self, widget, data=None):
        tool = self.widgets.tooledit1.get_selected_tool()
        if tool == None:
            message = _("you selected no or more than one tool, the tool selection must be unique")
            self.dialogs.warning_dialog(self, _("Important Warning!"), message)
            return
        if tool == self.stat.tool_in_spindle:
            message = _("Selected tool is already in spindle, no change needed.")
            self.dialogs.warning_dialog(self, _("Important Warning!"), message)
            return
        if tool or tool == 0:
            self.tool_change = True
            tool = int(tool)
            self.command.mode(linuxcnc.MODE_MDI)
            self.command.wait_complete()

            if widget == self.widgets.btn_change_tool:
                command = "T{0} M6".format(tool)
            else:
                command = "M61 Q{0}".format(tool)
            self.command.mdi(command)
            # Next two lines fix issue #3120 also caused by GStat missing changes in interpreter mode
            command = "G4 P{0}".format(self.get_ini_info.get_cycle_time()/1000)
            self.command.mdi(command)
        else:
            message = _("Could not understand the entered tool number. Will not change anything!")
            self.dialogs.warning_dialog(self, _("Important Warning!"), message)

# =========================================================
# gremlin relevant calls
    def on_rbt_view_p_toggled(self, widget, data=None):
        if self.widgets.rbt_view_p.get_active():
            self.widgets.gremlin.set_property("view", "p")
            self.prefs.putpref("view", "p")

    def on_rbt_view_x_toggled(self, widget, data=None):
        if self.widgets.rbt_view_x.get_active():
            self.widgets.gremlin.set_property("view", "x")
            self.prefs.putpref("view", "x")

    def on_rbt_view_y_toggled(self, widget, data=None):
        if self.widgets.rbt_view_y.get_active():
            self.widgets.gremlin.set_property("view", "y")
            self.prefs.putpref("view", "y")

    def on_rbt_view_z_toggled(self, widget, data=None):
        if self.widgets.rbt_view_z.get_active():
            self.widgets.gremlin.set_property("view", "z")
            self.prefs.putpref("view", "z")

    def on_rbt_view_y2_toggled(self, widget, data=None):
        if self.widgets.rbt_view_y2.get_active():
            self.widgets.gremlin.set_property("view", "y2")
            self.prefs.putpref("view", "y2")

    def on_btn_zoom_in_clicked(self, widget, data=None):
        self.widgets.gremlin.zoom_in()

    def on_btn_zoom_out_clicked(self, widget, data=None):
        self.widgets.gremlin.zoom_out()

    def on_btn_delete_view_clicked(self, widget, data=None):
        self.widgets.gremlin.clear_live_plotter()

    def on_tbtn_view_dimension_toggled(self, widget, data=None):
        self.widgets.gremlin.set_property("show_extents_option", widget.get_active())
        self.prefs.putpref("view_dimension", self.widgets.tbtn_view_dimension.get_active())

    def on_tbtn_view_tool_path_toggled(self, widget, data=None):
        self.widgets.gremlin.set_property("show_live_plot", widget.get_active())
        self.prefs.putpref("view_tool_path", self.widgets.tbtn_view_tool_path.get_active())

    def on_gremlin_line_clicked(self, widget, line):
        self.widgets.gcode_view.set_line_number(line)

    def on_btn_load_clicked(self, widget, data=None):
        self.widgets.ntb_button.set_current_page(_BB_LOAD_FILE)
        self.widgets.ntb_preview.set_current_page(3)
        self.widgets.tbtn_fullsize_preview0.set_active(True)

        # self.dialog = Gtk.FileChooserDialog(title="Please choose a file", parent=self.widgets.window1, action=Gtk.FileChooserAction.OPEN)
        # self.dialog.add_buttons( Gtk.STOCK_CANCEL,
        #                     Gtk.ResponseType.CANCEL,
        #                     Gtk.STOCK_OPEN,
        #                     Gtk.ResponseType.OK,
        #                   )

        # self.add_filters(self.dialog)

        # response = self.dialog.run()
        # if response == Gtk.ResponseType.OK:
        #     File_selected = self.dialog.get_filename()
        #     print("Open clicked")
        #     print("File selected: " + File_selected)
        #     self.on_IconFileSelection1_selected(widget = None, path = File_selected)
        # elif response == Gtk.ResponseType.CANCEL:
        #     print("Cancel clicked")

        # self.dialog.destroy()

        self._show_iconview_tab(True)
        self.widgets.IconFileSelection1.refresh_filelist()
        self.widgets.IconFileSelection1.iconView.grab_focus()
        self.gcodeerror = ""
        # deactivate mode buttons, so changing modes is not possible while we are in file chooser
        self.widgets.rbt_manual.set_sensitive(False)
        self.widgets.rbt_mdi.set_sensitive(False)
        self.widgets.tbtn_setup.set_sensitive(False)

    def add_filters(self, dialog):
        filter_ngc = Gtk.FileFilter()
        filter_ngc.set_name("NC files")
        filter_ngc.add_pattern("*.ngc")
        dialog.add_filter(filter_ngc)

        filter_py = Gtk.FileFilter()
        filter_py.set_name("Python files")
        filter_py.add_mime_type("text/x-python")
        dialog.add_filter(filter_py)

        filter_any = Gtk.FileFilter()
        filter_any.set_name("Any files")
        filter_any.add_pattern("*")
        dialog.add_filter(filter_any)

    def on_btn_sel_next_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_sel_next.emit("clicked")

    def on_btn_sel_prev_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_sel_prev.emit("clicked")

    def on_btn_home_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_home.emit("clicked")

    def on_btn_jump_to_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_jump_to.emit("clicked")

    def on_btn_dir_up_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_dir_up.emit("clicked")

    def on_btn_select_clicked(self, widget, data=None):
        self.widgets.IconFileSelection1.btn_select.emit("clicked")

    def on_IconFileSelection1_selected(self, widget, path=None):
        if path:
            self.widgets.hal_action_open.load_file(path)
            self.widgets.ntb_preview.set_current_page(0)
            self.widgets.tbtn_fullsize_preview0.set_active(False)
#            self.widgets.tbtn_fullsize_preview1.set_active(False)
            self.widgets.ntb_button.set_current_page(_BB_AUTO)
            self._show_iconview_tab(False)
            # re-enable mode buttons, when returning from file selection
            self.widgets.rbt_manual.set_sensitive(True)
            self.widgets.rbt_mdi.set_sensitive(True)
            self.widgets.tbtn_setup.set_sensitive(True)

    def on_IconFileSelection1_sensitive(self, widget, buttonname, state):
        self.widgets[buttonname].set_sensitive(state)

    def on_IconFileSelection1_exit(self, widget):
        LOG.debug("exit icon file selection")
        self.widgets.ntb_preview.set_current_page(0)
        self.widgets.tbtn_fullsize_preview0.set_active(False)
#        self.widgets.tbtn_fullsize_preview1.set_active(False)
        self._show_iconview_tab(False)

    # edit a program or make a new one
    def on_btn_edit_clicked(self, widget, data=None):
        self.widgets.ntb_button.set_current_page(_BB_EDIT)
        self.widgets.ntb_preview.hide()
        self.widgets.grid_DRO.hide()
        self.widgets.vbox14.hide()
        self.widgets.vbox_jog.set_hexpand(True)
        self.widgets.box_dro_side.hide()
        if not self.widgets.vbx_jog.get_visible():
            self.widgets.vbx_jog.set_visible(True)
        self.widgets.gcode_view.set_editable(True)
        self.widgets.gcode_view.grab_focus()
        if self.widgets.chk_use_kb_on_edit.get_active():
            self.widgets.ntb_info.set_current_page(1)
        else:
            self.widgets.ntb_info.hide()
            
        self.widgets.grid_search.show()
        self.widgets.progressbar_pgm.hide()
        self.gcodeerror = ""
        self.file_changed = False
        # deactivate the mode buttons, so changing modes is not possible while we are editing
        self.widgets.rbt_manual.set_sensitive(False)
        self.widgets.rbt_mdi.set_sensitive(False)
        self.widgets.tbtn_setup.set_sensitive(False)

    def on_gcode_view_changed(self, widget, state):
        LOG.debug("G-code view changed (modified: {})".format(state))
        self.file_changed = state

    # Search and replace handling in edit mode
    # undo changes while in edit mode
    def on_btn_undo_clicked(self, widget, data=None):
        self.widgets.gcode_view.undo()

    # search backward while in edit mode
    def on_btn_search_back_clicked(self, widget, data=None):
        self.widgets.gcode_view.text_search(direction=False,
                                            mixed_case=self.widgets.chk_ignore_case.get_active(),
                                            text=self.widgets.search_entry.get_text())

    # search forward while in edit mode
    def on_btn_search_forward_clicked(self, widget, data=None):
        self.widgets.gcode_view.text_search(direction=True,
                                            mixed_case=self.widgets.chk_ignore_case.get_active(),
                                            text=self.widgets.search_entry.get_text())

    # replace text in edit mode
    def on_btn_replace_clicked(self, widget, data=None):
        self.widgets.gcode_view.replace_text_search(direction=True,
                                                    mixed_case=self.widgets.chk_ignore_case.get_active(),
                                                    text=self.widgets.search_entry.get_text(),
                                                    re_text=self.widgets.replace_entry.get_text(),
                                                    replace_all=self.widgets.chk_replace_all.get_active())

    # redo changes while in edit mode
    def on_btn_redo_clicked(self, widget, data=None):
        self.widgets.gcode_view.redo()

        # if we leave the edit mode, we will have to show all widgets again
    def on_ntb_button_switch_page(self, *args):
        LOG.debug("ntb_button_switch_page")
        if self.widgets.ntb_preview.get_current_page() == 0:  # preview tab is active,
            # check if offset tab is visible, if so we have to hide it
            page = self.widgets.ntb_preview.get_nth_page(1)
            if page.get_visible():
                self._show_offset_tab(False)
        elif self.widgets.ntb_preview.get_current_page() == 1:
            self._show_offset_tab(False)
        elif self.widgets.ntb_preview.get_current_page() == 2:
            self._show_tooledit_tab(False)
        elif self.widgets.ntb_preview.get_current_page() == 3:
            self._show_iconview_tab(False)

# TODO:    This line is not compared with the corresponding constant:
#          self.widgets.ntb_preview.get_current_page() == _BB_HOME or \
#          This line will be always false as ntb_preview has only four pages:
#          self.widgets.ntb_preview.get_current_page() == _BB_LOAD_FILE:
        if self.widgets.ntb_button.get_current_page()  == _BB_EDIT or \
           self.widgets.ntb_preview.get_current_page() == _BB_HOME or \
           self.widgets.ntb_preview.get_current_page() == _BB_LOAD_FILE:
            LOG.debug("we are in special case")
            self.widgets.ntb_preview.show()
            self.widgets.grid_DRO.show()
            self.widgets.vbox14.show()
            self.widgets.vbox_jog.set_hexpand(False)
            self.widgets.box_dro_side.show()
            self.widgets.gcode_view.set_editable(False)
            self.widgets.btn_save.set_sensitive(True)
            self.widgets.hal_action_reload.emit("activate")
            self.widgets.ntb_info.set_current_page(0)
            self.widgets.ntb_info.show()
            self.widgets.ntb_info.set_size_request(-1, -1)
            self.widgets.grid_search.hide()
            self.widgets.progressbar_pgm.show()

    # make a new file
    def on_btn_new_clicked(self, widget, data=None):
        tempfilename = os.path.join(_TEMPDIR, "temp.ngc")
        content = self.get_ini_info.get_RS274_start_code()
        content += "\n\n\n\nM2"
        gcodefile = open(tempfilename, "w")
        gcodefile.write(content)
        gcodefile.close()
        if self.widgets.lbl_program.get_label() == tempfilename:
            self.widgets.hal_action_reload.emit("activate")
        else:
            self.widgets.hal_action_open.load_file(tempfilename)
            # self.command.program_open(tempfilename)
        self.widgets.gcode_view.grab_focus()
        self.widgets.btn_save.set_sensitive(False)

    def saved_as(self, widget):
        self.widgets.btn_save.set_sensitive(True)

    def on_tbtn_optional_blocks_toggled(self, widget, data=None):
        opt_blocks = widget.get_active()
        self.command.set_block_delete(opt_blocks)
        self.prefs.putpref("blockdel", opt_blocks)
        self.widgets.hal_action_reload.emit("activate")
        widget.set_image(self.widgets["img_skip_optional_" + ("active" if opt_blocks else "inactive")])

    #def on_tbtn_optional_stops_toggled(self, widget, data=None):
    #    opt_stops = widget.get_active()
    #    self.command.set_optional_stop(opt_stops)
    #    self.prefs.putpref("opstop", opt_stops)

    # this can not be done with the status widget,
    # because it will not emit a RESUME signal
    def on_tbtn_pause_toggled(self, widget, data=None):
        widgetlist = ["rbt_forward", "rbt_reverse", "rbt_stop", "gcode_view"]
        self._sensitize_widgets(widgetlist, widget.get_active())
        widget.set_image(self.widgets["img_pause_active" if widget.get_active() else "img_pause"])

    def on_btn_stop_clicked(self, widget, data=None):
        self.command.abort()
        self.start_line = 0
        self.widgets.gcode_view.set_line_number(0)
        self.widgets.tbtn_pause.set_active(False)

    def on_btn_run_clicked(self, widget, data=None):
        self.command.auto(linuxcnc.AUTO_RUN, self.start_line)

    def on_btn_from_line_clicked(self, widget, data=None):
        self.dialogs.restart_dialog(self)
        
    def on_chk_en_audio_toggled(self, widget, data=None):
        self.prefs.putpref("audio_enabled", widget.get_active())
        self.audio_active = widget.get_active()

    def on_change_sound(self, widget, sound=None):
        file = widget.get_filename()
        if file:
            if widget == self.widgets.audio_error_chooser:
                self.error_sound = file
                self.prefs.putpref("audio_error", file)
            else:
                self.alert_sound = file
                self.prefs.putpref("audio_alert", file)
            self.audio.set_sound(file)
            self.audio.run()

    def on_tbtn_switch_mode_toggled(self, widget, data=None):
        if widget.get_active():
            self.widgets.tbtn_switch_mode.set_label(_(" Joint\nmode"))
            # Mode 1 = joint ; Mode 2 = MDI ; Mode 3 = teleop
            # so in mode 1 we have to show Joints and in Modes 2 and 3 axis values
            self._set_motion_mode(0)
        else:
            self.widgets.tbtn_switch_mode.set_label(_("World\nmode"))
            self._set_motion_mode(1)

# =========================================================
# Hal Pin Handling Start

    def _on_counts_changed(self, pin, widget):
        if not self.initialized:
            return
        difference = 0
        counts = pin.get()
        if self.halcomp["feed.feed-override.count-enable"]:
            if widget == "spc_feed":
                difference = (counts - self.fo_counts) * self.scale_feed_override
                self.fo_counts = counts
                self._check_counts(counts)
        if self.halcomp["rapid.rapid-override.count-enable"]:
            if widget == "spc_rapid":
                difference = (counts - self.ro_counts) * self.scale_rapid_override
                self.ro_counts = counts
                self._check_counts(counts)
        if self.halcomp["spindle.spindle-override.count-enable"]:
            if widget == "spc_spindle":
                difference = (counts - self.so_counts) * self.scale_spindle_override
                self.so_counts = counts
                self._check_counts(counts)
        if self.halcomp["jog.jog-velocity.count-enable"]:
            if widget == "spc_lin_jog_vel":
                difference = (counts - self.jv_counts) * self.scale_jog_vel
                if self.widgets.tbtn_turtle_jog.get_active():
                    difference = difference / self.turtle_jog_factor
                self.jv_counts = counts
                self._check_counts(counts)
        if not self.halcomp["feed.feed-override.count-enable"] \
                            and not self.halcomp["spindle.spindle-override.count-enable"] \
                            and not self.halcomp["jog.jog-velocity.count-enable"] \
                            and not self.halcomp["rapid.rapid-override.count-enable"]:
            self._check_counts(counts)

        val = self.widgets[widget].get_value() + difference
        if val < 0:
            val = 0
        if difference != 0:
            self.widgets[widget].set_value(val)

    def _check_counts(self, counts):
        # as we do not know how the user did connect the jog wheels, we have to check all
        # possibilities. Does he use only one jog wheel and a selection switch or do he use
        # a mpg for each slider or one for speeds and one for override, or ??
        if self.halcomp["feed.feed-override.counts"] == self.halcomp["spindle.spindle-override.counts"]:
            if self.halcomp["feed.feed-override.count-enable"] and self.halcomp["spindle.spindle-override.count-enable"]:
                return
            self.fo_counts = self.so_counts = counts
        if self.halcomp["feed.feed-override.counts"] == self.halcomp["jog.jog-velocity.counts"]:
            if self.halcomp["feed.feed-override.count-enable"] and self.halcomp["jog.jog-velocity.count-enable"]:
                return
            self.fo_counts = self.jv_counts = counts
        if self.halcomp["feed.feed-override.counts"] == self.halcomp["rapid.rapid-override.counts"]:
            if self.halcomp["feed.feed-override.count-enable"] and self.halcomp["rapid.rapid-override.count-enable"]:
                return
            self.fo_counts = self.ro_counts = counts
        if self.halcomp["spindle.spindle-override.counts"] == self.halcomp["jog.jog-velocity.counts"]:
            if self.halcomp["spindle.spindle-override.count-enable"] and self.halcomp["jog.jog-velocity.count-enable"]:
                return
            self.so_counts = self.jv_counts = counts
        if self.halcomp["spindle.spindle-override.counts"] == self.halcomp["rapid.rapid-override.counts"]:
            if self.halcomp["spindle.spindle-override.count-enable"] and self.halcomp["rapid.rapid-override.count-enable"]:
                return
            self.so_counts = self.ro_counts = counts
        if self.halcomp["jog.jog-velocity.counts"] == self.halcomp["rapid.rapid-override.counts"]:
            if self.halcomp["jog.jog-velocity.count-enable"] and self.halcomp["rapid.rapid-override.count-enable"]:
                return
            self.jv_counts = self.ro_counts = counts

    def _on_analog_enable_changed(self, pin, widget):
        if not self.initialized:
            return
        if widget == "spc_spindle":
            if pin.get():
                self.widgets.btn_spindle_100.hide()
            else:
                self.widgets.btn_spindle_100.show()
        if widget == "spc_feed":
            if pin.get():
                self.widgets.btn_feed_100.hide()
            else:
                self.widgets.btn_feed_100.show()
        # widget can also be spc_lin_jog_vel and spc_rapid
        self.widgets[widget].hide_button(pin.get())

        if pin.get() and widget == "spc_lin_jog_vel":
            # special case of jog_vel, as we have to take care of both modes,
            # more details see _on_analog_value_changed
            if self.widgets.tbtn_turtle_jog.get_active():
                value = self.rabbit_jog = self.jog_rate_max * self.halcomp["jog.jog-velocity.direct-value"]
            elif not self.widgets.tbtn_turtle_jog.get_active():
                value = self.turtle_jog = self.jog_rate_max / self.turtle_jog_factor * self.halcomp["jog.jog-velocity.direct-value"]
            self.widgets.spc_lin_jog_vel.set_value(value)

    def _on_analog_value_changed(self, pin, widget):
        if not self.initialized:
            return
        if widget == "spc_lin_jog_vel" and not self.halcomp["jog.jog-velocity.analog-enable"]:
            return
        if widget == "spc_feed" and not self.halcomp["feed.feed-override.analog-enable"]:
            return
        if widget == "spc_spindle" and not self.halcomp["spindle.spindle-override.analog-enable"]:
            return
        if widget == "spc_rapid" and not self.halcomp["rapid.rapid-override.analog-enable"]:
            return
        percentage = pin.get()
        if percentage > 1.0:
            percentage = 1.0
        range = self.widgets[widget].get_property("max") - self.widgets[widget].get_property("min")
        try:  # otherwise a value of 0.0 would give an error
            value = self.widgets[widget].get_property("min") + (range * percentage)
        except:
            value = 0
        self.widgets[widget].set_value(value)

        # special case of jog_vel, as we have to take care of both modes,
        # meaning the analog value must be applied to both! If we do not do this,
        # it might be that a user has analog in signal set to 0.5 and switch the mode
        # but in the other mode he had only 0.3 from its value, so a small change of the
        # analog in to 0.51 would result in a jump of 20 %!
        if self.widgets.tbtn_turtle_jog.get_active():
            self.rabbit_jog = self.jog_rate_max * pin.get()
        elif not self.widgets.tbtn_turtle_jog.get_active():
            self.turtle_jog = self.jog_rate_max / self.turtle_jog_factor * pin.get()

    def _on_unlock_settings_changed(self, pin):
        if not self.initialized:
            return
        if not self.widgets.rbt_hal_unlock.get_active() and not self.user_mode:
            return
        self.widgets.tbtn_setup.set_sensitive(pin.get())

    def _on_play_sound(self, widget, sound = None):
        LOG.debug("_on_play_sound {0} {1} {2}".format(self,widget,sound))
        if self.audio_active and sound:
            if sound == "error":
                self.audio.set_sound(self.error_sound)
            elif sound == "alert":
                self.audio.set_sound(self.alert_sound)
            else:
                LOG.debug("got unknown sound to play")
                return
            self.audio.run()

    def _on_message_deleted(self, widget, messages):
        number = []
        for message in messages:
            if message[2] == ALERT_ICON:
                number.append(message[0])
        if len(number) == 0:
            self.halcomp["error"] = False

    def _del_message_changed(self, pin):
        if pin.get():
            if self.halcomp["error"] == True:
                number = []
                messages = self.notification.messages
                for message in messages:
                    if message[2] == ALERT_ICON:
                        number.append(message[0])
                self.notification.del_message(number[0])
                if len(number) == 1:
                    self.halcomp["error"] = False
            else:
                self.notification.del_last()

    def _on_pin_incr_changed(self, pin, buttonnumber):
#        if self.stat.state != 1:
#            self.command.abort()
#            self.command.wait_complete()
        if not pin.get():
            return
        btn_name = "rbt_{0}".format(buttonnumber)
        self._jog_increment_changed(self.incr_rbt_dic[btn_name])
        self.incr_rbt_dic[btn_name].set_active(True)

    def _on_pin_jog_changed(self, pin, button_name):
        LOG.debug("Jog Pin Changed")
        LOG.debug(button_name)
        if self.stat.kinematics_type != linuxcnc.KINEMATICS_IDENTITY:
            if self.stat.motion_mode == 1 and pin.get():
                message = _("Axis jogging is only allowed in world mode, but you are in joint mode!")
                LOG.debug(message)
                self._show_error((13, message))
                return

        if pin.get():
            self._on_btn_jog_pressed(None, button_name)
        else:
            self._on_btn_jog_released(None, button_name)

    def _reset_override(self, pin, type):
        if pin.get():
            if type == "rapid":
                self.command.rapidrate(1.0)
                return
            self.widgets["btn_{0}_100".format(type)].emit("clicked")

    def _on_blockheight_value_changed(self, pin):
        self.widgets.lbl_blockheight.set_text("blockheight = {0:.3f}".format(pin.get()))
        if self.lathe_mode:
            self.widgets.lbl_blockheight.hide()

    def _optional_blocks(self, pin):
        LOG.debug("Received a signal from pin {0} with state = {1}".format(pin.name, pin.get()))
        self.widgets["tbtn_optional_blocks"].set_active(pin.get())

    def _blockdelete(self, pin):
        LOG.debug("Received a signal from pin {0} with state = {1}".format(pin.name, pin.get()))
        self.command.set_optional_stop(pin.get())

# =========================================================
# The actions of the buttons
    def _button_pin_changed(self, pin):
        # we check if the button is pressed or released,
        # otherwise a signal will be emitted, if the button is released and
        # the signal drop down to zero
        if not pin.get():
            return

        if "h-button" in pin.name:
            location = "bottom"
        elif "v-button" in pin.name:
            location = "right"
        else:
            LOG.debug("Received a not classified signal from pin {0}".format(pin.name))
            return

        try:
            number = int(pin.name[-1])
        except:
            LOG.debug("Could not translate {0} to number".format(pin.name))
            return

        button = self._get_child_button(location, number)
        if not button:
            LOG.debug("no button here")
            return
        elif button == -1:
            LOG.debug("the button is not sensitive")
            return

        if type(button[0]) == Gtk.ToggleButton:
            button[0].set_active(not button[0].get_active())
            LOG.debug("Button {0} has been toggled".format(button[1]))
        elif type(button[0]) == Gtk.RadioButton:
            button[0].set_active(True)
            button[0].emit("pressed")
            LOG.debug("Button {0} has been pressed".format(button[1]))
        else:
            button[0].emit("clicked")
            LOG.debug("Button {0} has been clicked".format(button[1]))

    # this handles the relation between hardware button and the software button
    def _get_child_button(self, location, number = None):
        # get the position of each button to be able to connect to hardware button
        self.child_button_dic = {}

        if location == "bottom":
            page = self.widgets.ntb_button.get_current_page()
            container = self.widgets.ntb_button.get_children()[page]
        elif location == "right":
            container = self.widgets.vbtb_main
        else:
            LOG.debug("got wrong location to locate the children")

        children = container.get_children()
        hidden = 0
        for child in children:
            if not child.get_visible():
                hidden +=1
            else:
                if type(child) != Gtk.Label:
                    pos = container.child_get_property(child, "position")
                    name = child.get_property("name")
                    if name == None:
                        name = Gtk.Buildable.get_name(child)
                    self.child_button_dic[pos - hidden] = (child, name)

        if number is not None:
            try:
                if self.child_button_dic[number][0].get_sensitive():
                    return self.child_button_dic[number]
                else:
                    return -1
            except:
                return None
        else:
            return self.child_button_dic


# We need extra HAL pins here is where we do it.
# we make pins for the hardware buttons which can be placed around the
# screen to activate the corresponding buttons on the GUI
    def _make_hal_pins(self):
        # generate the horizontal button pins
        for h_button in range(0, 10):
            pin = self.halcomp.newpin("h-button.button-{0}".format(h_button), hal.HAL_BIT, hal.HAL_IN)
            hal_glib.GPin(pin).connect("value_changed", self._button_pin_changed)

        # generate the vertical button pins
        for v_button in range(0, 7):
            pin = self.halcomp.newpin("v-button.button-{0}".format(v_button), hal.HAL_BIT, hal.HAL_IN)
            hal_glib.GPin(pin).connect("value_changed", self._button_pin_changed)

        # buttons for jogging the axis
        for jog_button in self.axis_list:
            pin = self.halcomp.newpin("jog.axis.jog-{0}-plus".format(jog_button), hal.HAL_BIT, hal.HAL_IN)
            hal_glib.GPin(pin).connect("value_changed", self._on_pin_jog_changed, "{0}+".format(jog_button))
            pin = self.halcomp.newpin("jog.axis.jog-{0}-minus".format(jog_button), hal.HAL_BIT, hal.HAL_IN)
            hal_glib.GPin(pin).connect("value_changed", self._on_pin_jog_changed, "{0}-".format(jog_button))

        if self.stat.kinematics_type != linuxcnc.KINEMATICS_IDENTITY:
            for joint_button in range(0, self.stat.joints):
                pin = self.halcomp.newpin("jog.joint.jog-{0}-plus".format(joint_button), hal.HAL_BIT, hal.HAL_IN)
                hal_glib.GPin(pin).connect("value_changed", self._on_pin_jog_changed, "{0}+".format(joint_button))
                pin = self.halcomp.newpin("jog.joint.jog-{0}-minus".format(joint_button), hal.HAL_BIT, hal.HAL_IN)
                hal_glib.GPin(pin).connect("value_changed", self._on_pin_jog_changed, "{0}+".format(joint_button))

        # jog_increment out pin
        self.halcomp.newpin("jog.jog-increment", hal.HAL_FLOAT, hal.HAL_OUT)

        # generate the pins to set the increments
        for buttonnumber in range(0, len(self.jog_increments)):
            pin = self.halcomp.newpin("jog.jog-inc-{0}".format(buttonnumber), hal.HAL_BIT, hal.HAL_IN)
            hal_glib.GPin(pin).connect("value_changed", self._on_pin_incr_changed, buttonnumber)

        # make the pin for unlocking settings page
        pin = self.halcomp.newpin("unlock-settings", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_unlock_settings_changed)

        # generate the pins to connect encoders to the sliders
        pin = self.halcomp.newpin("feed.feed-override.counts", hal.HAL_S32, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_counts_changed, "spc_feed")
        pin = self.halcomp.newpin("spindle.spindle-override.counts", hal.HAL_S32, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_counts_changed, "spc_spindle")
        pin = self.halcomp.newpin("jog.jog-velocity.counts", hal.HAL_S32, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_counts_changed, "spc_lin_jog_vel")
        pin = self.halcomp.newpin("rapid.rapid-override.counts", hal.HAL_S32, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_counts_changed, "spc_rapid")
        self.halcomp.newpin("feed.feed-override.count-enable", hal.HAL_BIT, hal.HAL_IN)
        self.halcomp.newpin("spindle.spindle-override.count-enable", hal.HAL_BIT, hal.HAL_IN)
        self.halcomp.newpin("jog.jog-velocity.count-enable", hal.HAL_BIT, hal.HAL_IN)
        self.halcomp.newpin("rapid.rapid-override.count-enable", hal.HAL_BIT, hal.HAL_IN)

        # generate the pins to connect analog inputs for sliders
        pin = self.halcomp.newpin("feed.feed-override.analog-enable", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_enable_changed, "spc_feed")
        pin = self.halcomp.newpin("spindle.spindle-override.analog-enable", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_enable_changed, "spc_spindle")
        pin = self.halcomp.newpin("jog.jog-velocity.analog-enable", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_enable_changed, "spc_lin_jog_vel")
        pin = self.halcomp.newpin("rapid.rapid-override.analog-enable", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_enable_changed, "spc_rapid")
        pin = self.halcomp.newpin("feed.feed-override.direct-value", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_value_changed, "spc_feed")
        pin = self.halcomp.newpin("spindle.spindle-override.direct-value", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_value_changed, "spc_spindle")
        pin = self.halcomp.newpin("jog.jog-velocity.direct-value", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_value_changed, "spc_lin_jog_vel")
        pin = self.halcomp.newpin("rapid.rapid-override.direct-value", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_analog_value_changed, "spc_rapid")

        # make a pin to set turtle jog vel
        pin = self.halcomp.newpin("jog.turtle-jog", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._on_pin_turtle_jog)

        # make the pins for tool measurement
        self.halcomp.newpin("probeheight", hal.HAL_FLOAT, hal.HAL_OUT)
        pin = self.halcomp.newpin("blockheight", hal.HAL_FLOAT, hal.HAL_OUT)
        hal_glib.GPin(pin).connect("value_changed", self._on_blockheight_value_changed)
        preset = self.prefs.getpref("blockheight", 0.0, float)
        self.halcomp["blockheight"] = preset
        self.halcomp.newpin("toolmeasurement", hal.HAL_BIT, hal.HAL_OUT)
        self.halcomp.newpin("searchvel", hal.HAL_FLOAT, hal.HAL_OUT)
        self.halcomp.newpin("probevel", hal.HAL_FLOAT, hal.HAL_OUT)

        # make pins to react to tool_offset changes
        pin = self.halcomp.newpin("tooloffset-x", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._offset_changed, "tooloffset-x")
        pin = self.halcomp.newpin("tooloffset-z", hal.HAL_FLOAT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._offset_changed, "tooloffset-z")
        self.halcomp.newpin("tool-diameter", hal.HAL_FLOAT, hal.HAL_OUT)

        # make a pin to delete a notification message
        pin = self.halcomp.newpin("delete-message", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._del_message_changed)

        # for manual tool change dialog
        self.halcomp.newpin("toolchange-number", hal.HAL_S32, hal.HAL_IN)
        self.halcomp.newpin("toolchange-changed", hal.HAL_BIT, hal.HAL_OUT)
        pin = self.halcomp.newpin('toolchange-change', hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect('value_changed', self.on_tool_change)
        self.halcomp.newpin('toolchange-confirm', hal.HAL_BIT, hal.HAL_IN)

        # make a pin to confirm a warning dialog
        self.halcomp.newpin('warning-confirm', hal.HAL_BIT, hal.HAL_IN)

        # make a pin to reset feed override to 100 %
        pin = self.halcomp.newpin("feed.reset-feed-override", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._reset_override, "feed")

        # make a pin to reset rapid override to 100 %
        pin = self.halcomp.newpin("rapid.reset-rapid-override", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._reset_override, "rapid")

        # make a pin to reset spindle override to 100 %
        pin = self.halcomp.newpin("spindle.reset-spindle-override", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._reset_override, "spindle")

        # make an error pin to indicate a error to hardware
        self.halcomp.newpin("error", hal.HAL_BIT, hal.HAL_OUT)

        # make pins to indicate program progress information
        self.halcomp.newpin("program.length", hal.HAL_S32, hal.HAL_OUT)
        self.halcomp.newpin("program.current-line", hal.HAL_S32, hal.HAL_OUT)
        self.halcomp.newpin("program.progress", hal.HAL_FLOAT, hal.HAL_OUT)

        # make a pin to set ignore limits
        pin = self.halcomp.newpin("ignore-limits", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._ignore_limits)

        # make pins to set optional stops and block delete
        pin = self.halcomp.newpin("optional-stop", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._optional_blocks)
        pin = self.halcomp.newpin("blockdelete", hal.HAL_BIT, hal.HAL_IN)
        hal_glib.GPin(pin).connect("value_changed", self._blockdelete)


# Hal Pin Handling End
# =========================================================

if __name__ == "__main__":
    # Set up the base logger
    #   We have do do this before importing other modules because on import
    #   they set up their own loggers as children of the base logger.

    # If log_file is none, logger.py will attempt to find the log file specified in
    # INI [DISPLAY] LOG_FILE, failing that it will log to $HOME/<base_log_name>.log

    # Note: In all other modules it is best to use the `__name__` attribute
    #   to ensure we get a logger with the correct hierarchy.
    #   Ex: LOG = logger.getLogger(__name__)

    from qtvcp import logger
    LOG = logger.initBaseLogger('Gmoccapy', log_file=None, log_level=logger.WARNING)

    # we set the log level early so the imported modules get the right level
    # The order is: VERBOSE, DEBUG, INFO, WARNING, ERROR, CRITICAL.

    if '-d' in sys.argv:
        # Log level defaults to WARNING, so set lower if in debug mode
        logger.setGlobalLevel(logger.DEBUG)
        LOG.debug('DEBUGGING logging on')
    elif '-i' in sys.argv:
        # Log level defaults to WARNING, so set lower if in info mode
        logger.setGlobalLevel(logger.INFO)
        LOG.info('INFO logging on')
    elif '-v' in sys.argv:
        # Log level defaults to WARNING, so set lowest if in verbose mode
        logger.setGlobalLevel(logger.VERBOSE)
        LOG.verbose('VERBOSE logging on')
    elif '-q' in sys.argv:
        logger.setGlobalLevel(logger.ERROR)

    # Some of these libraries log when imported so logging level must already be set.
    import gladevcp.makepins
    from gladevcp.core import Info
    from gladevcp.combi_dro import Combi_DRO  # we will need it to make the DRO
    from gmoccapy import widgets       # a class to handle the widgets

    from gmoccapy import notification  # this is the module we use for our error handling
    from gmoccapy import preferences   # this handles the preferences
    from gmoccapy import getiniinfo    # this handles the INI File reading so checking is done in that module
    from gmoccapy import dialogs       # this takes the code of all our dialogs
    from gmoccapy import icon_theme_helper
    try:
        from gmoccapy import player        # a class to handle sounds
        _AUDIO_AVAILABLE = True
    except:
        _AUDIO_AVAILABLE = False

    # instantiate gmoccapy
    app = gmoccapy(sys.argv)

    # get the INI path
    inifile = sys.argv[2]

    # find the POST_GUI HAL file path
    LOG.debug("inifile = {0} :".format(sys.argv[2]))
    postgui_halfile = app.get_ini_info.get_postgui_halfile()

    # run it depending on type
    LOG.debug("postgui halfile = {0}".format(postgui_halfile))
    if postgui_halfile is not None:
        for f in postgui_halfile:
            if f.lower().endswith('.tcl'):
                res = os.spawnvp(os.P_WAIT, "haltcl", ["haltcl", "-i", inifile, f])
            else:
                res = os.spawnvp(os.P_WAIT, "halcmd", ["halcmd", "-i", inifile, "-f", f])
                if res:
                    LOG.error('postgui halfile error:{}'.format(res))
                    raise SystemExit(res)

        postgui_halcmds = app.get_ini_info.get_postgui_halcmds()
        LOG.info("Postgui commands: yellow<{}>".format(postgui_halcmds))
        if postgui_halcmds is not None:
            for f in postgui_halcmds:
                f = os.path.expanduser(f)
                res = os.spawnvp(os.P_WAIT, "halcmd", ["halcmd"] + f.split())
                if res:
                    LOG.error('postgui command error:{}'.format(res))
                    raise SystemExit(res)


    # start the event loop
    Gtk.main()

