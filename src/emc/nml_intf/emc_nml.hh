/********************************************************************
* Description: emc_nml.hh
*   Declarations for EMC NML vocabulary
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/
#ifndef EMC_NML_HH
#define EMC_NML_HH
#include "linuxcnc.h"
#include "emc.hh"
#include "rcs.hh"
#include "cmd_msg.hh"
#include "stat_msg.hh"
#include "emcpos.h"
#include "modal_state.hh"
#include "canon.hh"		// CANON_TOOL_TABLE, CANON_UNITS
#include "rs274ngc.hh"		// ACTIVE_G_CODES, etc

// ------------------
// CLASS DECLARATIONS
// ------------------

// declarations for EMC general classes

/**
 * Send a textual error message to the operator.
 * The message is put in the errlog buffer to be read by the GUI.
 * This allows the controller a generic way to send error messages to
 * the operator.
 */
class EMC_OPERATOR_ERROR:public RCS_CMD_MSG {
  public:
    EMC_OPERATOR_ERROR()
      : RCS_CMD_MSG(EMC_OPERATOR_ERROR_TYPE, sizeof(EMC_OPERATOR_ERROR)),
        error{}
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
    char error[LINELEN];
};

/**
 * Send a textual information message to the operator.
 * This is similar to EMC_OPERATOR_ERROR message except that the messages are
 * sent in situations not necessarily considered to be errors.
 */
class EMC_OPERATOR_TEXT:public RCS_CMD_MSG {
  public:
    EMC_OPERATOR_TEXT()
      : RCS_CMD_MSG(EMC_OPERATOR_TEXT_TYPE, sizeof(EMC_OPERATOR_TEXT)),
        text{}
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
    char text[LINELEN];
};

/**
 * Send the URL or filename of a document to display.
 * This message is placed in the errlog buffer  to be read by the GUI.
 * If the GUI is capable of doing so it will show the operator a
 * previously created document, using the URL or filename provided.
 * This message is placed in the errlog channel to be read by the GUI.
 * This provides a general means of reporting an error from within the
 * controller without having to program the GUI to recognize each error type.
 */
class EMC_OPERATOR_DISPLAY:public RCS_CMD_MSG {
  public:
    EMC_OPERATOR_DISPLAY()
      : RCS_CMD_MSG(EMC_OPERATOR_DISPLAY_TYPE, sizeof(EMC_OPERATOR_DISPLAY)),
        display{}
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
    char display[LINELEN];
};

#define EMC_SYSTEM_CMD_LEN 256
/*
  execute a system command
*/
class EMC_SYSTEM_CMD:public RCS_CMD_MSG {
  public:
    EMC_SYSTEM_CMD()
      : RCS_CMD_MSG(EMC_SYSTEM_CMD_TYPE, sizeof(EMC_SYSTEM_CMD)),
        string{}
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char string[EMC_SYSTEM_CMD_LEN];
};

class EMC_NULL:public RCS_CMD_MSG {
  public:
    EMC_NULL():RCS_CMD_MSG(EMC_NULL_TYPE, sizeof(EMC_NULL)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_SET_DEBUG:public RCS_CMD_MSG {
  public:
    EMC_SET_DEBUG()
      : RCS_CMD_MSG(EMC_SET_DEBUG_TYPE, sizeof(EMC_SET_DEBUG)),
        debug(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned debug;
};


/*
 * EMC_JOG_CMD_MSG class.
 */
class EMC_JOG_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_JOG_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s),
        joint_or_axis(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // joint_or_axis == joint_number          for joint jogs (jjogmode==1)
    // joint_or_axis == 0 for X, 1 for Y,...  for axis  jogs (jjogmode==0)
    int joint_or_axis;
};

// AXIS status base class
class EMC_AXIS_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_AXIS_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s),
        axis(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int axis;
};

class EMC_AXIS_STAT:public EMC_AXIS_STAT_MSG {
  public:
    EMC_AXIS_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double minPositionLimit;
    double maxPositionLimit;
    double velocity;		// current velocity
};

// declarations for EMC_JOINT classes

/*
 * JOINT command base class.
 * This is the base class for all commands that operate on a single joint.
 * The joint parameter specifies which joint the command affects.
 * These commands are sent to the emcCommand buffer to be read by the
 * TASK program that will then pass along corresponding messages to the
 * motion system.
 */
class EMC_JOINT_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_JOINT_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s),
        joint(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int joint;
};

/**
 * Set the Axis backlash.
 * This command sets the backlash value.
 */
class EMC_JOINT_SET_BACKLASH:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_SET_BACKLASH()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_SET_BACKLASH_TYPE, sizeof(EMC_JOINT_SET_BACKLASH)),
        backlash(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double backlash;
};

class EMC_JOINT_SET_MIN_POSITION_LIMIT:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_SET_MIN_POSITION_LIMIT()
    : EMC_JOINT_CMD_MSG (EMC_JOINT_SET_MIN_POSITION_LIMIT_TYPE, sizeof(EMC_JOINT_SET_MIN_POSITION_LIMIT)),
      limit(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double limit;
};

class EMC_JOINT_SET_MAX_POSITION_LIMIT:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_SET_MAX_POSITION_LIMIT()
      : EMC_JOINT_CMD_MSG (EMC_JOINT_SET_MAX_POSITION_LIMIT_TYPE, sizeof(EMC_JOINT_SET_MAX_POSITION_LIMIT)),
        limit(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double limit;
};

class EMC_JOINT_SET_FERROR:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_SET_FERROR()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_SET_FERROR_TYPE, sizeof(EMC_JOINT_SET_FERROR)),
        ferror(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double ferror;
};

class EMC_JOINT_SET_MIN_FERROR:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_SET_MIN_FERROR()
      : EMC_JOINT_CMD_MSG (EMC_JOINT_SET_MIN_FERROR_TYPE, sizeof(EMC_JOINT_SET_MIN_FERROR)),
        ferror(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double ferror;
};

class EMC_JOINT_SET_HOMING_PARAMS:public EMC_JOINT_CMD_MSG {
  public:
    // False positive. Triggers on 'offset'
    // cppcheck-suppress uninitMemberVar
    EMC_JOINT_SET_HOMING_PARAMS()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_SET_HOMING_PARAMS_TYPE, sizeof(EMC_JOINT_SET_HOMING_PARAMS)),
        home(0.0),
        offset(0.0),
        home_final_vel(0.0),
        search_vel(0.0),
        latch_vel(0.0),
        use_index(0),
        encoder_does_not_reset(0),
        ignore_limits(0),
        is_shared(0),
        home_sequence(0),
        volatile_home(0),
        locking_indexer(0),
        absolute_encoder(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double home;
    double offset;
    double home_final_vel;
    double search_vel;
    double latch_vel;
    int use_index;
    int encoder_does_not_reset;
    int ignore_limits;
    int is_shared;
    int home_sequence;
    int volatile_home;
    int locking_indexer;
    int absolute_encoder;
};

class EMC_JOINT_HALT:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_HALT()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_HALT_TYPE, sizeof(EMC_JOINT_HALT))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_JOINT_HOME:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_HOME()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_HOME_TYPE, sizeof(EMC_JOINT_HOME))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_JOINT_UNHOME:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_UNHOME()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_UNHOME_TYPE, sizeof(EMC_JOINT_UNHOME))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_JOG_CONT:public EMC_JOG_CMD_MSG {
  public:
    EMC_JOG_CONT()
      : EMC_JOG_CMD_MSG(EMC_JOG_CONT_TYPE, sizeof(EMC_JOG_CONT)),
        vel(0.0),
        jjogmode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class EMC_JOG_INCR:public EMC_JOG_CMD_MSG {
  public:
    EMC_JOG_INCR()
      : EMC_JOG_CMD_MSG(EMC_JOG_INCR_TYPE, sizeof(EMC_JOG_INCR)),
        incr(0.0),
        vel(0.0),
        jjogmode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double incr;
    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class EMC_JOG_ABS:public EMC_JOG_CMD_MSG {
  public:
    EMC_JOG_ABS()
      : EMC_JOG_CMD_MSG(EMC_JOG_ABS_TYPE, sizeof(EMC_JOG_ABS)),
        pos(0.0),
        vel(0.0),
        jjogmode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double pos;
    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class EMC_JOG_STOP:public EMC_JOG_CMD_MSG {
  public:
    EMC_JOG_STOP()
      : EMC_JOG_CMD_MSG(EMC_JOG_STOP_TYPE, sizeof(EMC_JOG_STOP)),
        jjogmode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class EMC_JOINT_OVERRIDE_LIMITS:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_OVERRIDE_LIMITS()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_OVERRIDE_LIMITS_TYPE, sizeof(EMC_JOINT_OVERRIDE_LIMITS))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_JOINT_LOAD_COMP:public EMC_JOINT_CMD_MSG {
  public:
    EMC_JOINT_LOAD_COMP()
      : EMC_JOINT_CMD_MSG(EMC_JOINT_LOAD_COMP_TYPE, sizeof(EMC_JOINT_LOAD_COMP)),
        file{},
        type(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    char file[LINELEN];
    int type; // type of the comp file. type==0 means nom, forw, rev triplets
              // type != 0 means nom, forw_trim, rev_trim triplets
};


// JOINT status base class
class EMC_JOINT_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_JOINT_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s),
        joint(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int joint;
};

class EMC_JOINT_STAT:public EMC_JOINT_STAT_MSG {
  public:
    EMC_JOINT_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    // configuration parameters
    unsigned char jointType;	// EMC_JOINT_LINEAR, EMC_JOINT_ANGULAR
    double units;		// units per mm, deg for linear, angular
    double backlash;
    double minPositionLimit;
    double maxPositionLimit;
    double maxFerror;
    double minFerror;

    // dynamic status
    double ferrorCurrent;	// current following error
    double ferrorHighMark;	// magnitude of max following error
    /*! \todo FIXME - is this really position, or the DAC output? */
    double output;		// commanded output position
    double input;		// current input position
    double velocity;		// current velocity
    unsigned char inpos;	// non-zero means in position
    unsigned char homing;	// non-zero means homing
    unsigned char homed;	// non-zero means has been homed
    unsigned char fault;	// non-zero means axis amp fault
    unsigned char enabled;	// non-zero means enabled
    unsigned char minSoftLimit;	// non-zero means min soft limit exceeded
    unsigned char maxSoftLimit;	// non-zero means max soft limit exceeded
    unsigned char minHardLimit;	// non-zero means min hard limit exceeded
    unsigned char maxHardLimit;	// non-zero means max hard limit exceeded
    unsigned char overrideLimits; // non-zero means limits are overridden
};

// declarations for EMC_TRAJ classes

// EMC_TRAJ command base class
class EMC_TRAJ_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_TRAJ_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s),
        tag()
    {};

    //NOTE this does NOT have a corresponding CMS update. This only works
    //because motion commands don't actually go through NML.
    StateTag tag;
    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_TRAJ_SET_MODE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_MODE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_MODE_TYPE, sizeof(EMC_TRAJ_SET_MODE)),
        mode(EMC_TRAJ_MODE::FREE)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EMC_TRAJ_MODE mode;
};

class EMC_TRAJ_SET_VELOCITY:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_VELOCITY()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_VELOCITY_TYPE, sizeof(EMC_TRAJ_SET_VELOCITY)),
        velocity(0.0),
        ini_maxvel(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double velocity;
    double ini_maxvel;
};

class EMC_TRAJ_SET_ACCELERATION:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_ACCELERATION()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_ACCELERATION_TYPE, sizeof(EMC_TRAJ_SET_ACCELERATION)),
        acceleration(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double acceleration;
};

class EMC_TRAJ_SET_MAX_VELOCITY:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_MAX_VELOCITY()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_MAX_VELOCITY_TYPE, sizeof(EMC_TRAJ_SET_MAX_VELOCITY)),
        velocity(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double velocity;
};

class EMC_TRAJ_SET_SCALE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_SCALE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_SCALE_TYPE, sizeof(EMC_TRAJ_SET_SCALE)),
        scale(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double scale;
};

class EMC_TRAJ_SET_RAPID_SCALE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_RAPID_SCALE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_RAPID_SCALE_TYPE, sizeof(EMC_TRAJ_SET_RAPID_SCALE)),
        scale(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double scale;
};

class EMC_TRAJ_SET_SPINDLE_SCALE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_SPINDLE_SCALE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_SPINDLE_SCALE_TYPE, sizeof(EMC_TRAJ_SET_SPINDLE_SCALE)),
        spindle(0),
        scale(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    int spindle;
    double scale;
};

class EMC_TRAJ_SET_FO_ENABLE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_FO_ENABLE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_FO_ENABLE_TYPE, sizeof(EMC_TRAJ_SET_FO_ENABLE)),
        mode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    unsigned char mode; //mode=0, override off (will work with 100% FO), mode != 0, override on, user can change FO
};

class EMC_TRAJ_SET_SO_ENABLE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_SO_ENABLE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_SO_ENABLE_TYPE, sizeof(EMC_TRAJ_SET_SO_ENABLE)),
        spindle(0),
        mode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
    unsigned char mode; //mode=0, override off (will work with 100% SO), mode != 0, override on, user can change SO
};

class EMC_TRAJ_SET_FH_ENABLE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_FH_ENABLE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_FH_ENABLE_TYPE, sizeof(EMC_TRAJ_SET_FH_ENABLE)),
        mode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    unsigned char mode; //mode=0, override off (feedhold is disabled), mode != 0, override on, user can use feedhold
};

class EMC_TRAJ_ABORT:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_ABORT()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_ABORT_TYPE, sizeof(EMC_TRAJ_ABORT))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TRAJ_PAUSE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_PAUSE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_PAUSE_TYPE, sizeof(EMC_TRAJ_PAUSE))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TRAJ_RESUME:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_RESUME()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_RESUME_TYPE, sizeof(EMC_TRAJ_RESUME))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TRAJ_DELAY:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_DELAY()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_DELAY_TYPE, sizeof(EMC_TRAJ_DELAY)),
        delay(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double delay;		// delay in seconds
};

class EMC_TRAJ_LINEAR_MOVE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_LINEAR_MOVE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_LINEAR_MOVE_TYPE, sizeof(EMC_TRAJ_LINEAR_MOVE)),
        type(0),
        end{},
        vel(0.0),
        ini_maxvel(0.0),
        acc(0.0),
        feed_mode(0),
        indexer_jnum(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int type;
    EmcPose end;		// end point
    double vel, ini_maxvel, acc;
    int feed_mode;
    int indexer_jnum;
};

class EMC_TRAJ_CIRCULAR_MOVE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_CIRCULAR_MOVE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_CIRCULAR_MOVE_TYPE, sizeof(EMC_TRAJ_CIRCULAR_MOVE)),
        end{},
        center{},
        normal{},
        turn(0),
        type(0),
        vel(0.0),
        ini_maxvel(0.0),
        acc(0.0),
        feed_mode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EmcPose end;
    PM_CARTESIAN center;
    PM_CARTESIAN normal;
    int turn;
    int type;
    double vel, ini_maxvel, acc;
    int feed_mode;
};

class EMC_TRAJ_SET_TERM_COND:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_TERM_COND()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_TERM_COND_TYPE, sizeof(EMC_TRAJ_SET_TERM_COND)),
        cond(false),
        tolerance(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int cond;
    double tolerance; // used to set the precision/tolerance of path deviation 
		      // during CONTINUOUS motion mode. 
};

class EMC_TRAJ_SET_SPINDLESYNC:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_SPINDLESYNC()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_SPINDLESYNC_TYPE, sizeof(EMC_TRAJ_SET_SPINDLESYNC)),
        spindle(0),
        feed_per_revolution(0.0),
        velocity_mode(false)
    {};

    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
    double feed_per_revolution;
    bool velocity_mode;
};

class EMC_TRAJ_SET_OFFSET:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_OFFSET()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_OFFSET_TYPE, sizeof(EMC_TRAJ_SET_OFFSET)),
        offset{}
    {};

    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    // For internal NML/CMS use only.
    void update(CMS * cms);

    EmcPose offset;
};

class EMC_TRAJ_SET_G5X:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_G5X()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_G5X_TYPE, sizeof(EMC_TRAJ_SET_G5X)),
        g5x_index(0),
        origin{}
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    int g5x_index;
    EmcPose origin;
};

class EMC_TRAJ_SET_G92:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_G92()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_G92_TYPE, sizeof(EMC_TRAJ_SET_G92)),
        origin{}
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EmcPose origin;
};

class EMC_TRAJ_SET_ROTATION:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_ROTATION()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_ROTATION_TYPE, sizeof(EMC_TRAJ_SET_ROTATION)),
        rotation(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double rotation;
};

class EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE, sizeof(EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TRAJ_SET_TELEOP_ENABLE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_SET_TELEOP_ENABLE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_SET_TELEOP_ENABLE_TYPE, sizeof(EMC_TRAJ_SET_TELEOP_ENABLE)),
        enable(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int enable;
};

class EMC_TRAJ_PROBE:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_PROBE()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_PROBE_TYPE, sizeof(EMC_TRAJ_PROBE)),
        pos{},
        type(0),
        vel(0.0),
        ini_maxvel(0.0),
        acc(0.0),
        probe_type(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EmcPose pos;
    int type;
    double vel, ini_maxvel, acc;
    unsigned char probe_type;
};

class EMC_TRAJ_RIGID_TAP:public EMC_TRAJ_CMD_MSG {
  public:
    EMC_TRAJ_RIGID_TAP()
      : EMC_TRAJ_CMD_MSG(EMC_TRAJ_RIGID_TAP_TYPE, sizeof(EMC_TRAJ_RIGID_TAP)),
        pos{},
        vel(0.0),
        ini_maxvel(0.0),
        acc(0.0),
        scale(1.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EmcPose pos;
    double vel, ini_maxvel, acc, scale;
};

// EMC_TRAJ status base class
class EMC_TRAJ_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_TRAJ_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_TRAJ_STAT:public EMC_TRAJ_STAT_MSG {
  public:
    EMC_TRAJ_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double linearUnits;		// units per mm
    double angularUnits;	// units per degree
    double cycleTime;		// cycle time, in seconds
    int joints;			// maximum joint number
    int spindles;			// maximum spindle number
    int axis_mask;		// mask of axes actually present
    EMC_TRAJ_MODE mode;	// EMC_TRAJ_MODE::FREE,
    // EMC_TRAJ_MODE::COORD
    bool enabled;		// non-zero means enabled

    bool inpos;			// non-zero means in position
    int queue;			// number of pending motions, counting
    // current
    int activeQueue;		// number of motions blending
    bool queueFull;		// non-zero means can't accept another motion
    int id;			// id of the currently executing motion
    bool paused;			// non-zero means motion paused
    double scale;		// velocity scale factor
    double rapid_scale;		// rapid scale factor
    //double spindle_scale;	// moved to EMC_SPINDLE_STAT

    EmcPose position;		// current commanded position
    EmcPose actualPosition;	// current actual position, from forward kins
    double velocity;		// system velocity, for subsequent motions
    double acceleration;	// system acceleration, for subsequent
    // motions
    double maxVelocity;		// max system velocity
    double maxAcceleration;	// system acceleration

    EmcPose probedPosition;	// last position where probe was tripped.
    bool probe_tripped;		// Has the probe been tripped since the last
    // clear.
    bool probing;		// Are we currently looking for a probe
    // signal.
    int probeval;		// Current value of probe input.
    int kinematics_type;	// identity=1,serial=2,parallel=3,custom=4
    int motion_type;
    double distance_to_go;         // in current move
    EmcPose dtg;
    double current_vel;         // in current move
    bool feed_override_enabled;
    //bool spindle_override_enabled; moved to SPINDLE_STAT
    bool adaptive_feed_enabled;
    bool feed_hold_enabled;
    StateTag tag;
};

// emc_MOTION is aggregate of all EMC motion-related status classes

// EMC_MOTION command base class
class EMC_MOTION_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_MOTION_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_MOTION_SET_AOUT:public EMC_MOTION_CMD_MSG {
  public:
    EMC_MOTION_SET_AOUT()
      : EMC_MOTION_CMD_MSG(EMC_MOTION_SET_AOUT_TYPE, sizeof(EMC_MOTION_SET_AOUT)),
        index(0),
        start(0.0),
        end(0.0),
        now(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    unsigned char index;	// which to set
    double start;		// value at start
    double end;			// value at end
    unsigned char now;		// whether command is immediate or synched with motion
};

class EMC_MOTION_SET_DOUT:public EMC_MOTION_CMD_MSG {
  public:
    EMC_MOTION_SET_DOUT()
      : EMC_MOTION_CMD_MSG(EMC_MOTION_SET_DOUT_TYPE, sizeof(EMC_MOTION_SET_DOUT)),
        index(0),
        start(0.0),
        end(0.0),
        now(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    unsigned char index;	// which to set
    unsigned char start;	// binary value at start
    unsigned char end;		// binary value at end
    unsigned char now;		// whether command is immediate or synched with motion
};

class EMC_MOTION_ADAPTIVE:public EMC_MOTION_CMD_MSG {
  public:
    EMC_MOTION_ADAPTIVE()
      : EMC_MOTION_CMD_MSG(EMC_MOTION_ADAPTIVE_TYPE, sizeof(EMC_MOTION_ADAPTIVE)),
        status(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    unsigned char status;		// status=0 stop; status=1 start.
};

// EMC_MOTION status base class
class EMC_MOTION_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_MOTION_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s)
    {};
};


// EMC_SPINDLE status base class
class EMC_SPINDLE_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_SPINDLE_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_SPINDLE_STAT:public EMC_SPINDLE_STAT_MSG {
  public:
    EMC_SPINDLE_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    double speed;		// spindle speed in RPMs
    double spindle_scale;	// spindle over-ride
    double css_maximum;
    double css_factor;  // CSS Status
    int state;
    int direction;		// 0 stopped, 1 forward, -1 reverse
    int brake;			// 0 released, 1 engaged
    int increasing;		// 1 increasing, -1 decreasing, 0 neither
    int enabled;		// non-zero means enabled
    int orient_state;
    int orient_fault;
    bool spindle_override_enabled;
    bool homed;
};

class EMC_MOTION_STAT:public EMC_MOTION_STAT_MSG {
  public:
    EMC_MOTION_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // aggregate of motion-related status classes
    EMC_TRAJ_STAT traj;
    EMC_JOINT_STAT joint[EMCMOT_MAX_JOINTS];
    EMC_AXIS_STAT axis[EMCMOT_MAX_AXIS];
    EMC_SPINDLE_STAT spindle[EMCMOT_MAX_SPINDLES];

    int synch_di[EMCMOT_MAX_DIO];  // motion inputs queried by interp
    int synch_do[EMCMOT_MAX_DIO];  // motion outputs queried by interp
    double analog_input[EMCMOT_MAX_AIO]; //motion analog inputs queried by interp
    double analog_output[EMCMOT_MAX_AIO]; //motion analog outputs queried by interp
    int misc_error[EMCMOT_MAX_MISC_ERROR];
    int debug;			// copy of EMC_DEBUG global
    int on_soft_limit;
    int external_offsets_applied;
    EmcPose eoffset_pose;
    int numExtraJoints;
    bool jogging_active;
};

// declarations for EMC_TASK classes

// EMC_TASK command base class
class EMC_TASK_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_TASK_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_ABORT:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_ABORT()
      : EMC_TASK_CMD_MSG(EMC_TASK_ABORT_TYPE, sizeof(EMC_TASK_ABORT))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_SET_MODE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_SET_MODE()
      : EMC_TASK_CMD_MSG(EMC_TASK_SET_MODE_TYPE, sizeof(EMC_TASK_SET_MODE)),
        mode(EMC_TASK_MODE::AUTO)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EMC_TASK_MODE mode;
};

class EMC_TASK_SET_STATE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_SET_STATE()
      : EMC_TASK_CMD_MSG(EMC_TASK_SET_STATE_TYPE, sizeof(EMC_TASK_SET_STATE)),
        state(EMC_TASK_STATE::ESTOP)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EMC_TASK_STATE state;
};

class EMC_TASK_PLAN_OPEN:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_OPEN()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_OPEN_TYPE, sizeof(EMC_TASK_PLAN_OPEN)),
        file{},
        remote_filesize(0),
        remote_buffersize(0),
        remote_buffer{}
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    // (local) path to file
    char file[LINELEN];
    // total size of file in bytes (if issued from remote process, 0 otherwise)
    size_t remote_filesize;
    // amount of bytes currently in buffer (if issued from remote process, 0 otherwise)
    size_t remote_buffersize;
    // buffer used to transfer send a chunk of file contents (if loaded from remote process)
    char remote_buffer[4096];
};

class EMC_TASK_PLAN_RUN:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_RUN()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_RUN_TYPE, sizeof(EMC_TASK_PLAN_RUN)),
        line(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int line;			// line to run from; 0 or 1 means from start,
    // negative means run through to verify
};

class EMC_TASK_PLAN_EXECUTE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_EXECUTE()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_EXECUTE_TYPE, sizeof(EMC_TASK_PLAN_EXECUTE)),
        command{}
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    char command[LINELEN];
};

class EMC_TASK_PLAN_PAUSE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_PAUSE()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_PAUSE_TYPE, sizeof(EMC_TASK_PLAN_PAUSE))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_REVERSE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_REVERSE()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_REVERSE_TYPE, sizeof(EMC_TASK_PLAN_REVERSE))
    {};

};

class EMC_TASK_PLAN_FORWARD:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_FORWARD()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_FORWARD_TYPE, sizeof(EMC_TASK_PLAN_FORWARD))
    {};

};


class EMC_TASK_PLAN_STEP:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_STEP()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_STEP_TYPE, sizeof(EMC_TASK_PLAN_STEP))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_RESUME:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_RESUME()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_RESUME_TYPE, sizeof(EMC_TASK_PLAN_RESUME))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_END:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_END()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_END_TYPE, sizeof(EMC_TASK_PLAN_END))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_CLOSE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_CLOSE()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_CLOSE_TYPE, sizeof(EMC_TASK_PLAN_CLOSE))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_INIT:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_INIT()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_INIT_TYPE, sizeof(EMC_TASK_PLAN_INIT))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_SYNCH:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_SYNCH()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_SYNCH_TYPE, sizeof(EMC_TASK_PLAN_SYNCH))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TASK_PLAN_SET_OPTIONAL_STOP:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_SET_OPTIONAL_STOP()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_SET_OPTIONAL_STOP_TYPE, sizeof(EMC_TASK_PLAN_SET_OPTIONAL_STOP)),
        state(false)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    
    bool state; //state == ON, optional stop is on (e.g. we stop on any stops)
};

class EMC_TASK_PLAN_SET_BLOCK_DELETE:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_SET_BLOCK_DELETE()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_SET_BLOCK_DELETE_TYPE, sizeof(EMC_TASK_PLAN_SET_BLOCK_DELETE)),
        state(false)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    
    bool state; //state == ON, block delete is on, we ignore lines starting with "/"
};

class EMC_TASK_PLAN_OPTIONAL_STOP:public EMC_TASK_CMD_MSG {
  public:
    EMC_TASK_PLAN_OPTIONAL_STOP()
      : EMC_TASK_CMD_MSG(EMC_TASK_PLAN_OPTIONAL_STOP_TYPE, sizeof(EMC_TASK_PLAN_OPTIONAL_STOP))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    
};


// EMC_TASK status base class
class EMC_TASK_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_TASK_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s),
	heartbeat(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    uint32_t heartbeat;
};

class EMC_TASK_STAT:public EMC_TASK_STAT_MSG {
  public:
    EMC_TASK_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    EMC_TASK_MODE mode;	// EMC_TASK_MODE::MANUAL, etc.
    EMC_TASK_STATE state;	// EMC_TASK_STATE::ESTOP, etc.

    EMC_TASK_EXEC execState;	// EMC_DONE,WAITING_FOR_MOTION, etc.
    EMC_TASK_INTERP interpState;	// EMC_IDLE,READING,PAUSED,WAITING
    int callLevel;              // current subroutine level - 0 if not in a subroutine, > 0 otherwise
    int motionLine;		// line motion is executing-- may lag
    int currentLine;		// line currently executing
    int readLine;		// line interpreter has read to
    bool optional_stop_state;	// state of optional stop (== ON means we stop on M1)
    bool block_delete_state;	// state of block delete (== ON means we ignore lines starting with "/")
    bool input_timeout;		// has a timeout happened on digital input
    char file[LINELEN];
    char command[LINELEN];
    char ini_filename[LINELEN];
    EmcPose g5x_offset;		// in user units, currently active
    int g5x_index;              // index of active g5x system
    EmcPose g92_offset;		// in user units, currently active
    double rotation_xy;
    EmcPose toolOffset;		// tool offset, in general pose form
    int activeGCodes[ACTIVE_G_CODES];
    int activeMCodes[ACTIVE_M_CODES];
    double activeSettings[ACTIVE_SETTINGS];
    CANON_UNITS programUnits;	// CANON_UNITS_INCHES, MM, CM

    int interpreter_errcode;	// return value from rs274ngc function 
    // (only useful for new interpreter.)
    int task_paused;		// non-zero means task is paused
    double delayLeft;           // delay time left of G4, M66..
    int queuedMDIcommands;      // current length of MDI input queue
};

// declarations for EMC_TOOL classes

// EMC_TOOL command base class
class EMC_TOOL_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_TOOL_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TOOL_HALT:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_HALT()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_HALT_TYPE, sizeof(EMC_TOOL_HALT))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TOOL_ABORT:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_ABORT()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_ABORT_TYPE, sizeof(EMC_TOOL_ABORT)),
        reason(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    int reason;		//  convey reason for abort to iocontrol
};

class EMC_TOOL_PREPARE:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_PREPARE()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_PREPARE_TYPE, sizeof(EMC_TOOL_PREPARE)),
        tool(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    int tool;
};

class EMC_TOOL_LOAD:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_LOAD()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_LOAD_TYPE, sizeof(EMC_TOOL_LOAD))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TOOL_UNLOAD:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_UNLOAD()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_UNLOAD_TYPE, sizeof(EMC_TOOL_UNLOAD))
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_TOOL_LOAD_TOOL_TABLE:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_LOAD_TOOL_TABLE()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_LOAD_TOOL_TABLE_TYPE, sizeof(EMC_TOOL_LOAD_TOOL_TABLE)),
        file{}
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    char file[LINELEN];		// name of tool table, empty means default
};

class EMC_TOOL_SET_OFFSET:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_SET_OFFSET()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_SET_OFFSET_TYPE, sizeof(EMC_TOOL_SET_OFFSET)),
        pocket(0),
        toolno(0),
        offset{},
        diameter(0.0),
        frontangle(0.0),
        backangle(0.0),
        orientation(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int pocket;
    int toolno;
    EmcPose offset;
    double diameter;
    double frontangle;
    double backangle;
    int    orientation;
};

class EMC_TOOL_SET_NUMBER:public EMC_TOOL_CMD_MSG {
  public:
    EMC_TOOL_SET_NUMBER()
      : EMC_TOOL_CMD_MSG(EMC_TOOL_SET_NUMBER_TYPE, sizeof(EMC_TOOL_SET_NUMBER)),
        tool(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int tool; //number to use for currently loaded tool
};

// EMC_TOOL status base class
class EMC_TOOL_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_TOOL_STAT_MSG(NMLTYPE t, size_t s)
      : RCS_STAT_MSG(t, s)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_TOOL_STAT:public EMC_TOOL_STAT_MSG {
  public:
    EMC_TOOL_STAT();
    EMC_TOOL_STAT(const EMC_TOOL_STAT &) = delete; // No copy constructor

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    EMC_TOOL_STAT& operator =(const EMC_TOOL_STAT &s);	// need this for [] members

    int pocketPrepped;		// idx ready for loading from
    int toolInSpindle;		// tool loaded, 0 is no tool
    int toolFromPocket;     // tool was loaded from this pocket
#ifdef TOOL_NML //{
    CANON_TOOL_TABLE toolTable[CANON_POCKETS_MAX];
#else //}{
    CANON_TOOL_TABLE toolTableCurrent; //current tool data
#endif //}

};

// EMC_AUX type declarations

// EMC_AUX command base class
class EMC_AUX_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_AUX_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_AUX_INPUT_WAIT:public EMC_AUX_CMD_MSG {
  public:
    EMC_AUX_INPUT_WAIT()
      : EMC_AUX_CMD_MSG(EMC_AUX_INPUT_WAIT_TYPE, sizeof(EMC_AUX_INPUT_WAIT)),
        index(0),
        input_type(0),
        wait_type(0),
        timeout(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int index;			// input channel to wait for
    int input_type;		// DIGITAL or ANALOG
    int wait_type;		// 0 - immediate, 1- rise, 2 - fall, 3 - be high, 4 - be low
    double timeout;		// timeout for waiting
};


// EMC_AUX status base class
class EMC_AUX_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_AUX_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_AUX_STAT:public EMC_AUX_STAT_MSG {
  public:
    EMC_AUX_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int estop;			// non-zero means estopped
};

// EMC_SPINDLE type declarations

// EMC_SPINDLE command base class
class EMC_SPINDLE_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_SPINDLE_CMD_MSG(NMLTYPE t, size_t s)
      : RCS_CMD_MSG(t, s),
        spindle(0)
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
};

class EMC_SPINDLE_SPEED:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_SPEED()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_SPEED_TYPE, sizeof(EMC_SPINDLE_SPEED)),
        spindle(0),
        speed(0.0),
        factor(0.0),
        xoffset(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
    double speed;   // commanded speed in RPMs or maximum speed for CSS
    double factor;  // Zero for constant RPM.  numerator of speed for CSS
    double xoffset; // X axis offset compared to center of rotation, for CSS
};

class EMC_SPINDLE_ORIENT:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_ORIENT()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_ORIENT_TYPE, sizeof(EMC_SPINDLE_ORIENT)),
        spindle(0),
        orientation(0.0),
        mode(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
    double orientation;   // desired spindle position
    int    mode;   
};

class EMC_SPINDLE_WAIT_ORIENT_COMPLETE:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_WAIT_ORIENT_COMPLETE()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE, sizeof(EMC_SPINDLE_WAIT_ORIENT_COMPLETE)),
        spindle(0),
        timeout(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
    double timeout;   // how long to wait until spindle orient completes; > 0
};


class EMC_SPINDLE_ON:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_ON()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_ON_TYPE, sizeof(EMC_SPINDLE_ON)),
	spindle(0),
        speed(0),
        factor(0),
        xoffset(0),
        wait_for_spindle_at_speed(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;    // the spindle to be turned on
    double speed;   // commanded speed in RPMs or maximum speed for CSS
    double factor;  // Zero for constant RPM.  numerator of speed for CSS
    double xoffset; // X axis offset compared to center of rotation, for CSS
    int wait_for_spindle_at_speed;
};

class EMC_SPINDLE_OFF:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_OFF()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_OFF_TYPE, sizeof(EMC_SPINDLE_OFF)),
        spindle(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;    // the spindle to be turned off
};

class EMC_SPINDLE_INCREASE:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_INCREASE()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_INCREASE_TYPE, sizeof(EMC_SPINDLE_INCREASE)),
        spindle(0),
        speed(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
    int spindle;        // the spindle to be increased
    double speed;		// commanded speed in RPMs
};

class EMC_SPINDLE_DECREASE:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_DECREASE()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_DECREASE_TYPE, sizeof(EMC_SPINDLE_DECREASE)),
        spindle(0),
        speed(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;        // the spindle to be decreased
    double speed;		// commanded speed in RPMs
};

class EMC_SPINDLE_CONSTANT:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_CONSTANT()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_CONSTANT_TYPE, sizeof(EMC_SPINDLE_CONSTANT)),
        spindle(0),
        speed(0.0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;        // the spindle to be constanted?
    double speed;		// commanded speed in RPMs
};

class EMC_SPINDLE_BRAKE_RELEASE:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_BRAKE_RELEASE()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_BRAKE_RELEASE_TYPE, sizeof(EMC_SPINDLE_BRAKE_RELEASE)),
        spindle(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
};

class EMC_SPINDLE_BRAKE_ENGAGE:public EMC_SPINDLE_CMD_MSG {
  public:
    EMC_SPINDLE_BRAKE_ENGAGE()
      : EMC_SPINDLE_CMD_MSG(EMC_SPINDLE_BRAKE_ENGAGE_TYPE, sizeof(EMC_SPINDLE_BRAKE_ENGAGE)),
        spindle(0)
    {};

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int spindle;
};

// EMC_COOLANT type declarations

// EMC_COOLANT command base class
class EMC_COOLANT_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_COOLANT_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_COOLANT_MIST_ON:public EMC_COOLANT_CMD_MSG {
  public:
    EMC_COOLANT_MIST_ON():EMC_COOLANT_CMD_MSG(EMC_COOLANT_MIST_ON_TYPE,
					      sizeof(EMC_COOLANT_MIST_ON))
    {
    };

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_COOLANT_MIST_OFF:public EMC_COOLANT_CMD_MSG {
  public:
    EMC_COOLANT_MIST_OFF():EMC_COOLANT_CMD_MSG(EMC_COOLANT_MIST_OFF_TYPE,
					       sizeof
					       (EMC_COOLANT_MIST_OFF)) {
    };

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_COOLANT_FLOOD_ON:public EMC_COOLANT_CMD_MSG {
  public:
    EMC_COOLANT_FLOOD_ON():EMC_COOLANT_CMD_MSG(EMC_COOLANT_FLOOD_ON_TYPE,
					       sizeof
					       (EMC_COOLANT_FLOOD_ON)) {
    };

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

class EMC_COOLANT_FLOOD_OFF:public EMC_COOLANT_CMD_MSG {
  public:
    EMC_COOLANT_FLOOD_OFF():EMC_COOLANT_CMD_MSG(EMC_COOLANT_FLOOD_OFF_TYPE,
						sizeof
						(EMC_COOLANT_FLOOD_OFF)) {
    };

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);
};

// EMC_COOLANT status base class
class EMC_COOLANT_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_COOLANT_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_COOLANT_STAT:public EMC_COOLANT_STAT_MSG {
  public:
    EMC_COOLANT_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    int mist;			// 0 off, 1 on
    int flood;			// 0 off, 1 on
};

// EMC_IO is aggregate of all EMC IO-related status classes

// EMC_IO status base class
class EMC_IO_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_IO_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };
};

class EMC_IO_STAT:public EMC_IO_STAT_MSG {
  public:
    EMC_IO_STAT()
      : EMC_IO_STAT_MSG(EMC_IO_STAT_TYPE, sizeof(EMC_IO_STAT)),
        debug(0),
        reason(0),
        fault(0),
        tool(),
        coolant(),
        aux()
    {};

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // top-level stuff
    int debug;			// copy of EMC_DEBUG global
    int reason;			// to communicate abort/fault cause
    int fault;                  //  0 on success, 1 on fault during M6
    // aggregate of IO-related status classes
    EMC_TOOL_STAT tool;
    EMC_COOLANT_STAT coolant;
    EMC_AUX_STAT aux;

};

// EMC is aggregate of EMC_TASK, EMC_TRAJ, EMC_IO, etc.

// EMC command base class
class EMC_CMD_MSG:public RCS_CMD_MSG {
  public:
    EMC_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

// EMC status base class

class EMC_STAT_MSG:public RCS_STAT_MSG {
  public:
    EMC_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class EMC_STAT:public EMC_STAT_MSG {
  public:
    EMC_STAT();

    // For internal NML/CMS use only.
    // Sub-class update() calls base-class update()
    // cppcheck-suppress duplInheritedMember
    void update(CMS * cms);

    // the top-level EMC_TASK status class
    EMC_TASK_STAT task;

    // subordinate status classes
    EMC_MOTION_STAT motion;
    EMC_IO_STAT io;

    int debug;			// copy of EMC_DEBUG global
};

/*
   Declarations of EMC status class implementations, for major subsystems.
   These are defined in the appropriate main() files, and referenced
   by code in other files to get EMC status.
   */


#endif
