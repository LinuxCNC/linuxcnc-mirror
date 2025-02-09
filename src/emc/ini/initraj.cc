/********************************************************************
* Description: initraj.cc
*   INI file initialization for trajectory level
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include <stdio.h>		// NULL
#include <stdlib.h>		// atol()
#include <string.h>		// strlen()
#include <ctype.h>		// isspace()

#include "emc.hh"
#include "emcpos.h"             // EmcPose
#include "rcs_print.hh"
#include "posemath.h"		// PM_POSE, PM_RPY
#include "emcIniFile.hh"
#include "initraj.hh"		// these decls
#include "emcglb.h"		/*! \todo TRAVERSE_RATE (FIXME) */
#include "inihal.hh"
#include <rtapi_string.h>

extern value_inihal_data old_inihal_data;

/*
  loadKins()

  JOINTS <int>                  number of joints (DOF) in system

  calls:

  emcTrajSetJoints(int joints);
*/

static int loadKins(EmcIniFile *trajInifile)
{
    trajInifile->EnableExceptions(EmcIniFile::ERR_CONVERSION);

    try {
	int joints = 0;
	trajInifile->Find(&joints, "JOINTS", "KINS");

        if (0 != emcTrajSetJoints(joints)) {
            return -1;
        }
    }
   
    catch (EmcIniFile::Exception &e) {
        e.Print();
        return -1;
    }
    
    return 0;
}


/*
  loadTraj()

  Loads INI file params for traj

  COORDINATES <char[]>            axes in system
  LINEAR_UNITS <float>            units per mm
  ANGULAR_UNITS <float>           units per degree
  DEFAULT_LINEAR_VELOCITY <float> default linear velocity
  MAX_LINEAR_VELOCITY <float>     max linear velocity
  DEFAULT_LINEAR_ACCELERATION <float> default linear acceleration
  MAX_LINEAR_ACCELERATION <float>     max linear acceleration

  calls:

  emcTrajSetAxes(int axes, int axismask);
  emcTrajSetUnits(double linearUnits, double angularUnits);
  emcTrajSetVelocity(double vel, double ini_maxvel);
  emcTrajSetAcceleration(double acc);
  emcTrajSetMaxVelocity(double vel);
  emcTrajSetMaxAcceleration(double acc);
  */

static int loadTraj(EmcIniFile *trajInifile)
{
    EmcLinearUnits linearUnits;
    EmcAngularUnits angularUnits;
    double vel;
    double acc;

    trajInifile->EnableExceptions(EmcIniFile::ERR_CONVERSION);

    try{
		int spindles = 1;
		trajInifile->Find(&spindles, "SPINDLES", "TRAJ");
		if (0 != emcTrajSetSpindles(spindles)) {
			return -1;
		}
    }

    catch (EmcIniFile::Exception &e) {
        e.Print();
        return -1;
    }

    try{
	int axismask = 0;
	const char *coord = trajInifile->Find("COORDINATES", "TRAJ");
	if(coord) {
	    if(strchr(coord, 'x') || strchr(coord, 'X')) {
	         axismask |= 1;
            }
	    if(strchr(coord, 'y') || strchr(coord, 'Y')) {
	         axismask |= 2;
            }
	    if(strchr(coord, 'z') || strchr(coord, 'Z')) {
	         axismask |= 4;
            }
	    if(strchr(coord, 'a') || strchr(coord, 'A')) {
	         axismask |= 8;
            }
	    if(strchr(coord, 'b') || strchr(coord, 'B')) {
	         axismask |= 16;
            }
	    if(strchr(coord, 'c') || strchr(coord, 'C')) {
	         axismask |= 32;
            }
	    if(strchr(coord, 'u') || strchr(coord, 'U')) {
	         axismask |= 64;
            }
	    if(strchr(coord, 'v') || strchr(coord, 'V')) {
	         axismask |= 128;
            }
	    if(strchr(coord, 'w') || strchr(coord, 'W')) {
	         axismask |= 256;
            }
	} else {
	    axismask = 1 | 2 | 4;		// default: XYZ machine
	}
        if (0 != emcTrajSetAxes(axismask)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetAxes\n");
            }
            return -1;
        }

        linearUnits = 0;
        trajInifile->FindLinearUnits(&linearUnits, "LINEAR_UNITS", "TRAJ");
        angularUnits = 0;
        trajInifile->FindAngularUnits(&angularUnits, "ANGULAR_UNITS", "TRAJ");
        if (0 != emcTrajSetUnits(linearUnits, angularUnits)) {
            rcs_print("emcTrajSetUnits failed to set "
                      "[TRAJ]LINEAR_UNITS or [TRAJ]ANGULAR_UNITS\n");
            return -1;
        }

        vel = 1.0;
        trajInifile->Find(&vel, "DEFAULT_LINEAR_VELOCITY", "TRAJ");
        old_inihal_data.traj_default_velocity = vel;

        // and set dynamic value
        if (0 != emcTrajSetVelocity(0, vel)) { //default velocity on startup 0
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetVelocity\n");
            }
            return -1;
        }

        vel = 1e99; // by default, use AXIS limit
        trajInifile->Find(&vel, "MAX_LINEAR_VELOCITY", "TRAJ");
        // XXX CJR merge question: Set something in TrajConfig here?
        old_inihal_data.traj_max_velocity = vel;

        // and set dynamic value
        if (0 != emcTrajSetMaxVelocity(vel)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetMaxVelocity\n");
            }
            return -1;
        }

        acc = 1e99; // let the axis values apply
        trajInifile->Find(&acc, "DEFAULT_LINEAR_ACCELERATION", "TRAJ");
        if (0 != emcTrajSetAcceleration(acc)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetAcceleration\n");
            }
            return -1;
        }
        old_inihal_data.traj_default_acceleration = acc;

        acc = 1e99; // let the axis values apply
        trajInifile->Find(&acc, "MAX_LINEAR_ACCELERATION", "TRAJ");
        if (0 != emcTrajSetMaxAcceleration(acc)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetMaxAcceleration\n");
            }
            return -1;
        }
        old_inihal_data.traj_max_acceleration = acc;

        int arcBlendEnable = 1;
        int arcBlendFallbackEnable = 0;
        int arcBlendOptDepth = 50;
        int arcBlendGapCycles = 4;
        double arcBlendRampFreq = 100.0;
        double arcBlendTangentKinkRatio = 0.1;

        trajInifile->Find(&arcBlendEnable, "ARC_BLEND_ENABLE", "TRAJ");
        trajInifile->Find(&arcBlendFallbackEnable, "ARC_BLEND_FALLBACK_ENABLE", "TRAJ");
        trajInifile->Find(&arcBlendOptDepth, "ARC_BLEND_OPTIMIZATION_DEPTH", "TRAJ");
        trajInifile->Find(&arcBlendGapCycles, "ARC_BLEND_GAP_CYCLES", "TRAJ");
        trajInifile->Find(&arcBlendRampFreq, "ARC_BLEND_RAMP_FREQ", "TRAJ");
        trajInifile->Find(&arcBlendTangentKinkRatio, "ARC_BLEND_KINK_RATIO", "TRAJ");

        if (0 != emcSetupArcBlends(arcBlendEnable, arcBlendFallbackEnable,
                    arcBlendOptDepth, arcBlendGapCycles, arcBlendRampFreq, arcBlendTangentKinkRatio)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcSetupArcBlends\n");
            }
            return -1;
        } 

        old_inihal_data.traj_arc_blend_enable = arcBlendEnable;
        old_inihal_data.traj_arc_blend_fallback_enable = arcBlendFallbackEnable;
        old_inihal_data.traj_arc_blend_optimization_depth = arcBlendOptDepth;
        old_inihal_data.traj_arc_blend_gap_cycles = arcBlendGapCycles;
        old_inihal_data.traj_arc_blend_ramp_freq = arcBlendRampFreq;
        old_inihal_data.traj_arc_blend_tangent_kink_ratio = arcBlendTangentKinkRatio;
        //TODO update inihal

        double maxFeedScale = 1.0;
        trajInifile->Find(&maxFeedScale, "MAX_FEED_OVERRIDE", "DISPLAY");

        if (0 != emcSetMaxFeedOverride(maxFeedScale)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcSetMaxFeedOverride\n");
            }
            return -1;
        } 
        
        int j_inhibit = 0;
        int h_inhibit = 0;
        trajInifile->Find(&j_inhibit, "NO_PROBE_JOG_ERROR", "TRAJ");
        trajInifile->Find(&h_inhibit, "NO_PROBE_HOME_ERROR", "TRAJ");
        if (0 != emcSetProbeErrorInhibit(j_inhibit, h_inhibit)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcSetProbeErrorInhibit\n");
            }
            return -1;
        }
    }

    catch (EmcIniFile::Exception &e) {
        e.Print();
        return -1;
    }
    try{
        const char *inistring;
        unsigned char coordinateMark[6] = { 1, 1, 1, 0, 0, 0 };
        int t;
        int len;
        char homes[LINELEN];
        char home[LINELEN];
        EmcPose homePose = { {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        double d;
        if (NULL != (inistring = trajInifile->Find("HOME", "TRAJ"))) {
            // [TRAJ]HOME is important for genhexkins.c kinetmaticsForward()
            // and probably other non-identity kins that solve the forward
            // kinematics with an iterative algorithm when the homePose
            // is not all zeros

            // found it, now interpret it according to coordinateMark[]
            rtapi_strxcpy(homes, inistring);
            len = 0;
            for (t = 0; t < 6; t++) {
                if (!coordinateMark[t]) {
                    continue;    // position t at index of next non-zero mark
                }
                // there is a mark, so read the string for a value
                if (1 == sscanf(&homes[len], "%254s", home) &&
                    1 == sscanf(home, "%lf", &d)) {
                    // got an entry, index into coordinateMark[] is 't'
                    if (t == 0)
                        homePose.tran.x = d;
                    else if (t == 1)
                        homePose.tran.y = d;
                    else if (t == 2)
                        homePose.tran.z = d;
                    else if (t == 3)
                        homePose.a = d;
                    else if (t == 4)
                        homePose.b = d;
                    else if (t == 5)
                        homePose.c = d;
/*
 * The following have no effect. The loop only counts [0..5].
                    else if (t == 6)
                        homePose.u = d;
                    else if (t == 7)
                        homePose.v = d;
                    else
                        homePose.w = d;
*/

                    // position string ptr past this value
                    len += strlen(home);
                    // and at start of next value
                    while ((len < LINELEN) && (homes[len] == ' ' || homes[len] == '\t')) {
                        len++;
                    }
                    if (len >= LINELEN) {
                        break;    // out of for loop
                    }
                } else {
                    // badly formatted entry
                    rcs_print("invalid INI file value for [TRAJ] HOME: %s\n",
                          inistring);
                        return -1;
                }
            }  // end of for-loop on coordinateMark[]
        }
        if (0 != emcTrajSetHome(homePose)) {
            if (emc_debug & EMC_DEBUG_CONFIG) {
                rcs_print("bad return value from emcTrajSetHome\n");
            }
            return -1;
        }
     } //try

    catch (EmcIniFile::Exception &e) {
        e.Print();
        return -1;
    }
    return 0;
}

/*
  iniTraj(const char *filename)

  Loads INI file parameters for trajectory, from [TRAJ] section
 */
int iniTraj(const char *filename)
{
    EmcIniFile trajInifile;

    if (trajInifile.Open(filename) == false) {
	return -1;
    }
    // load trajectory values
    if (0 != loadKins(&trajInifile)) {
	return -1;
    }
    // load trajectory values
    if (0 != loadTraj(&trajInifile)) {
	return -1;
    }

    return 0;
}
