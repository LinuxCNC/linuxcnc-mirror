/********************************************************************
* Description: emccanon.cc
*   Canonical definitions for 3-axis NC application
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/
/*

  Notes:

  Units
  -----
  Values are stored internally as mm and degree units, e.g, program
  offsets, end point, tool length offset.  These are "internal
  units". "External units" are the units used by the EMC motion planner.
  All lengths and units output by the interpreter are converted to
  internal units here, using FROM_PROG_LEN,ANG, and then
  TO_EXT_LEN(),ANG are called to convert these to external units.

  Tool Length Offsets
  -------------------
  The interpreter does not subtract off tool length offsets. It calls
  USE_TOOL_LENGTH_OFFSETS(length), which we record here and apply to
  all appropriate values subsequently.

  NURBS
  -----
  The code in this file for nurbs calculations is from University of Palermo.
  The publications can be found at: http://wiki.linuxcnc.org/cgi-bin/wiki.pl?NURBS
  AMST08_art837759.pdf and ECME14.pdf

  1: 
  M. Leto, R. Licari, E. Lo Valvo1 , M. Piacentini:
  CAD/CAM INTEGRATION FOR NURBS PATH INTERPOLATION ON PC BASED REAL-TIME NUMERICAL CONTROL
  Proceedings of AMST 2008 Conference, 2008, pp. 223-233

  2:
  ERNESTO LO VALVO, STEFANO DRAGO:
  An Efficient NURBS Path Generator for a Open Source CNC
  Recent Advances in Mechanical Engineering (pp.173-180). WSEAS Press

  The code from University of Palermo is modified to work on planes xy, yz and zx by Joachim Franek
  */

#include "config.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>		// strncpy()
#include <ctype.h>		// isspace()
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "canon.hh"
#include "canon_position.hh"		// data type for a machine position
#include "interpl.hh"		// interp_list
#include "emcglb.h"		// TRAJ_MAX_VELOCITY
#include <rtapi_string.h>
#include "modal_state.hh"
#include "tooldata.hh"
#include <algorithm>

//#define EMCCANON_DEBUG

//Simple compile-time debug macro
#ifdef EMCCANON_DEBUG
#define canon_debug(...) printf(__VA_ARGS__)
#else
#define canon_debug(...)
#endif

/*
  Origin offsets, length units, and active plane are all maintained
  here in this file. Controller runs in absolute mode, and does not
  have plane select concept.

  programOrigin is stored in mm always, and converted when set or read.
  When it's applied to positions, convert positions to mm units first
  and then add programOrigin.

  Units are then converted from mm to external units, as reported by
  the GET_EXTERNAL_LENGTH_UNITS() function.
  */

static CanonConfig_t canon;

static int debug_velacc = 0;

static StateTag _tag;

void UPDATE_TAG(const StateTag& tag) {
    canon_debug("--Got UPDATE_TAG: %d--\n",tag.fields[GM_FIELD_LINE_NUMBER]);
    _tag = tag;
}

/* macros for converting internal (mm/deg) units to external units */
#define TO_EXT_LEN(mm) ((mm) * GET_EXTERNAL_LENGTH_UNITS())
#define TO_EXT_ANG(deg) ((deg) * GET_EXTERNAL_ANGLE_UNITS())

/* macros for converting external units to internal (mm/deg) units */
#define FROM_EXT_LEN(ext) ((ext) / GET_EXTERNAL_LENGTH_UNITS())
#define FROM_EXT_ANG(ext) ((ext) / GET_EXTERNAL_ANGLE_UNITS())

/* macros for converting internal (mm/deg) units to program units */
#define TO_PROG_LEN(mm) ((mm) / (canon.lengthUnits == CANON_UNITS_INCHES ? 25.4 : canon.lengthUnits == CANON_UNITS_CM ? 10.0 : 1.0))
#define TO_PROG_ANG(deg) (deg)

/* macros for converting program units to internal (mm/deg) units */
#define FROM_PROG_LEN(prog) ((prog) * (canon.lengthUnits == CANON_UNITS_INCHES ? 25.4 : canon.lengthUnits == CANON_UNITS_CM ? 10.0 : 1.0))
#define FROM_PROG_ANG(prog) (prog)

/* Certain axes are periodic.  Hardcode this for now */
#define IS_PERIODIC(axisnum) \
    ((axisnum) == 3 || (axisnum) == 4 || (axisnum) == 5)

// this doesn't quite work yet: disable
#undef IS_PERIODIC
#define IS_PERIODIC(axisnum) (0)

#define AXIS_PERIOD(axisnum) (IS_PERIODIC(axisnum) ? 360 : 0)

//KLUDGE kinematic data struct (instead of returning a single float value)
//FIXME This should really be refactored into a more general structure, but this
//means tearing up the getStraightXXX functions, which probably means
//converting to canon_position operators
struct VelData {
    double tmax;
    double vel;
    double dtot;
};

struct AccelData{
    double tmax;
    double acc;
    double dtot;
};

static PM_QUATERNION quat(1, 0, 0, 0);

static void flush_segments(void);


/**
 * Add the the given line number and append the message to the interp list.
 * Note that the append function takes the message by reference, so this also
 * needs to have the message passed in by reference or it barfs.
 */
static inline void tag_and_send(std::unique_ptr<EMC_TRAJ_CMD_MSG> &&msg, StateTag const &tag) {
    msg->tag = tag;
    interp_list.append(std::move(msg));
}

/*
  These decls were from the old 3-axis canon.hh, and refer functions
  defined here that are used for convenience but no longer have decls
  in the 6-axis canon.hh. So, we declare them here now.
*/

#ifndef D2R
#define D2R(r) ((r)*M_PI/180.0)
#endif

static void rotate(double &x, double &y, double theta) {
    double xx, yy;
    double t = D2R(theta);
    xx = x;
    yy = y;
    x = xx * cos(t) - yy * sin(t); 
    y = xx * sin(t) + yy * cos(t);
}


/**
 * Implementation of planar rotation for a 3D vector.
 * This is basically a shortcut for "rotate" when the values are stored in a
 * cartesian vector.
 * The use of static "xy_rotation" is ugly here, but is at least consistent.
 */
static void to_rotated(PM_CARTESIAN &vec) {
    rotate(vec.x,vec.y,canon.xy_rotation);
}
#if 0
static void from_rotated(PM_CARTESIAN &vec) {
    rotate(vec.x,vec.y,-canon.xy_rotation);
}
#endif
static void rotate_and_offset(CANON_POSITION & pos) {

    pos += canon.g92Offset;

    rotate(pos.x, pos.y, canon.xy_rotation);

    pos += canon.g5xOffset;

    pos += canon.toolOffset;
}

static void rotate_and_offset_xyz(PM_CARTESIAN & xyz) {

    xyz += canon.g92Offset.xyz();

    rotate(xyz.x, xyz.y, canon.xy_rotation);

    xyz += canon.g5xOffset.xyz();

    xyz += PM_CARTESIAN(canon.toolOffset.tran.x,
			canon.toolOffset.tran.y,
			canon.toolOffset.tran.z);
}

static CANON_POSITION unoffset_and_unrotate_pos(const CANON_POSITION& pos) {
    CANON_POSITION res;

    res = pos;

    res -= canon.toolOffset;
    
    res -= canon.g5xOffset;

    rotate(res.x, res.y, -canon.xy_rotation);

    res -= canon.g92Offset;

    return res;
}

static void rotate_and_offset_pos(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w) {
    x += canon.g92Offset.x;
    y += canon.g92Offset.y;
    z += canon.g92Offset.z;
    a += canon.g92Offset.a;
    b += canon.g92Offset.b;
    c += canon.g92Offset.c;
    u += canon.g92Offset.u;
    v += canon.g92Offset.v;
    w += canon.g92Offset.w;

    rotate(x, y, canon.xy_rotation);

    x += canon.g5xOffset.x;
    y += canon.g5xOffset.y;
    z += canon.g5xOffset.z;
    a += canon.g5xOffset.a;
    b += canon.g5xOffset.b;
    c += canon.g5xOffset.c;
    u += canon.g5xOffset.u;
    v += canon.g5xOffset.v;
    w += canon.g5xOffset.w;

    x += canon.toolOffset.tran.x;
    y += canon.toolOffset.tran.y;
    z += canon.toolOffset.tran.z;
    a += canon.toolOffset.a;
    b += canon.toolOffset.b;
    c += canon.toolOffset.c;
    u += canon.toolOffset.u;
    v += canon.toolOffset.v;
    w += canon.toolOffset.w;
}


static CANON_POSITION unoffset_and_unrotate_pos(const EmcPose& pos) {
    CANON_POSITION res(pos);
    return unoffset_and_unrotate_pos(res);
}

static void from_prog(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w) {
    x = FROM_PROG_LEN(x);
    y = FROM_PROG_LEN(y);
    z = FROM_PROG_LEN(z);
    a = FROM_PROG_ANG(a);
    b = FROM_PROG_ANG(b);
    c = FROM_PROG_ANG(c);
    u = FROM_PROG_LEN(u);
    v = FROM_PROG_LEN(v);
    w = FROM_PROG_LEN(w);
}

static void from_prog(CANON_POSITION &pos) {
    pos.x = FROM_PROG_LEN(pos.x);
    pos.y = FROM_PROG_LEN(pos.y);
    pos.z = FROM_PROG_LEN(pos.z);
    pos.a = FROM_PROG_ANG(pos.a);
    pos.b = FROM_PROG_ANG(pos.b);
    pos.c = FROM_PROG_ANG(pos.c);
    pos.u = FROM_PROG_LEN(pos.u);
    pos.v = FROM_PROG_LEN(pos.v);
    pos.w = FROM_PROG_LEN(pos.w);
}

static void from_prog_len(PM_CARTESIAN &vec) {
    vec.x = FROM_PROG_LEN(vec.x);
    vec.y = FROM_PROG_LEN(vec.y);
    vec.z = FROM_PROG_LEN(vec.z);
}
#if 0
static void to_ext(double &x, double &y, double &z, double &a, double &b, double &c, double &u, double &v, double &w) {
    x = TO_EXT_LEN(x);
    y = TO_EXT_LEN(y);
    z = TO_EXT_LEN(z);
    a = TO_EXT_ANG(a);
    b = TO_EXT_ANG(b);
    c = TO_EXT_ANG(c);
    u = TO_EXT_LEN(u);
    v = TO_EXT_LEN(v);
    w = TO_EXT_LEN(w);
}

static void to_ext(CANON_POSITION & pos) {
    pos.x=TO_EXT_LEN(pos.x);
    pos.y=TO_EXT_LEN(pos.y);
    pos.z=TO_EXT_LEN(pos.z);
    pos.a=TO_EXT_ANG(pos.a);
    pos.b=TO_EXT_ANG(pos.b);
    pos.c=TO_EXT_ANG(pos.c);
    pos.u=TO_EXT_LEN(pos.u);
    pos.v=TO_EXT_LEN(pos.v);
    pos.w=TO_EXT_LEN(pos.w);
}
#endif

static PM_CARTESIAN to_ext_len(const PM_CARTESIAN & pos) {
    PM_CARTESIAN ret;
    ret.x = TO_EXT_LEN(pos.x);
    ret.y = TO_EXT_LEN(pos.y);
    ret.z = TO_EXT_LEN(pos.z);
    return ret;
}

static EmcPose to_ext_pose(double x, double y, double z, double a, double b, double c, double u, double v, double w) {
    EmcPose result;
    result.tran.x = TO_EXT_LEN(x);
    result.tran.y = TO_EXT_LEN(y);
    result.tran.z = TO_EXT_LEN(z);
    result.a = TO_EXT_ANG(a);
    result.b = TO_EXT_ANG(b);
    result.c = TO_EXT_ANG(c);
    result.u = TO_EXT_LEN(u);
    result.v = TO_EXT_LEN(v);
    result.w = TO_EXT_LEN(w);
    return result;
}

static EmcPose to_ext_pose(const CANON_POSITION & pos) {
    EmcPose result;
    result.tran.x = TO_EXT_LEN(pos.x);
    result.tran.y = TO_EXT_LEN(pos.y);
    result.tran.z = TO_EXT_LEN(pos.z);
    result.a = TO_EXT_ANG(pos.a);
    result.b = TO_EXT_ANG(pos.b);
    result.c = TO_EXT_ANG(pos.c);
    result.u = TO_EXT_LEN(pos.u);
    result.v = TO_EXT_LEN(pos.v);
    result.w = TO_EXT_LEN(pos.w);
    return result;
}

static void to_prog(CANON_POSITION &e) {
    e.x = TO_PROG_LEN(e.x);
    e.y = TO_PROG_LEN(e.y);
    e.z = TO_PROG_LEN(e.z);
    e.a = TO_PROG_ANG(e.a);
    e.b = TO_PROG_ANG(e.b);
    e.c = TO_PROG_ANG(e.c);
    e.u = TO_PROG_LEN(e.u);
    e.v = TO_PROG_LEN(e.v);
    e.w = TO_PROG_LEN(e.w);
}

static int axis_valid(int n) {
    return emcStatus->motion.traj.axis_mask & (1<<n);
}

static void canonUpdateEndPoint(double x, double y, double z, 
                                double a, double b, double c,
                                double u, double v, double w)
{
    canon.endPoint.x = x;
    canon.endPoint.y = y;
    canon.endPoint.z = z;

    canon.endPoint.a = a;
    canon.endPoint.b = b;
    canon.endPoint.c = c;

    canon.endPoint.u = u;
    canon.endPoint.v = v;
    canon.endPoint.w = w;
}

static void canonUpdateEndPoint(const CANON_POSITION & pos)
{
    canon.endPoint = pos;
}

/* External call to update the canon end point.
   Called by emctask during skipping of lines (run-from-line) */
void CANON_UPDATE_END_POINT(double x, double y, double z, 
			    double a, double b, double c, 
			    double u, double v, double w)
{
    canonUpdateEndPoint(FROM_PROG_LEN(x),FROM_PROG_LEN(y),FROM_PROG_LEN(z),
    			FROM_PROG_ANG(a),FROM_PROG_ANG(b),FROM_PROG_ANG(c),
			FROM_PROG_LEN(u),FROM_PROG_LEN(v),FROM_PROG_LEN(w));
}

static double toExtVel(double vel) {
    if (canon.cartesian_move && !canon.angular_move) {
	return TO_EXT_LEN(vel);
    } else if (!canon.cartesian_move && canon.angular_move) {
	return TO_EXT_ANG(vel);
    } else if (canon.cartesian_move && canon.angular_move) {
	return TO_EXT_LEN(vel);
    } else { //seems this case was forgotten, neither linear, neither angular move (we are only sending vel)
	return TO_EXT_LEN(vel);
    }	
}

static double toExtAcc(double acc) { return toExtVel(acc); }

static void send_g5x_msg(int index) {
    flush_segments();

    /* append it to interp list so it gets updated at the right time, not at
       read-ahead time */
    auto set_g5x_msg = std::make_unique<EMC_TRAJ_SET_G5X>();

    set_g5x_msg->g5x_index = index;

    set_g5x_msg->origin = to_ext_pose(canon.g5xOffset);

    for (int s = 0; s < emcStatus->motion.traj.spindles; s++){
        if(canon.spindle[s].css_maximum) {
            SET_SPINDLE_SPEED(s, canon.spindle[s].speed);
        }
    }
    interp_list.append(std::move(set_g5x_msg));
}

static void send_g92_msg(void) {
    flush_segments();

    /* append it to interp list so it gets updated at the right time, not at
       read-ahead time */
    auto set_g92_msg = std::make_unique<EMC_TRAJ_SET_G92>();

    set_g92_msg->origin = to_ext_pose(canon.g92Offset);

    for (int s = 0; s < emcStatus->motion.traj.spindles; s++){
        if(canon.spindle[s].css_maximum) {
            SET_SPINDLE_SPEED(s, canon.spindle[s].speed);
        }
    }
    interp_list.append(std::move(set_g92_msg));
}

void SET_XY_ROTATION(double t) {
    auto sr = std::make_unique<EMC_TRAJ_SET_ROTATION>();
    sr->rotation = t;
    interp_list.append(std::move(sr));

    canon.xy_rotation = t;
}

void SET_G5X_OFFSET(int index,
                    double x, double y, double z,
                    double a, double b, double c,
                    double u, double v, double w)
{
    CANON_POSITION pos(x,y,z,a,b,c,u,v,w);
    from_prog(pos);
    /* convert to mm units */
    canon.g5xOffset = pos;

    send_g5x_msg(index);
}

void SET_G92_OFFSET(double x, double y, double z,
                    double a, double b, double c,
                    double u, double v, double w) {
    /* convert to mm units */
    CANON_POSITION pos(x,y,z,a,b,c,u,v,w);
    from_prog(pos);

    canon.g92Offset = pos;

    send_g92_msg();
}

void USE_LENGTH_UNITS(CANON_UNITS in_unit)
{
    canon.lengthUnits = in_unit;

    emcStatus->task.programUnits = in_unit;
}

/* Free Space Motion */
void SET_TRAVERSE_RATE(double /*rate*/)
{
    // nothing need be done here
}

void SET_FEED_MODE(int spindle, int mode) {
    flush_segments();
    canon.feed_mode = mode;
    canon.spindle_num = spindle;
    if(canon.feed_mode == 0) STOP_SPEED_FEED_SYNCH();
}

void SET_FEED_RATE(double rate)
{

    if(canon.feed_mode) {
	START_SPEED_FEED_SYNCH(canon.spindle_num, rate, 1);
	canon.linearFeedRate = rate;
    } else {
	/* convert from /min to /sec */
	rate /= 60.0;


	/* convert to traj units (mm & deg) if needed */
	double newLinearFeedRate = FROM_PROG_LEN(rate),
	       newAngularFeedRate = FROM_PROG_ANG(rate);

	if(newLinearFeedRate != canon.linearFeedRate
		|| newAngularFeedRate != canon.angularFeedRate)
	    flush_segments();

	canon.linearFeedRate = newLinearFeedRate;
	canon.angularFeedRate = newAngularFeedRate;
    }
}

void SET_FEED_REFERENCE(CANON_FEED_REFERENCE /*reference*/)
{
    // nothing need be done here
}

/**
 * Get the shortest linear axis displacement that the TP can handle as a discrete move.
 *
 * If this looks dirty, it's because it is. Canon runs in its own units, but
 * the TP uses user units. Therefore, the minimum displacement has to be
 * computed the same way, with the same threshold, or short moves do strange
 * things (accel violations or infinite pauses).
 *
 * @todo revisit this when the TP is overhauled to use a consistent set of internal units.
 */
static double getMinLinearDisplacement()
{
    return FROM_EXT_LEN(CART_FUZZ);
}

/**
 * Equivalent of getMinLinearDisplacement for rotary axes.
 */
static double getMinAngularDisplacement()
{
    return FROM_EXT_ANG(CART_FUZZ);
}

/**
 * Apply the minimum displacement check to each axis delta.
 *
 * Checks that the axis is valid / active, and looks up the appropriate minimum
 * displacement for the axis type and user units.
 */
static void applyMinDisplacement(double &dx,
                                 double &dy,
                                 double &dz,
                                 double &da,
                                 double &db,
                                 double &dc,
                                 double &du,
                                 double &dv,
                                 double &dw
                                 )
{
    const double tiny_linear = getMinLinearDisplacement();
    const double tiny_angular = getMinAngularDisplacement();
    if(!axis_valid(0) || dx < tiny_linear) dx = 0.0;
    if(!axis_valid(1) || dy < tiny_linear) dy = 0.0;
    if(!axis_valid(2) || dz < tiny_linear) dz = 0.0;
    if(!axis_valid(3) || da < tiny_angular) da = 0.0;
    if(!axis_valid(4) || db < tiny_linear) db = 0.0;
    if(!axis_valid(5) || dc < tiny_linear) dc = 0.0;
    if(!axis_valid(6) || du < tiny_linear) du = 0.0;
    if(!axis_valid(7) || dv < tiny_linear) dv = 0.0;
    if(!axis_valid(8) || dw < tiny_linear) dw = 0.0;
}


/**
 * Get the limiting acceleration for a displacement from the current position to the given position.
 * returns a single acceleration that is the minimum of all axis accelerations.
 */
static AccelData getStraightAcceleration(double x, double y, double z,
                               double a, double b, double c,
                               double u, double v, double w)
{
    double dx, dy, dz, du, dv, dw, da, db, dc;
    double tx, ty, tz, tu, tv, tw, ta, tb, tc;
    AccelData out;

    out.acc = 0.0; // if a move to nowhere
    out.tmax = 0.0;
    out.dtot = 0.0;

    // Compute absolute travel distance for each axis:
    dx = fabs(x - canon.endPoint.x);
    dy = fabs(y - canon.endPoint.y);
    dz = fabs(z - canon.endPoint.z);
    da = fabs(a - canon.endPoint.a);
    db = fabs(b - canon.endPoint.b);
    dc = fabs(c - canon.endPoint.c);
    du = fabs(u - canon.endPoint.u);
    dv = fabs(v - canon.endPoint.v);
    dw = fabs(w - canon.endPoint.w);

    applyMinDisplacement(dx, dy, dz, da, db, dc, du, dv, dw);

    if(debug_velacc) 
        printf("getStraightAcceleration dx %g dy %g dz %g da %g db %g dc %g du %g dv %g dw %g ", 
               dx, dy, dz, da, db, dc, du, dv, dw);

    // Figure out what kind of move we're making.  This is used to determine
    // the units of vel/acc.
    if (dx <= 0.0 && dy <= 0.0 && dz <= 0.0 &&
        du <= 0.0 && dv <= 0.0 && dw <= 0.0) {
	canon.cartesian_move = 0;
    } else {
	canon.cartesian_move = 1;
    }
    if (da <= 0.0 && db <= 0.0 && dc <= 0.0) {
	canon.angular_move = 0;
    } else {
	canon.angular_move = 1;
    }

    // Pure linear move:
    if (canon.cartesian_move && !canon.angular_move) {
	tx = dx? (dx / FROM_EXT_LEN(emcAxisGetMaxAcceleration(0))): 0.0;
	ty = dy? (dy / FROM_EXT_LEN(emcAxisGetMaxAcceleration(1))): 0.0;
	tz = dz? (dz / FROM_EXT_LEN(emcAxisGetMaxAcceleration(2))): 0.0;
	tu = du? (du / FROM_EXT_LEN(emcAxisGetMaxAcceleration(6))): 0.0;
	tv = dv? (dv / FROM_EXT_LEN(emcAxisGetMaxAcceleration(7))): 0.0;
	tw = dw? (dw / FROM_EXT_LEN(emcAxisGetMaxAcceleration(8))): 0.0;
        out.tmax = std::max({tx, ty ,tz});
        out.tmax = std::max({tu, tv, tw, out.tmax});

        if(dx || dy || dz)
            out.dtot = sqrt(dx * dx + dy * dy + dz * dz);
        else
            out.dtot = sqrt(du * du + dv * dv + dw * dw);
        
	if (out.tmax > 0.0) {
	    out.acc = out.dtot / out.tmax;
	}
    }
    // Pure angular move:
    else if (!canon.cartesian_move && canon.angular_move) {
	ta = da? (da / FROM_EXT_ANG(emcAxisGetMaxAcceleration(3))): 0.0;
	tb = db? (db / FROM_EXT_ANG(emcAxisGetMaxAcceleration(4))): 0.0;
	tc = dc? (dc / FROM_EXT_ANG(emcAxisGetMaxAcceleration(5))): 0.0;
        out.tmax = std::max({ta, tb, tc});

	out.dtot = sqrt(da * da + db * db + dc * dc);
	if (out.tmax > 0.0) {
	    out.acc = out.dtot / out.tmax;
	}
    }
    // Combination angular and linear move:
    else if (canon.cartesian_move && canon.angular_move) {
	tx = dx? (dx / FROM_EXT_LEN(emcAxisGetMaxAcceleration(0))): 0.0;
	ty = dy? (dy / FROM_EXT_LEN(emcAxisGetMaxAcceleration(1))): 0.0;
	tz = dz? (dz / FROM_EXT_LEN(emcAxisGetMaxAcceleration(2))): 0.0;
	ta = da? (da / FROM_EXT_ANG(emcAxisGetMaxAcceleration(3))): 0.0;
	tb = db? (db / FROM_EXT_ANG(emcAxisGetMaxAcceleration(4))): 0.0;
	tc = dc? (dc / FROM_EXT_ANG(emcAxisGetMaxAcceleration(5))): 0.0;
	tu = du? (du / FROM_EXT_LEN(emcAxisGetMaxAcceleration(6))): 0.0;
	tv = dv? (dv / FROM_EXT_LEN(emcAxisGetMaxAcceleration(7))): 0.0;
	tw = dw? (dw / FROM_EXT_LEN(emcAxisGetMaxAcceleration(8))): 0.0;
        out.tmax = std::max({tx, ty, tz,
                    ta, tb, tc,
                    tu, tv, tw});

        if(debug_velacc)
            printf("getStraightAcceleration t^2 tx %g ty %g tz %g ta %g tb %g tc %g tu %g tv %g tw %g\n",
                   tx, ty, tz, ta, tb, tc, tu, tv, tw);
/*  According to NIST IR6556 Section 2.1.2.5 Paragraph A
    a combnation move is handled like a linear move, except
    that the angular axes are allowed sufficient time to
    complete their motion coordinated with the motion of
    the linear axes.
*/
        if(dx || dy || dz)
            out.dtot = sqrt(dx * dx + dy * dy + dz * dz);
        else
            out.dtot = sqrt(du * du + dv * dv + dw * dw);

	if (out.tmax > 0.0) {
	    out.acc = out.dtot / out.tmax;
	}
    }
    if(debug_velacc) 
        printf("cartesian %d ang %d acc %g\n", canon.cartesian_move, canon.angular_move, out.acc);
    return out;
}

static AccelData getStraightAcceleration(const CANON_POSITION& pos)
{

    return getStraightAcceleration(pos.x,
            pos.y,
            pos.z,
            pos.a,
            pos.b,
            pos.c,
            pos.u,
            pos.v,
            pos.w);
}

static VelData getStraightVelocity(double x, double y, double z,
			   double a, double b, double c,
                           double u, double v, double w)
{
    double dx, dy, dz, da, db, dc, du, dv, dw;
    double tx, ty, tz, ta, tb, tc, tu, tv, tw;
    VelData out;

/* If we get a move to nowhere (!canon.cartesian_move && !canon.angular_move)
   we might as well go there at the canon.linearFeedRate...
*/
    out.vel = canon.linearFeedRate;
    out.tmax = 0;
    out.dtot = 0;

    // Compute absolute travel distance for each axis:
    dx = fabs(x - canon.endPoint.x);
    dy = fabs(y - canon.endPoint.y);
    dz = fabs(z - canon.endPoint.z);
    da = fabs(a - canon.endPoint.a);
    db = fabs(b - canon.endPoint.b);
    dc = fabs(c - canon.endPoint.c);
    du = fabs(u - canon.endPoint.u);
    dv = fabs(v - canon.endPoint.v);
    dw = fabs(w - canon.endPoint.w);

    applyMinDisplacement(dx, dy, dz, da, db, dc, du, dv, dw);

    if(debug_velacc) 
        printf("getStraightVelocity dx %g dy %g dz %g da %g db %g dc %g du %g dv %g dw %g\n",
               dx, dy, dz, da, db, dc, du, dv, dw);

    // Figure out what kind of move we're making:
    if (dx <= 0.0 && dy <= 0.0 && dz <= 0.0 &&
        du <= 0.0 && dv <= 0.0 && dw <= 0.0) {
	canon.cartesian_move = 0;
    } else {
	canon.cartesian_move = 1;
    }
    if (da <= 0.0 && db <= 0.0 && dc <= 0.0) {
	canon.angular_move = 0;
    } else {
	canon.angular_move = 1;
    }

    // Pure linear move:
    if (canon.cartesian_move && !canon.angular_move) {
	tx = dx? fabs(dx / FROM_EXT_LEN(emcAxisGetMaxVelocity(0))): 0.0;
	ty = dy? fabs(dy / FROM_EXT_LEN(emcAxisGetMaxVelocity(1))): 0.0;
	tz = dz? fabs(dz / FROM_EXT_LEN(emcAxisGetMaxVelocity(2))): 0.0;
	tu = du? fabs(du / FROM_EXT_LEN(emcAxisGetMaxVelocity(6))): 0.0;
	tv = dv? fabs(dv / FROM_EXT_LEN(emcAxisGetMaxVelocity(7))): 0.0;
	tw = dw? fabs(dw / FROM_EXT_LEN(emcAxisGetMaxVelocity(8))): 0.0;
        out.tmax = std::max({tx, ty ,tz});
        out.tmax = std::max({tu, tv, tw, out.tmax});

        if(dx || dy || dz)
            out.dtot = sqrt(dx * dx + dy * dy + dz * dz);
        else
            out.dtot = sqrt(du * du + dv * dv + dw * dw);

        if (out.tmax <= 0.0) {
            out.vel = canon.linearFeedRate;
        } else {
            out.vel = out.dtot / out.tmax;
        }
    }
    // Pure angular move:
    else if (!canon.cartesian_move && canon.angular_move) {
	ta = da? fabs(da / FROM_EXT_ANG(emcAxisGetMaxVelocity(3))): 0.0;
	tb = db? fabs(db / FROM_EXT_ANG(emcAxisGetMaxVelocity(4))): 0.0;
	tc = dc? fabs(dc / FROM_EXT_ANG(emcAxisGetMaxVelocity(5))): 0.0;
        out.tmax = std::max({ta, tb, tc});

	out.dtot = sqrt(da * da + db * db + dc * dc);
	if (out.tmax <= 0.0) {
	    out.vel = canon.angularFeedRate;
	} else {
	    out.vel = out.dtot / out.tmax;
	}
    }
    // Combination angular and linear move:
    else if (canon.cartesian_move && canon.angular_move) {
	tx = dx? fabs(dx / FROM_EXT_LEN(emcAxisGetMaxVelocity(0))): 0.0;
	ty = dy? fabs(dy / FROM_EXT_LEN(emcAxisGetMaxVelocity(1))): 0.0;
	tz = dz? fabs(dz / FROM_EXT_LEN(emcAxisGetMaxVelocity(2))): 0.0;
	ta = da? fabs(da / FROM_EXT_ANG(emcAxisGetMaxVelocity(3))): 0.0;
	tb = db? fabs(db / FROM_EXT_ANG(emcAxisGetMaxVelocity(4))): 0.0;
	tc = dc? fabs(dc / FROM_EXT_ANG(emcAxisGetMaxVelocity(5))): 0.0;
	tu = du? fabs(du / FROM_EXT_LEN(emcAxisGetMaxVelocity(6))): 0.0;
	tv = dv? fabs(dv / FROM_EXT_LEN(emcAxisGetMaxVelocity(7))): 0.0;
	tw = dw? fabs(dw / FROM_EXT_LEN(emcAxisGetMaxVelocity(8))): 0.0;
        out.tmax = std::max({tx, ty, tz,
                    ta, tb, tc,
                    tu, tv, tw});

        if(debug_velacc)
            printf("getStraightVelocity times tx %g ty %g tz %g ta %g tb %g tc %g tu %g tv %g tw %g\n",
                    tx, ty, tz, ta, tb, tc, tu, tv, tw);

/*  According to NIST IR6556 Section 2.1.2.5 Paragraph A
    a combnation move is handled like a linear move, except
    that the angular axes are allowed sufficient time to
    complete their motion coordinated with the motion of
    the linear axes.
*/
        if(dx || dy || dz)
            out.dtot = sqrt(dx * dx + dy * dy + dz * dz);
        else
            out.dtot = sqrt(du * du + dv * dv + dw * dw);

        if (out.tmax <= 0.0) {
            out.vel = canon.linearFeedRate;
        } else {
            out.vel = out.dtot / out.tmax;
        }
    }
    if(debug_velacc) 
        printf("cartesian %d ang %d vel %g\n", canon.cartesian_move, canon.angular_move, out.vel);
    return out;
}

static VelData getStraightVelocity(const CANON_POSITION& pos)
{

    return getStraightVelocity(pos.x,
            pos.y,
            pos.z,
            pos.a,
            pos.b,
            pos.c,
            pos.u,
            pos.v,
            pos.w);
}

#include <vector>
struct pt {
    double x, y, z, a, b, c, u, v, w;
    int line_no;
    StateTag tag;
};

static std::vector<struct pt> chained_points;

static void drop_segments(void) {
    chained_points.clear();
}

static void flush_segments(void) {
    if(chained_points.empty()) return;

    struct pt &pos = chained_points.back();

    double x = pos.x, y = pos.y, z = pos.z;
    double a = pos.a, b = pos.b, c = pos.c;
    double u = pos.u, v = pos.v, w = pos.w;
    
    int line_no = pos.line_no;

#ifdef SHOW_JOINED_SEGMENTS
    for(unsigned int i=0; i != chained_points.size(); i++) { printf("."); }
    printf("\n");
#endif

    VelData linedata = getStraightVelocity(x, y, z, a, b, c, u, v, w);
    double vel = linedata.vel;

    if (canon.cartesian_move && !canon.angular_move) {
        if (vel > canon.linearFeedRate) {
            vel = canon.linearFeedRate;
        }
    } else if (!canon.cartesian_move && canon.angular_move) {
        if (vel > canon.angularFeedRate) {
            vel = canon.angularFeedRate;
        }
    } else if (canon.cartesian_move && canon.angular_move) {
        if (vel > canon.linearFeedRate) {
            vel = canon.linearFeedRate;
        }
    }


    auto linearMoveMsg = std::make_unique<EMC_TRAJ_LINEAR_MOVE>();
    linearMoveMsg->feed_mode = canon.feed_mode;

    // now x, y, z, and b are in absolute mm or degree units
    linearMoveMsg->end.tran.x = TO_EXT_LEN(x);
    linearMoveMsg->end.tran.y = TO_EXT_LEN(y);
    linearMoveMsg->end.tran.z = TO_EXT_LEN(z);

    linearMoveMsg->end.u = TO_EXT_LEN(u);
    linearMoveMsg->end.v = TO_EXT_LEN(v);
    linearMoveMsg->end.w = TO_EXT_LEN(w);

    // fill in the orientation
    linearMoveMsg->end.a = TO_EXT_ANG(a);
    linearMoveMsg->end.b = TO_EXT_ANG(b);
    linearMoveMsg->end.c = TO_EXT_ANG(c);

    linearMoveMsg->vel = toExtVel(vel);
    linearMoveMsg->ini_maxvel = toExtVel(linedata.vel);
    AccelData lineaccdata = getStraightAcceleration(x, y, z, a, b, c, u, v, w);
    double acc = lineaccdata.acc;
    linearMoveMsg->acc = toExtAcc(acc);

    linearMoveMsg->type = EMC_MOTION_TYPE_FEED;
    linearMoveMsg->indexer_jnum = -1;
    if ((vel && acc) || canon.spindle[canon.spindle_num].synched) {
        interp_list.set_line_number(line_no);
        tag_and_send(std::move(linearMoveMsg), pos.tag);
    }
    canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);

    drop_segments();
}

static void get_last_pos(double &lx, double &ly, double &lz) {
    if(chained_points.empty()) {
        lx = canon.endPoint.x;
        ly = canon.endPoint.y;
        lz = canon.endPoint.z;
    } else {
        struct pt &pos = chained_points.back();
        lx = pos.x;
        ly = pos.y;
        lz = pos.z;
    }
}

static bool
linkable(double x, double y, double z, 
         double a, double b, double c, 
         double u, double v, double w) {
    struct pt &pos = chained_points.back();
    if(canon.motionMode != CANON_CONTINUOUS || canon.naivecamTolerance == 0)
        return false;
    //FIXME make this length controlled elsewhere?
    if(chained_points.size() > 100) return false;

    //If ABCUVW motion, then the tangent calculation fails?
    // TODO is there a fundamental reason that we can't handle 9D motion here?
    if(a != pos.a) return false;
    if(b != pos.b) return false;
    if(c != pos.c) return false;
    if(u != pos.u) return false;
    if(v != pos.v) return false;
    if(w != pos.w) return false;

    if(x==canon.endPoint.x && y==canon.endPoint.y && z==canon.endPoint.z) return false;
    
    for(std::vector<struct pt>::iterator it = chained_points.begin();
            it != chained_points.end(); ++it) {
        PM_CARTESIAN M(x-canon.endPoint.x, y-canon.endPoint.y, z-canon.endPoint.z),
                     B(canon.endPoint.x, canon.endPoint.y, canon.endPoint.z),
                     P(it->x, it->y, it->z);
        double t0 = dot(M, P-B) / dot(M, M);
        if(t0 < 0) t0 = 0;
        if(t0 > 1) t0 = 1;

        double D = mag(P - (B + t0 * M));
        if(D > canon.naivecamTolerance) return false;
    }
    return true;
}

static void
see_segment(int line_number,
	    const StateTag& tag,
	    double x, double y, double z, 
            double a, double b, double c,
            double u, double v, double w) {
    bool changed_abc = (a != canon.endPoint.a)
        || (b != canon.endPoint.b)
        || (c != canon.endPoint.c);

    bool changed_uvw = (u != canon.endPoint.u)
        || (v != canon.endPoint.v)
        || (w != canon.endPoint.w);

    if(!chained_points.empty() && !linkable(x, y, z, a, b, c, u, v, w)) {
        flush_segments();
    }
    pt pos = {x, y, z, a, b, c, u, v, w, line_number, tag};
    chained_points.push_back(pos);
    if(changed_abc || changed_uvw) {
        flush_segments();
    }
}

void FINISH() {
    flush_segments();
}

void ON_RESET() {
    drop_segments();
}


void STRAIGHT_TRAVERSE(int line_number,
                       double x, double y, double z,
		               double a, double b, double c,
                       double u, double v, double w)
{
    double vel, acc;

    flush_segments();

    auto linearMoveMsg = std::make_unique<EMC_TRAJ_LINEAR_MOVE>();

    linearMoveMsg->feed_mode = 0;
    if (canon.rotary_unlock_for_traverse != -1)
        linearMoveMsg->type = EMC_MOTION_TYPE_INDEXROTARY;
    else
        linearMoveMsg->type = EMC_MOTION_TYPE_TRAVERSE;

    from_prog(x,y,z,a,b,c,u,v,w);
    rotate_and_offset_pos(x,y,z,a,b,c,u,v,w);

    VelData veldata = getStraightVelocity(x, y, z, a, b, c, u, v, w);
    AccelData accdata = getStraightAcceleration(x, y, z, a, b, c, u, v, w);

    vel = veldata.vel;
    acc = accdata.acc;

    linearMoveMsg->end = to_ext_pose(x,y,z,a,b,c,u,v,w);
    linearMoveMsg->vel = linearMoveMsg->ini_maxvel = toExtVel(vel);
    linearMoveMsg->acc = toExtAcc(acc);
    linearMoveMsg->indexer_jnum = canon.rotary_unlock_for_traverse;

    int old_feed_mode = canon.feed_mode;
    if(canon.feed_mode)
	STOP_SPEED_FEED_SYNCH();

    if(vel && acc)  {
        interp_list.set_line_number(line_number);
        tag_and_send(std::move(linearMoveMsg), _tag);
    }

    if(old_feed_mode)
	START_SPEED_FEED_SYNCH(canon.spindle_num, canon.linearFeedRate, 1);

    canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
}

void STRAIGHT_FEED(int line_number, double x, double y, double z, double a, double b, double c, double u, double v, double w)
{
    //EMC_TRAJ_LINEAR_MOVE linearMoveMsg;
    //linearMoveMsg.feed_mode = canon.feed_mode;

    from_prog(x,y,z,a,b,c,u,v,w);
    rotate_and_offset_pos(x,y,z,a,b,c,u,v,w);
    see_segment(line_number, _tag, x, y, z, a, b, c, u, v, w);
}


void RIGID_TAP(int line_number, double x, double y, double z, double scale)
{
    double ini_maxvel,acc;
    auto rigidTapMsg = std::make_unique<EMC_TRAJ_RIGID_TAP>();
    double unused=0;

    from_prog(x,y,z,unused,unused,unused,unused,unused,unused);
    rotate_and_offset_pos(x,y,z,unused,unused,unused,unused,unused,unused);

    VelData veldata = getStraightVelocity(x, y, z, 
                              canon.endPoint.a, canon.endPoint.b, canon.endPoint.c, 
                              canon.endPoint.u, canon.endPoint.v, canon.endPoint.w);
    ini_maxvel = veldata.vel;
    
    AccelData accdata = getStraightAcceleration(x, y, z, 
                                  canon.endPoint.a, canon.endPoint.b, canon.endPoint.c,
                                  canon.endPoint.u, canon.endPoint.v, canon.endPoint.w);
    acc = accdata.acc;
    
    rigidTapMsg->pos = to_ext_pose(x,y,z,
                                 canon.endPoint.a, canon.endPoint.b, canon.endPoint.c,
                                 canon.endPoint.u, canon.endPoint.v, canon.endPoint.w);

    rigidTapMsg->vel = toExtVel(ini_maxvel);
    rigidTapMsg->ini_maxvel = toExtVel(ini_maxvel);
    rigidTapMsg->acc = toExtAcc(acc);
    rigidTapMsg->scale = scale;
    flush_segments();

    if(ini_maxvel && acc)  {
        interp_list.set_line_number(line_number);
        interp_list.append(std::move(rigidTapMsg));
    }

    // don't move the endpoint because after this move, we are back where we started
}


/*
  STRAIGHT_PROBE is exactly the same as STRAIGHT_FEED, except that it
  uses a probe message instead of a linear move message.
*/

void STRAIGHT_PROBE(int line_number,
                    double x, double y, double z, 
                    double a, double b, double c,
                    double u, double v, double w,
                    unsigned char probe_type)
{
    double ini_maxvel, vel, acc;
    auto probeMsg = std::make_unique<EMC_TRAJ_PROBE>();

    from_prog(x,y,z,a,b,c,u,v,w);
    rotate_and_offset_pos(x,y,z,a,b,c,u,v,w);

    flush_segments();

    VelData veldata = getStraightVelocity(x, y, z, a, b, c, u, v, w);
    ini_maxvel = vel = veldata.vel;

    if (canon.cartesian_move && !canon.angular_move) {
	if (vel > canon.linearFeedRate) {
	    vel = canon.linearFeedRate;
	}
    } else if (!canon.cartesian_move && canon.angular_move) {
	if (vel > canon.angularFeedRate) {
	    vel = canon.angularFeedRate;
	}
    } else if (canon.cartesian_move && canon.angular_move) {
	if (vel > canon.linearFeedRate) {
	    vel = canon.linearFeedRate;
	}
    }

    AccelData accdata = getStraightAcceleration(x, y, z, a, b, c, u, v, w);
    acc = accdata.acc;

    probeMsg->vel = toExtVel(vel);
    probeMsg->ini_maxvel = toExtVel(ini_maxvel);
    probeMsg->acc = toExtAcc(acc);

    probeMsg->type = EMC_MOTION_TYPE_PROBING;
    probeMsg->probe_type = probe_type;

    probeMsg->pos = to_ext_pose(x,y,z,a,b,c,u,v,w);

    if(vel && acc)  {
        interp_list.set_line_number(line_number);
        interp_list.append(std::move(probeMsg));
    }
    canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
}

/* Machining Attributes */

void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode, double tolerance)
{
    auto setTermCondMsg = std::make_unique<EMC_TRAJ_SET_TERM_COND>();

    flush_segments();

    canon.motionMode = mode;
    canon.motionTolerance =  FROM_PROG_LEN(tolerance);

    switch (mode) {
    case CANON_CONTINUOUS:
        setTermCondMsg->cond = EMC_TRAJ_TERM_COND_BLEND;
        setTermCondMsg->tolerance = TO_EXT_LEN(canon.motionTolerance);
        break;
    case CANON_EXACT_PATH:
        setTermCondMsg->cond = EMC_TRAJ_TERM_COND_EXACT;
        break;

    case CANON_EXACT_STOP:
    default:
        setTermCondMsg->cond = EMC_TRAJ_TERM_COND_STOP;
        break;
    }

    interp_list.append(std::move(setTermCondMsg));
}

void SET_NAIVECAM_TOLERANCE(double tolerance)
{
    canon.naivecamTolerance =  FROM_PROG_LEN(tolerance);
}

void SELECT_PLANE(CANON_PLANE in_plane)
{
    canon.activePlane = in_plane;
}

void SET_CUTTER_RADIUS_COMPENSATION(double /*radius*/)
{
    // nothing need be done here
}

void START_CUTTER_RADIUS_COMPENSATION(int /*side*/)
{
    // nothing need be done here
}

void STOP_CUTTER_RADIUS_COMPENSATION()
{
    // nothing need be done here
}



void START_SPEED_FEED_SYNCH(int spindle, double feed_per_revolution, bool velocity_mode)
{
    flush_segments();
    auto spindleSyncMsg = std::make_unique<EMC_TRAJ_SET_SPINDLESYNC>();
    spindleSyncMsg->spindle = spindle;
    spindleSyncMsg->feed_per_revolution = TO_EXT_LEN(FROM_PROG_LEN(feed_per_revolution));
    spindleSyncMsg->velocity_mode = velocity_mode;
    interp_list.append(std::move(spindleSyncMsg));
    canon.spindle[spindle].synched = 1;
}

void STOP_SPEED_FEED_SYNCH()
{
    START_SPEED_FEED_SYNCH(0, 0, false);
    canon.spindle[canon.spindle_num].synched = 0;
}

/* Machining Functions */
static double chord_deviation(double sx, double sy, double ex, double ey, double cx, double cy, int rotation, double &mx, double &my) {
    double th1 = atan2(sy-cy, sx-cx),
           th2 = atan2(ey-cy, ex-cx),
           r = hypot(sy-cy, sx-cx),
           dth = th2 - th1;

    if(rotation < 0) {
        if(dth >= -1e-5) th2 -= 2*M_PI;
        // in the edge case where atan2 gives you -pi and pi, a second iteration is needed
        // to get these in the right order
        dth = th2 - th1;
        if(dth >= -1e-5) th2 -= 2*M_PI;
    } else {
        if(dth <= 1e-5) th2 += 2*M_PI;
        dth = th2 - th1;
        if(dth <= 1e-5) th2 += 2*M_PI;
    }

    double included = fabs(th2 - th1);
    double mid = (th2 + th1) / 2;
    mx = cx + r * cos(mid);
    my = cy + r * sin(mid);
    double dev = r * (1 - cos(included/2));
    return dev;
}

/* Spline and NURBS additional functions; */

static void unit(double *x, double *y) {
    double h = hypot(*x, *y);
    if(h != 0) { *x/=h; *y/=h; }
}


static void
arc(int lineno, double x0, double y0, double x1, double y1, double dx,
    double dy)
{

	double small = 0.000001;
	double x = x1 - x0, y = y1 - y0;
	double den = 2 * (y * dx - x * dy);
	CANON_POSITION p = unoffset_and_unrotate_pos(canon.endPoint);
	to_prog(p);
	if (fabs(den) > small) {
		double r = -(x * x + y * y) / den;
		double i = dy * r, j = -dx * r;
		double cx = x0 + i, cy = y0 + j;

		if (canon.activePlane == CANON_PLANE::XY) {
			ARC_FEED(lineno, x1, y1, cx, cy, r < 0 ? 1 : -1,
				 p.z, p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			ARC_FEED(lineno, x1, y1, cx, cy, r < 0 ? 1 : -1,
				 p.x, p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			ARC_FEED(lineno, x1, y1, cx, cy, r < 0 ? 1 : -1,
				 p.y, p.a, p.b, p.c, p.u, p.v, p.w);
		}
	} else {
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, x1, y1, p.z, p.a, p.b, p.c,
				      p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, x1, y1, p.a, p.b, p.c,
				      p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, y1, p.y, x1, p.a, p.b, p.c,
				      p.u, p.v, p.w);
		}
	}
}

static int
biarc(int lineno, double p0x, double p0y, double tsx, double tsy,
      double p4x, double p4y, double tex, double tey, double r = 1.0)
{
	unit(&tsx, &tsy);
	unit(&tex, &tey);

	double vx = p0x - p4x;	// Distanz x
	double vy = p0y - p4y;	// Distanz y
	double c = vx * vx + vy * vy;	// Distnaz r^2
	double b = 2 * (vx * (r * tsx + tex) + vy * (r * tsy + tey));
	double a = 2 * r * (tsx * tex + tsy * tey - 1);

	double discr = b * b - 4 * a * c;
	if (discr < 0) {
		return 0;
	}

	double disq = sqrt(discr);
	double beta1 = (-b - disq) / 2 / a;
	double beta2 = (-b + disq) / 2 / a;
	if (beta1 > 0 && beta2 > 0) {
		return 0;
	}

	double beta = std::max(beta1, beta2);
	double aa = 1, bb = 0, cc;
	cc = aa / bb;		//cc=inf
	//if(beta1 == cc || beta2 == cc) { // original Lo Valvo
	if (beta1 == cc || beta2 == cc || beta > (1 / CART_FUZZ)) {	// "beta > 1e8" added by jjf (without this condition "Ruota_dentata #101.ngc" is not working! beta=1e14)
		CANON_POSITION p =
		    unoffset_and_unrotate_pos(canon.endPoint);
		to_prog(p);
		STRAIGHT_FEED(lineno, p4x, p4y, p.z, p.a, p.b, p.c, p.u,
			      p.v, p.w);
	} else {
		double alpha = beta * r;
		double ab = alpha + beta;
		double p1x = p0x + alpha * tsx, p1y = p0y + alpha * tsy,
		    p3x = p4x - beta * tex, p3y = p4y - beta * tey,
		    p2x = (p1x * beta + p3x * alpha) / ab,
		    p2y = (p1y * beta + p3y * alpha) / ab;
		double tmx = p3x - p2x, tmy = p3y - p2y;
		unit(&tmx, &tmy);
		arc(lineno, p0x, p0y, p2x, p2y, tsx, tsy);
		arc(lineno, p2x, p2y, p4x, p4y, tmx, tmy);
	}
	return 1;
}

/* Canon calls */

//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_G5_FEED(int lineno,
		   const std::vector<NURBS_CONTROL_POINT>& nurbs_control_points,
		   unsigned int nurbs_order,
		   CANON_PLANE /*plane*/)
{
	flush_segments();

	unsigned int n = nurbs_control_points.size() - 1;
	double umax = n - nurbs_order + 2;
	unsigned int div = nurbs_control_points.size() * 4;

	std::vector<unsigned int> knot_vector =
	    nurbs_G5_knot_vector_creator(n, nurbs_order);
	NURBS_PLANE_POINT P0, P0T, P1, P1T;

	P0 = nurbs_G5_point(0, nurbs_order, nurbs_control_points,
			    knot_vector);
	P0T =
	    nurbs_G5_tangent(0, nurbs_order, nurbs_control_points,
			     knot_vector);

	for (unsigned int i = 1; i <= div; i++) {
		double u = umax * i / div;
		P1 = nurbs_G5_point(u, nurbs_order, nurbs_control_points,
				    knot_vector);
		P1T =
		    nurbs_G5_tangent(u, nurbs_order, nurbs_control_points,
				     knot_vector);
		biarc(lineno, P0.NURBS_X, P0.NURBS_Y, P0T.NURBS_X, P0T.NURBS_Y, P1.NURBS_X, P1.NURBS_Y, P1T.NURBS_X, P1T.NURBS_Y);	// G5
		P0 = P1;
		P0T = P1T;
	}
	knot_vector.clear();
}

// G6.2 Q_option=3=NICL Interpolazione NURBS con movimento lineare
// G6.2 Q_option=3=NICL NURBS interpolation with linear motion
//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_FEED_G6_2_WITH_LINEAR_MOTION(int lineno,
					const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points,
					const std::vector<double>& knot_vector,
					unsigned int k, double feedrate)
{
	flush_segments();
	unsigned int n = nurbs_control_points.size() - 1 - k;
	std::vector<double> span_knot_vector =
	    nurbs_interval_span_knot_vector_creator(n, k, knot_vector);
	std::vector<double> lenght_vector =
	    nurbs_lenght_vector_creator(k, nurbs_control_points,
					knot_vector, span_knot_vector);
	std::vector<double> Du_span_knot_vector =
	    nurbs_Du_span_knot_vector_creator(k, nurbs_control_points,
					      knot_vector,
					      span_knot_vector);
	std::vector<double> nurbs_costant =
	    nurbs_costant_crator(span_knot_vector, lenght_vector,
				 Du_span_knot_vector);
	NURBS_PLANE_POINT P1, P1x;
	double dl, T, ltot, u_l, alf;
	int alf1;

	T = 0.1;		// Tempo di CAMPIONAMENTO (multiplo)
	alf = (0.1 * 60 / (T * feedrate));
	alf1 = (int) alf;
	if (alf1 == 0) {
		alf1 = 1;
	}
	if (feedrate >= 1500) {
		feedrate = feedrate / 10;
	}
	dl = alf1 * (feedrate * T / 60);
	ltot =
	    nurbs_lenght_tot((lenght_vector.size() - 1), span_knot_vector,
			     lenght_vector);

	CANON_POSITION p = unoffset_and_unrotate_pos(canon.endPoint);	// hier steht z falsch drin
	to_prog(p);
	std::vector < std::vector < double >>A6;	// array
	A6 = nurbs_G6_Nmix_creator(knot_vector[0], k, n + 1, knot_vector);
	P1 = nurbs_G6_pointx(knot_vector[0], k, nurbs_control_points,
			     knot_vector, A6);
	if (canon.activePlane == CANON_PLANE::XY) {
		STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}
	if (canon.activePlane == CANON_PLANE::YZ) {
		STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}
	if (canon.activePlane == CANON_PLANE::XZ) {
		STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}

	double a = 0, du = 0;
	if (k >= 3) {
		for (double l_ = dl; l_ <= ltot - dl; l_ = l_ + dl) {
			u_l =
			    nurbs_uj_l(l_, span_knot_vector, lenght_vector,
				       nurbs_costant);
			du = abs(u_l - a);

			if (u_l >= a) {	// fa un controllo per rimediare a eventuali  errori groddolani di approssimazione
				P1x =
				    nurbs_G6_point_x(u_l, k,
						     nurbs_control_points,
						     knot_vector);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1x.NURBS_X,
						      P1x.NURBS_Y, p.z,
						      p.a, p.b, p.c, p.u,
						      p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1x.NURBS_X,
						      P1x.NURBS_Y, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1x.NURBS_Y,
						      p.y, P1x.NURBS_X,
						      p.a, p.b, p.c, p.u,
						      p.v, p.w);
				}
				a = u_l;
			} else {
				u_l = a + du / 4;
				P1x =
				    nurbs_G6_point_x(u_l, k,
						     nurbs_control_points,
						     knot_vector);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1x.NURBS_X,
						      P1x.NURBS_Y, p.z,
						      p.a, p.b, p.c, p.u,
						      p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1x.NURBS_X,
						      P1x.NURBS_Y, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1x.NURBS_Y,
						      p.y, P1x.NURBS_X,
						      p.a, p.b, p.c, p.u,
						      p.v, p.w);
				}
				a = u_l;
			}
		}
		A6 = nurbs_G6_Nmix_creator(knot_vector[n + k], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[n + k], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		span_knot_vector.clear();
		lenght_vector.clear();
		Du_span_knot_vector.clear();
		A6.clear();
	} else {
		for (double l_ = dl; l_ <= ltot - dl; l_ = l_ + dl) {
			u_l =
			    nurbs_uj_l(l_, span_knot_vector, lenght_vector,
				       nurbs_costant);
			du = abs(u_l - a);

			if (u_l >= a) {	// fa un controllo per rimediare a eventuali  errori grossolani di approssimazione
				A6 = nurbs_G6_Nmix_creator(u_l, k, n + 1,
							   knot_vector);
				P1 = nurbs_G6_pointx(u_l, k,
						     nurbs_control_points,
						     knot_vector, A6);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1.NURBS_X,
						      P1.NURBS_Y, p.z, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1.NURBS_X,
						      P1.NURBS_Y, p.a, p.b,
						      p.c, p.u, p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1.NURBS_Y,
						      p.y, P1.NURBS_X, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				a = u_l;
			} else {
				u_l = a + du / 4;	// il precedente valore di u è incrementato di 1/4;
				A6 = nurbs_G6_Nmix_creator(u_l, k, n + 1,
							   knot_vector);
				P1 = nurbs_G6_pointx(u_l, k,
						     nurbs_control_points,
						     knot_vector, A6);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1.NURBS_X,
						      P1.NURBS_Y, p.z, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1.NURBS_X,
						      P1.NURBS_Y, p.a, p.b,
						      p.c, p.u, p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1.NURBS_Y,
						      p.y, P1.NURBS_X, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
			}
		}
		A6 = nurbs_G6_Nmix_creator(knot_vector[n + k], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[n + k], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		span_knot_vector.clear();
		lenght_vector.clear();
		Du_span_knot_vector.clear();
		A6.clear();
	}
}

// G6.2 Q_option=2=NICC Interpolazione NURBS con movimento circolare
// G6.2 Q_option=2=NICC NURBS interpolation with circular motion
//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_FEED_G6_2_WITH_CIRCULAR_MOTION(int lineno,
					  const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points,
					  const std::vector<double>& knot_vector,
					  unsigned int k, double feedrate)
{
	flush_segments();
	unsigned int n = nurbs_control_points.size() - 1 - k;
	std::vector < double >span_knot_vector =
	    nurbs_interval_span_knot_vector_creator(n, k, knot_vector);
	std::vector<double> lenght_vector =
	    nurbs_lenght_vector_creator(k, nurbs_control_points,
					knot_vector, span_knot_vector);
	std::vector<double> Du_span_knot_vector =
	    nurbs_Du_span_knot_vector_creator(k, nurbs_control_points,
					      knot_vector,
					      span_knot_vector);
	std::vector<double> nurbs_costant =
	    nurbs_costant_crator(span_knot_vector, lenght_vector,
				 Du_span_knot_vector);
	NURBS_PLANE_POINT P0, P1, P0T, P1T;
	double dl, T, ltot, u_l, alf;
	int alf1;
	T = 0.1;		// Tempo T di campionamento (multiplo)
	alf = (0.1 * 60 / (T * feedrate));
	alf1 = (int) alf;
	if (alf1 == 0) {
		alf1 = 1;
	}
	if (feedrate >= 1500) {
		feedrate = feedrate / 10;
	}
	dl = alf1 * (feedrate * T / 60);
	ltot =
	    nurbs_lenght_tot((lenght_vector.size() - 1), span_knot_vector,
			     lenght_vector);
	CANON_POSITION p = unoffset_and_unrotate_pos(canon.endPoint);
	to_prog(p);
	std::vector < std::vector < double >>A6;	//A6 è la matrice dove sono memorizzati i valori Ni,p(u) per dato valore di u

	A6 = nurbs_G6_Nmix_creator(knot_vector[0], k, n + 1, knot_vector);
	P1 = nurbs_G6_pointx(knot_vector[0], k, nurbs_control_points,
			     knot_vector, A6);
	if (canon.activePlane == CANON_PLANE::XY) {
		STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}
	if (canon.activePlane == CANON_PLANE::YZ) {
		STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}
	if (canon.activePlane == CANON_PLANE::XZ) {
		STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X, p.a,
			      p.b, p.c, p.u, p.v, p.w);
	}
	double a = 0, du = 0;
	if (k >= 3) {		//l'algoritmo di De Boor non funziona per ordine <3 quindi uso la formulazione ricorsiva, ossia A6
		P0 = nurbs_G6_point_x(knot_vector[0], k,
				      nurbs_control_points, knot_vector);
		P0T =
		    nurbs_G6_tangent_x(knot_vector[0], k,
				       nurbs_control_points, knot_vector);
		for (double l_ = dl; l_ <= ltot - dl; l_ = l_ + dl) {
			u_l =
			    nurbs_uj_l(l_, span_knot_vector, lenght_vector,
				       nurbs_costant);
			du = abs(u_l - a);

			if (u_l >= a) {	// fa un controllo per rimediare a eventuali  errori grossolani di approssimazione
				P1 = nurbs_G6_point_x(u_l, k,
						      nurbs_control_points,
						      knot_vector);
				P1T =
				    nurbs_G6_tangent_x(u_l, k,
						       nurbs_control_points,
						       knot_vector);
				biarc(lineno, P0.NURBS_X, P0.NURBS_Y,
				      P0T.NURBS_X, P0T.NURBS_Y, P1.NURBS_X,
				      P1.NURBS_Y, P1T.NURBS_X,
				      P1T.NURBS_Y);
				P0 = P1;
				P0T = P1T;
				a = u_l;
			} else {
				u_l = a + du / 4;	// il precedente valore di u è incrementato della metta del precedente du;

				P1 = nurbs_G6_point_x(u_l, k,
						      nurbs_control_points,
						      knot_vector);
				P1T =
				    nurbs_G6_tangent_x(u_l, k,
						       nurbs_control_points,
						       knot_vector);
				biarc(lineno, P0.NURBS_X, P0.NURBS_Y,
				      P0T.NURBS_X, P0T.NURBS_Y, P1.NURBS_X,
				      P1.NURBS_Y, P1T.NURBS_X,
				      P1T.NURBS_Y);
				P0 = P1;
				P0T = P1T;
				a = u_l;
			}
		}
		A6 = nurbs_G6_Nmix_creator(knot_vector[n + k], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[n + k], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		span_knot_vector.clear();
		lenght_vector.clear();
		Du_span_knot_vector.clear();
		A6.clear();
	} else {
		for (double l_ = dl; l_ <= ltot - dl; l_ = l_ + dl) {
			u_l =
			    nurbs_uj_l(l_, span_knot_vector, lenght_vector,
				       nurbs_costant);
			du = abs(u_l - a);
			if (u_l >= a) {
				A6 = nurbs_G6_Nmix_creator(u_l, k, n + 1,
							   knot_vector);
				P1 = nurbs_G6_pointx(u_l, k,
						     nurbs_control_points,
						     knot_vector, A6);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1.NURBS_X,
						      P1.NURBS_Y, p.z, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1.NURBS_X,
						      P1.NURBS_Y, p.a, p.b,
						      p.c, p.u, p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1.NURBS_Y,
						      p.y, P1.NURBS_X, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				a = u_l;
			} else {	// fa un controllo per rimediare a eventuali  errori grossolani di approssimazione
				u_l = a + du / 4;
				A6 = nurbs_G6_Nmix_creator(u_l, k, n + 1,
							   knot_vector);
				P1 = nurbs_G6_pointx(u_l, k,
						     nurbs_control_points,
						     knot_vector, A6);
				if (canon.activePlane == CANON_PLANE::XY) {
					STRAIGHT_FEED(lineno, P1.NURBS_X,
						      P1.NURBS_Y, p.z, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				if (canon.activePlane == CANON_PLANE::YZ) {
					STRAIGHT_FEED(lineno, p.x,
						      P1.NURBS_X,
						      P1.NURBS_Y, p.a, p.b,
						      p.c, p.u, p.v, p.w);
				}
				if (canon.activePlane == CANON_PLANE::XZ) {
					STRAIGHT_FEED(lineno, P1.NURBS_Y,
						      p.y, P1.NURBS_X, p.a,
						      p.b, p.c, p.u, p.v,
						      p.w);
				}
				a = u_l;
			}
		}
		A6 = nurbs_G6_Nmix_creator(knot_vector[n + k], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[n + k], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		span_knot_vector.clear();
		lenght_vector.clear();
		Du_span_knot_vector.clear();
		A6.clear();
	}
}

// G6.2/G6.3 Q_option=1=NICU interpolazione NURBS con biarchi, du=cost
// G6.2/G6.3 Q_option=1=NICU NURBS interpolation with biarc, du=const
//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_FEED_G6_2_WITH_BIARCH_DU_CONST(int lineno,
					  const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points,
					  const std::vector<double>& knot_vector,
					  unsigned int k, double /*feedrate*/)
{
	double u = knot_vector[0];
	int dim = nurbs_control_points.size();
	unsigned int n = dim - k - 1;
	double umax = knot_vector[knot_vector.size() - 1];
	unsigned int div = (dim - k) * 15;

	std::vector<std::vector<double>> A6;
	if (k >= 3) {
		NURBS_PLANE_POINT P1, P0, P0T, P1T;
		CANON_POSITION p =
		    unoffset_and_unrotate_pos(canon.endPoint);
		to_prog(p);

		A6 = nurbs_G6_Nmix_creator(knot_vector[0], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[0], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		P0 = nurbs_G6_point_x(knot_vector[0], k,
				      nurbs_control_points, knot_vector);
		P0T =
		    nurbs_G6_tangent_x(knot_vector[0], k,
				       nurbs_control_points, knot_vector);
		double du = umax / div;
		for (u = du; u <= (umax - du); u = u + du) {
			P1 = nurbs_G6_point_x(u, k, nurbs_control_points,
					      knot_vector);
			P1T =
			    nurbs_G6_tangent_x(u, k, nurbs_control_points,
					       knot_vector);
			biarc(lineno, P0.NURBS_X, P0.NURBS_Y, P0T.NURBS_X,
			      P0T.NURBS_Y, P1.NURBS_X, P1.NURBS_Y,
			      P1T.NURBS_X, P1T.NURBS_Y);
			P0 = P1;
			P0T = P1T;
		}

		A6 = nurbs_G6_Nmix_creator(umax, k, n + 1, knot_vector);
		P1 = nurbs_G6_pointx(umax, k, nurbs_control_points,
				     knot_vector, A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		A6.clear();
	} else {
		NURBS_PLANE_POINT P1;

		CANON_POSITION p =
		    unoffset_and_unrotate_pos(canon.endPoint);
		to_prog(p);
		A6 = nurbs_G6_Nmix_creator(knot_vector[0], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[0], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		double du = umax / div;
		for (u = du; u <= (umax - du); u = u + du) {
			P1 = nurbs_G6_point_x(u, k, nurbs_control_points,
					      knot_vector);
			if (canon.activePlane == CANON_PLANE::XY) {
				STRAIGHT_FEED(lineno, P1.NURBS_X,
					      P1.NURBS_Y, p.z, p.a, p.b,
					      p.c, p.u, p.v, p.w);
			}
			if (canon.activePlane == CANON_PLANE::YZ) {
				STRAIGHT_FEED(lineno, p.x, P1.NURBS_X,
					      P1.NURBS_Y, p.a, p.b, p.c,
					      p.u, p.v, p.w);
			}
			if (canon.activePlane == CANON_PLANE::XZ) {
				STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y,
					      P1.NURBS_X, p.a, p.b, p.c,
					      p.u, p.v, p.w);
			}
		}
		A6 = nurbs_G6_Nmix_creator(knot_vector[n + k], k, n + 1,
					   knot_vector);
		P1 = nurbs_G6_pointx(knot_vector[n + k], k,
				     nurbs_control_points, knot_vector,
				     A6);
		if (canon.activePlane == CANON_PLANE::XY) {
			STRAIGHT_FEED(lineno, P1.NURBS_X, P1.NURBS_Y, p.z,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::YZ) {
			STRAIGHT_FEED(lineno, p.x, P1.NURBS_X, P1.NURBS_Y,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		if (canon.activePlane == CANON_PLANE::XZ) {
			STRAIGHT_FEED(lineno, P1.NURBS_Y, p.y, P1.NURBS_X,
				      p.a, p.b, p.c, p.u, p.v, p.w);
		}
		A6.clear();
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_FEED_DIVIDE(int lineno,
		       std::vector<NURBS_G6_CONTROL_POINT>
		       nurbs_control_points,
		       std::vector<double> knot_vector,
		       unsigned int nurbs_order, double feedrate,
		       int Q_option)
{
	std::vector<double> knot_vector_new = knot_vector;
	int k1 = knot_vector.size() / 2 - 1;
	double ut;
	ut = (knot_vector[k1] + 1);
	//printf("ut: %f (F: %s L: %d)\n", ut, __FILE__, __LINE__);

	//SEGMENTO NURBS 1
	std::vector < NURBS_CONTROL_POINT > nurbs_control_points1;
	// si passa il valore u in corrispondenza del quale suddividere la curva
	// der u-Wert wird übergeben, bei dem die Kurve geteilt werden soll
	std::vector < double >control_point_nurbs1_new1 =
	    nurbs_G6_new_control_point_nurbs1(ut, nurbs_order,
					      nurbs_control_points,
					      knot_vector);

	NURBS_CONTROL_POINT C_P1;
	for (long unsigned int i = 0; i < control_point_nurbs1_new1.size();
	     i = i + 3) {
		C_P1.NURBS_X = control_point_nurbs1_new1[i + 0];
		C_P1.NURBS_Y = control_point_nurbs1_new1[i + 1];
		C_P1.NURBS_W = control_point_nurbs1_new1[i + 2];
		nurbs_control_points1.push_back(C_P1);	//1
	}

	//SEGMENTO NURBS 2
	//printf("*- SEGMENTO NURBS 2 ---------------------------------------- (%s %d)\n", __FILE__,__LINE__);
	//Secondo segmento NURBS
	// si passa il valore u in corrispondenza del quale suddividere la curva
	// der u-Wert wird übergeben, bei dem die Kurve geteilt werden soll
	std::vector<double> control_point_nurbs2_new2 =
	    nurbs_G6_new_control_point_nurbs2(ut, nurbs_order,
					      nurbs_control_points,
					      knot_vector);

	std::vector<NURBS_CONTROL_POINT> nurbs_control_points2;
	NURBS_CONTROL_POINT C_P2;
	for (long unsigned int i = 0; i < control_point_nurbs2_new2.size();
	     i = i + 3) {
		C_P2.NURBS_X = control_point_nurbs2_new2[i];
		C_P2.NURBS_Y = control_point_nurbs2_new2[i + 1];
		C_P2.NURBS_W = control_point_nurbs2_new2[i + 2];
		nurbs_control_points2.push_back(C_P2);	//1
	}

	for (int j_segment = 0; j_segment <= 1; ++j_segment) {
		// ridefinisco Control_point per il primo SGMENTO (1) NURBS
		// ich definiere Control_point für die ersten SGMENTO (1) NURBS neu
		nurbs_control_points.clear();
		NURBS_G6_CONTROL_POINT CPU = {};
		if (j_segment == 0) {
			for (unsigned j = 0;
			     j < nurbs_control_points1.size(); ++j) {
				CPU.NURBS_X =
				    nurbs_control_points1[j].NURBS_X;
				CPU.NURBS_Y =
				    nurbs_control_points1[j].NURBS_Y;
				CPU.NURBS_R = nurbs_control_points1[j].NURBS_W;	// peso del punto di controll // Kontrollpunktgewicht
				nurbs_control_points.push_back(CPU);
			}
			for (unsigned j = 0; j < nurbs_order; ++j) {
				nurbs_control_points.push_back(CPU);
			}
			knot_vector =
			    nurbs_G6_knot_vector_new_creator_sgment
			    (nurbs_order, nurbs_control_points);
		} else {
			for (unsigned j = 0;
			     j < nurbs_control_points2.size(); ++j) {
				CPU.NURBS_X =
				    nurbs_control_points2[j].NURBS_X;
				CPU.NURBS_Y =
				    nurbs_control_points2[j].NURBS_Y;
				CPU.NURBS_R = nurbs_control_points2[j].NURBS_W;	// peso del punto di controll
				nurbs_control_points.push_back(CPU);
			}

			// la porzione di codice aggiunge zeri, l'obiettivo è di far aumentare di k la dimenzione, questo in modo da farne coincidere le dimenzione rispetto a quando la suddivisione non avviene realizzata.
			// Der Teil des Codes fügt Nullen hinzu, das Ziel ist es, die Dimension um k zu erhöhen, um die Dimensionen in Bezug darauf zusammenfallen zu lassen, wenn die Unterteilung nicht stattfindet.
			for (unsigned j = 0; j < nurbs_order; ++j) {
				nurbs_control_points.push_back(CPU);
			}
			knot_vector =
			    nurbs_G6_knot_vector_new_creator_sgment
			    (nurbs_order, nurbs_control_points);
		}
		if ((nurbs_control_points.size() - nurbs_order) >
		    3 * nurbs_order) {
			NURBS_FEED_DIVIDE(lineno, nurbs_control_points, knot_vector, nurbs_order, feedrate, Q_option);	// recursiv
		} else {
			if (Q_option == 3) {
				NURBS_FEED_G6_2_WITH_LINEAR_MOTION(lineno,
								   nurbs_control_points,
								   knot_vector,
								   nurbs_order,
								   feedrate);
			} else {
				if (Q_option == 2) {
					NURBS_FEED_G6_2_WITH_CIRCULAR_MOTION
					    (lineno, nurbs_control_points,
					     knot_vector, nurbs_order,
					     feedrate);
				} else {
					if (Q_option == 1) {
						flush_segments();
						NURBS_FEED_G6_2_WITH_BIARCH_DU_CONST
						    (lineno,
						     nurbs_control_points,
						     knot_vector,
						     nurbs_order,
						     feedrate);
					} else {	//metodo misto, if there is a wrong code on #1 
						if (feedrate <= 300) {	//velocità di avanzamento 300mm/min //METODO MISTO
							//printf("G6.2_L_mist=interpolazione NURBS tramite movimento lineare (è il controllo a decidere di applicare tale metodo in funzione del valore ds)\n");
							//printf("G6.2_L_mist=NURBS interpolation using linear movement (it is the control that decides to apply this method according to the ds value)\n");
							NURBS_FEED_G6_2_WITH_LINEAR_MOTION
							    (lineno,
							     nurbs_control_points,
							     knot_vector,
							     nurbs_order,
							     feedrate);
						} else {	//SECONDA OPZIONE METODO MISTO
							//printf("G6.2_B_mist=interpolazione NURBS tramite movimento circolare (è il controllo a decidere di applicare tale metodo in funzione del valore ds)\n");
							NURBS_FEED_G6_2_WITH_CIRCULAR_MOTION
							    (lineno,
							     nurbs_control_points,
							     knot_vector,
							     nurbs_order,
							     feedrate);
						}
					}
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------------------------------------------------
//Non viene realizzata alcuna suddivisione in quanto il numero dei punti di controllo è <2*k =due volte l'ordine
//No subdivision is made as the number of control points is <3*k = twice the order
void NURBS_FEED_NO_SUBDIVISION(int lineno,
			       const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points,
			       const std::vector<double>& knot_vector,
			       unsigned int nurbs_order, double feedrate,
			       int Q_option)
{
	if (Q_option == 3) {
		NURBS_FEED_G6_2_WITH_LINEAR_MOTION(lineno,
						   nurbs_control_points,
						   knot_vector,
						   nurbs_order, feedrate);
	} else {
		if (Q_option == 2) {
			NURBS_FEED_G6_2_WITH_CIRCULAR_MOTION(lineno,
							     nurbs_control_points,
							     knot_vector,
							     nurbs_order,
							     feedrate);
		} else {
			if (Q_option == 1) {
				flush_segments();
				NURBS_FEED_G6_2_WITH_BIARCH_DU_CONST
				    (lineno, nurbs_control_points,
				     knot_vector, nurbs_order, feedrate);
			} else {	//metodo misto
				// this does not occur, because L is 1 or 2 or 3
				if (feedrate <= 300) {	//velocità di avanzamento 300mm/min
					//printf("G6.2_L_mist=interpolazione NURBS tramite movimento lineare (è il controllo a decidere di applicare tale metodo in funzione del valore ds)\n");
					//printf("G6.2_L_mist=NURBS interpolation using linear movement (it is the control that decides to apply this method according to the ds value)\n");
					NURBS_FEED_G6_2_WITH_LINEAR_MOTION
					    (lineno, nurbs_control_points,
					     knot_vector, nurbs_order,
					     feedrate);
				} else {	//SECONDA OPZIONE METODO MISTO
					//printf("G6.2_B_mist=interpolazione NURBS tramite movimento circolare (è il controllo a decidere di applicare tale metodo in funzione del valore ds)\n");
					//printf("G6.2_B_mist=NURBS interpolation via circular movement (it is the control that decides to apply this method according to the ds value)\n");
					NURBS_FEED_G6_2_WITH_CIRCULAR_MOTION
					    (lineno, nurbs_control_points,
					     knot_vector, nurbs_order,
					     feedrate);
				}
			}
		}
	}
}

/* Canon calls G_6_2 */
//-----------------------------------------------------------------------------------------------------------------------------------------
void NURBS_G6_FEED(int lineno,
		   const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points,
		   unsigned int nurbs_order,
		   double feedrate, int Q_option, CANON_PLANE /*plane*/)
{				// (L_option: NICU, NICL, NICC see publication from Lo Valvo and Drago) 
	int n = nurbs_control_points.size() - 1 - nurbs_order;
	std::vector<double> knot_vector =
	    nurbs_g6_knot_vector_creator(n, nurbs_order,
					 nurbs_control_points);

	if (n > (int) (3 * nurbs_order)) {
		//Operazione di verifica che eventualmente attiva la suddivisione della NURBS in segmenti
		//Verification operation which eventually activates the subdivision of the NURBS into segments

		//printf("n>3*k, è operata una suddivisione della curva NURBS;  n=%d\n",nurbs_control_points.size() - 1-k);
		//printf("n>3*k, a subdivision of the NURBS curve is made;  n=%ld (F: %s L: %d)\n",nurbs_control_points.size() - 1 - k, __FILE__, __LINE__);
		NURBS_FEED_DIVIDE(lineno, nurbs_control_points,
				  knot_vector, nurbs_order, feedrate,
				  Q_option);
	} else {
		// printf("n<3*k, Non è operata nessuna suddivisione della curva NURBS,  n=%d\n",n);
		//printf("n<3*k, No subdivision of the NURBS curve is made,  n=%d (F: %s L: %d)\n",n, __FILE__, __LINE__);
		NURBS_FEED_NO_SUBDIVISION(lineno, nurbs_control_points,
					  knot_vector, nurbs_order,
					  feedrate, Q_option);
	}
}
/**
 * Simple circular shift function for PM_CARTESIAN type.
 * Cycle around axes without changing the individual values. A circshift of -1
 * makes the X value become the new Y, Y become the Z, and Z become the new X.
 */
static PM_CARTESIAN circshift(PM_CARTESIAN & vec, int steps)
{
    int X=0,Y=1,Z=2;

    int s = 3;
    // Use mod to cycle indices around by steps
    X = (X + steps + s) % s;
    Y = (Y + steps + s) % s;
    Z = (Z + steps + s) % s;
    return PM_CARTESIAN(vec[X],vec[Y],vec[Z]);
}

#if 0
static CANON_POSITION get_axis_max_velocity()
{
    CANON_POSITION maxvel;
    maxvel.x = axis_valid(0) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(0)) : 0.0;
    maxvel.y = axis_valid(1) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(1)) : 0.0;
    maxvel.z = axis_valid(2) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(2)) : 0.0;

    maxvel.a = axis_valid(3) ? FROM_EXT_ANG(emcAxisGetMaxVelocity(3)) : 0.0;
    maxvel.b = axis_valid(4) ? FROM_EXT_ANG(emcAxisGetMaxVelocity(4)) : 0.0;
    maxvel.c = axis_valid(5) ? FROM_EXT_ANG(emcAxisGetMaxVelocity(5)) : 0.0;

    maxvel.u = axis_valid(6) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(6)) : 0.0;
    maxvel.v = axis_valid(7) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(7)) : 0.0;
    maxvel.w = axis_valid(8) ? FROM_EXT_LEN(emcAxisGetMaxVelocity(8)) : 0.0;
    return maxvel;
}

static CANON_POSITION get_axis_max_acceleration()
{
    CANON_POSITION maxacc;
    maxacc.x = axis_valid(0) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(0)) : 0.0;
    maxacc.y = axis_valid(1) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(1)) : 0.0;
    maxacc.z = axis_valid(2) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(2)) : 0.0;

    maxacc.a = axis_valid(3) ? FROM_EXT_ANG(emcAxisGetMaxAcceleration(3)) : 0.0;
    maxacc.b = axis_valid(4) ? FROM_EXT_ANG(emcAxisGetMaxAcceleration(4)) : 0.0;
    maxacc.c = axis_valid(5) ? FROM_EXT_ANG(emcAxisGetMaxAcceleration(5)) : 0.0;

    maxacc.u = axis_valid(6) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(6)) : 0.0;
    maxacc.v = axis_valid(7) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(7)) : 0.0;
    maxacc.w = axis_valid(8) ? FROM_EXT_LEN(emcAxisGetMaxAcceleration(8)) : 0.0;
    return maxacc;
}

static double axis_motion_time(const CANON_POSITION & start, const CANON_POSITION & end)
{

    CANON_POSITION disp = end - start;
    CANON_POSITION times; 
    CANON_POSITION maxvel = get_axis_max_velocity();

    canon_debug(" in axis_motion_time\n");
    // For active axes, find the time required to reach the displacement in each axis
    int ind = 0;
    for (ind = 0; ind < 9; ++ind) {
        double v = maxvel[ind];
        if (v > 0.0) {
            times[ind] = fabs(disp[ind]) / v;
        } else {
            times[ind]=0;
        }
        canon_debug("  ind = %d, maxvel = %f, disp = %f, time = %f\n", ind, v, disp[ind], times[ind]);
    }

    return times.max();
}

// NOTE: not exactly times, comment TODO
static double axis_acc_time(const CANON_POSITION & start, const CANON_POSITION & end)
{

    CANON_POSITION disp = end - start;
    CANON_POSITION times; 
    CANON_POSITION maxacc = get_axis_max_acceleration();

    for (int i = 0; i < 9; ++i) {
        double a = maxacc[i];
        if (a > 0.0) {
            times[i] = fabs(disp[i]) / a;
        } else {
            times[i]=0;
        }
    }

    return times.max();
}
#endif

void ARC_FEED(int line_number, 
				double first_end, double second_end,
				double first_axis, double second_axis, int rotation,
				double axis_end_point, 
				double a, double b, double c,
				double u, double v, double w)
{

	auto circularMoveMsg = std::make_unique<EMC_TRAJ_CIRCULAR_MOVE>();
	auto linearMoveMsg = std::make_unique<EMC_TRAJ_LINEAR_MOVE>();

	canon_debug("line = %d\n", line_number);
	canon_debug("first_end = %f, second_end = %f\n", first_end,second_end);

    if( canon.activePlane == CANON_PLANE::XY && canon.motionMode == CANON_CONTINUOUS) {
		double mx, my;
		double lx, ly, lz;
		double unused = 0;

		get_last_pos(lx, ly, lz);

        double fe=FROM_PROG_LEN(first_end), se=FROM_PROG_LEN(second_end), ae=FROM_PROG_LEN(axis_end_point);
		double fa=FROM_PROG_LEN(first_axis), sa=FROM_PROG_LEN(second_axis);
		rotate_and_offset_pos(fe, se, ae, unused, unused, unused, unused, unused, unused);
		rotate_and_offset_pos(fa, sa, unused, unused, unused, unused, unused, unused, unused);
        if (chord_deviation(lx, ly, fe, se, fa, sa, rotation, mx, my) < canon.naivecamTolerance) {
			a = FROM_PROG_ANG(a);
			b = FROM_PROG_ANG(b);
			c = FROM_PROG_ANG(c);
			u = FROM_PROG_LEN(u);
			v = FROM_PROG_LEN(v);
			w = FROM_PROG_LEN(w);

			rotate_and_offset_pos(unused, unused, unused, a, b, c, u, v, w);
			see_segment(line_number, _tag, mx, my,
									(lz + ae)/2, 
									(canon.endPoint.a + a)/2, 
									(canon.endPoint.b + b)/2, 
									(canon.endPoint.c + c)/2, 
									(canon.endPoint.u + u)/2, 
									(canon.endPoint.v + v)/2, 
									(canon.endPoint.w + w)/2);
			see_segment(line_number, _tag, fe, se, ae, a, b, c, u, v, w);
			return;
			}
		}

    linearMoveMsg->feed_mode = canon.feed_mode;
    circularMoveMsg->feed_mode = canon.feed_mode;
    flush_segments();

    // Start by defining 3D points for the motion end and center.
    PM_CARTESIAN end_cart(first_end, second_end, axis_end_point);
    PM_CARTESIAN center_cart(first_axis, second_axis, axis_end_point);
    PM_CARTESIAN normal_cart(0.0,0.0,1.0);
    PM_CARTESIAN plane_x(1.0,0.0,0.0);
    PM_CARTESIAN plane_y(0.0,1.0,0.0);


    canon_debug("start = %f %f %f\n",
            canon.endPoint.x,
            canon.endPoint.y,
            canon.endPoint.z);
    canon_debug("end = %f %f %f\n",
            end_cart.x,
            end_cart.y,
            end_cart.z);
    canon_debug("center = %f %f %f\n",
            center_cart.x,
            center_cart.y,
            center_cart.z);

    // Rearrange the X Y Z coordinates in the correct order based on the active plane (XY, YZ, or XZ)
    // KLUDGE CANON_PLANE is 1-indexed, hence the subtraction here to make a 0-index value
    int shift_ind = 0;
    switch(canon.activePlane) {
        case CANON_PLANE::XY:
            shift_ind = 0;
            break;
        case CANON_PLANE::XZ:
            shift_ind = -2;
            break;
        case CANON_PLANE::YZ:
            shift_ind = -1;
            break;
        case CANON_PLANE::UV:
        case CANON_PLANE::VW:
        case CANON_PLANE::UW:
            CANON_ERROR("Can't set plane in UVW axes, assuming XY");
            break;
    }

    canon_debug("active plane is %d, shift_ind is %d\n",canon.activePlane,shift_ind);
    end_cart = circshift(end_cart, shift_ind);
    center_cart = circshift(center_cart, shift_ind);
    normal_cart = circshift(normal_cart, shift_ind);
    plane_x = circshift(plane_x, shift_ind);
    plane_y = circshift(plane_y, shift_ind);

    canon_debug("normal = %f %f %f\n",
            normal_cart.x,
            normal_cart.y,
            normal_cart.z);

    canon_debug("plane_x = %f %f %f\n",
            plane_x.x,
            plane_x.y,
            plane_x.z);

    canon_debug("plane_y = %f %f %f\n",
            plane_y.x,
            plane_y.y,
            plane_y.z);
    // Define end point in PROGRAM units and convert to CANON
    CANON_POSITION endpt(0,0,0,a,b,c,u,v,w);
    from_prog(endpt);

    // Store permuted XYZ end position
    from_prog_len(end_cart);
    endpt.set_xyz(end_cart);

    // Convert to CANON units
    from_prog_len(center_cart);

    // Rotate and offset the new end point to be in the same coordinate system as the current end point
    rotate_and_offset(endpt);
    rotate_and_offset_xyz(center_cart);
    rotate_and_offset_xyz(end_cart);
    // Also rotate the basis vectors
    to_rotated(plane_x);
    to_rotated(plane_y);
    to_rotated(normal_cart);

    canon_debug("end = %f %f %f\n",
            end_cart.x,
            end_cart.y,
            end_cart.z);

    canon_debug("endpt = %f %f %f\n",
            endpt.x,
            endpt.y,
            endpt.z);
    canon_debug("center = %f %f %f\n",
            center_cart.x,
            center_cart.y,
            center_cart.z);

    canon_debug("normal = %f %f %f\n",
            normal_cart.x,
            normal_cart.y,
            normal_cart.z);
    // Note that the "start" point is already rotated and offset

    // Define displacement vectors from center to end and center to start (3D)
    PM_CARTESIAN end_rel = end_cart - center_cart;
    PM_CARTESIAN start_rel = canon.endPoint.xyz() - center_cart;

    // Project each displacement onto the active plane
    double p_end_1 = dot(end_rel,plane_x);
    double p_end_2 = dot(end_rel,plane_y);
    double p_start_1 = dot(start_rel,plane_x);
    double p_start_2 = dot(start_rel,plane_y);

    canon_debug("planar end = %f %f\n", p_end_1, p_end_2);
    canon_debug("planar start = %f %f\n", p_start_1, p_start_2);

    canon_debug("rotation = %d\n",rotation);

    // Use the "X" (1) and Y" (2) components of the planar projections to get
    // the starting and ending angle. Note that atan2 arguments are atan2(Y,X).
    double theta_start = atan2(p_start_2, p_start_1);
    double theta_end= atan2(p_end_2,p_end_1);
    double start_radius = hypot(p_start_1, p_start_2);
    double end_radius = hypot(p_end_1, p_end_2);
    canon_debug("radius = %f\n",start_radius);
    canon_debug("raw values: theta_end = %.17e, theta_start = %.17e\n", theta_end, theta_start);

    // Correct for angle wrap so that theta_end - theta_start > 0
    int is_clockwise = rotation < 0;

    // FIXME should be a constant in canon.hh or elsewhere
    const double min_arc_angle = 1e-12;

    if (is_clockwise) {
        if((theta_end + min_arc_angle) >= theta_start) theta_end -= M_PI * 2.0;
    } else {
        if((theta_end - min_arc_angle) <= theta_start) theta_end += M_PI * 2.0;
    }

    canon_debug("theta_end = %f, theta_start = %f\n", theta_end, theta_start);

    /*
       mapping of rotation to full turns:

       rotation full COUNTERCLOCKWISE turns (- implies clockwise)
       -------- -----
              0 none (linear move)
              1 0
              2 1
             -1 0
             -2 -1 
    */

    // Compute the number of FULL turns in addition to the principal angle
    int full_turns = 0;
    if (rotation > 1) {
        full_turns = rotation - 1;
    }
    if (rotation < -1) {
        full_turns = rotation + 1;
    }

    double angle = theta_end - theta_start;
    double full_angle = angle + 2.0 * M_PI * (double)full_turns;
    canon_debug("angle = %f\n", angle);
    canon_debug("full turns = %d\n", full_turns);

		canon_debug("full_angle = %.17e\n", full_angle);

    //Use total angle to get spiral properties
    double spiral = end_radius - start_radius;
    double dr = spiral / fabs(full_angle);
    double min_radius = fmin(start_radius, end_radius);
    double effective_radius = sqrt(dr*dr + min_radius*min_radius);

    // KLUDGE: assumes 0,1,2 for X Y Z
    // Find normal axis
    int norm_axis_ind = (2 - shift_ind) % 3;
    // Find maximum velocities and accelerations for planar axes
    int axis1 = (norm_axis_ind + 1) % 3;
    int axis2 = (norm_axis_ind + 2) % 3;

    canon_debug("axis1 = %d, axis2 = %d\n",axis1, axis2);

    // Get planar velocity bounds
    double v1 = FROM_EXT_LEN(emcAxisGetMaxVelocity(axis1));
    double v2 = FROM_EXT_LEN(emcAxisGetMaxVelocity(axis2));
    double v_max_axes = std::min(v1, v2);

    // Get planar acceleration bounds
    double a1 = FROM_EXT_LEN(emcAxisGetMaxAcceleration(axis1));
    double a2 = FROM_EXT_LEN(emcAxisGetMaxAcceleration(axis2));
    double a_max_axes = std::min(a1, a2);

    if(canon.xy_rotation && canon.activePlane != CANON_PLANE::XY) {
        // also consider the third plane's constraint, which may get
        // involved since we're rotated.

        int axis3 = (norm_axis_ind + 3) % 3;
        if (axis_valid(axis3)) {
            double v3 = FROM_EXT_LEN(emcAxisGetMaxVelocity(axis3));
            double a3 = FROM_EXT_LEN(emcAxisGetMaxAcceleration(axis3));
            v_max_axes = std::min(v3, v_max_axes);
            a_max_axes = std::min(a3, a_max_axes);
        }
    }

    //FIXME allow tangential acceleration like in TP
    double a_max_normal = a_max_axes * sqrt(3.0)/2.0;
    canon_debug("a_max_axes = %f\n", a_max_axes);

    // Compute the centripetal acceleration
    double v_max_radial = sqrt(a_max_normal * effective_radius);
    canon_debug("v_max_radial = %f\n", v_max_radial);

    // Restrict our maximum velocity in-plane if need be
    double v_max_planar = std::min(v_max_radial, v_max_axes);
    canon_debug("v_max_planar = %f\n", v_max_planar);

    // Find the equivalent maximum velocity for a linear displacement
    // This accounts for speed restrictions due to helical and other axes
    VelData veldata = getStraightVelocity(endpt);

    // Compute spiral length, first by the minimum circular arc length
    double circular_length = min_radius * fabs(full_angle);
    // Then by linear approximation of the spiral arc length function of angle
    // TODO use quadratic approximation
    double spiral_length = hypot(circular_length, spiral);

    // Compute length along normal axis and total XYZ arc length
    double axis_len = dot(end_cart - canon.endPoint.xyz(), normal_cart);
    double total_xyz_length = hypot(spiral_length, axis_len);

    // Next, compute the minimum time that we must take to complete the segment. 
    // The motion computation gives us min time needed for the helical and auxiliary axes
    double t_max_motion = veldata.tmax;
    // Assumes worst case that velocity can be in any direction in the plane, so
    // we assume tangential velocity is always less than the planar velocity limit.
    // The spiral time is the min time needed to stay under the planar velocity limit.
    double t_max_spiral = spiral_length / v_max_planar;

    // Now, compute actual XYZ max velocity from this min time and the total arc length
    double t_max = fmax(t_max_motion, t_max_spiral);

    double v_max = total_xyz_length / t_max;
    canon_debug("v_max = %f\n", v_max);


//COMPUTE ACCEL
    
    // Use "straight" acceleration measure to compute acceleration bounds due
    // to non-circular components (helical axis, other axes)
    AccelData accdata = getStraightAcceleration(endpt);

    double tt_max_motion = accdata.tmax;
    double tt_max_spiral = spiral_length / a_max_axes;
    double tt_max = fmax(tt_max_motion, tt_max_spiral);

    // a_max could be higher than a_max_axes, but the projection onto the
    // circle plane and helical axis will still be within limits
    double a_max = total_xyz_length / tt_max;

    // Limit velocity by maximum
    double vel = std::min(canon.linearFeedRate, v_max);
    canon_debug("current F = %f\n",canon.linearFeedRate);
    canon_debug("vel = %f\n",vel);

    canon_debug("v_max = %f\n",v_max);
    canon_debug("a_max = %f\n",a_max);

    canon.cartesian_move = 1;

    if (rotation == 0) {
        // linear move
        // FIXME (Rob) Am I missing something? the P word should never be zero,
        // or we wouldn't be calling ARC_FEED
        linearMoveMsg->end = to_ext_pose(endpt);
        linearMoveMsg->type = EMC_MOTION_TYPE_ARC;
        linearMoveMsg->vel = toExtVel(vel);
        linearMoveMsg->ini_maxvel = toExtVel(v_max);
        linearMoveMsg->acc = toExtAcc(a_max);
        linearMoveMsg->indexer_jnum = -1;
        if(vel && a_max){
            interp_list.set_line_number(line_number);
            tag_and_send(std::move(linearMoveMsg), _tag);
        }
    } else {
        circularMoveMsg->end = to_ext_pose(endpt);

        // Convert internal center and normal to external units
        circularMoveMsg->center = to_ext_len(center_cart);
        circularMoveMsg->normal = to_ext_len(normal_cart);

        if (rotation > 0)
            circularMoveMsg->turn = rotation - 1;
        else
            // reverse turn
            circularMoveMsg->turn = rotation;

        circularMoveMsg->type = EMC_MOTION_TYPE_ARC;

        circularMoveMsg->vel = toExtVel(vel);
        circularMoveMsg->ini_maxvel = toExtVel(v_max);
        circularMoveMsg->acc = toExtAcc(a_max);

        //FIXME what happens if accel or vel is zero?
        // The end point is still updated, but nothing is added to the interp list
        // seems to be a crude way to indicate a zero length segment?
        if(vel && a_max) {
            interp_list.set_line_number(line_number);
            tag_and_send(std::move(circularMoveMsg), _tag);
        }
    }
    // update the end point
    canonUpdateEndPoint(endpt);
}


void DWELL(double seconds)
{
    auto delayMsg = std::make_unique<EMC_TRAJ_DELAY>();

    flush_segments();

    delayMsg->delay = seconds;

    interp_list.append(std::move(delayMsg));
}

/* Spindle Functions */
void SPINDLE_RETRACT_TRAVERSE()
{
    /*! \todo FIXME-- unimplemented */
}

void SET_SPINDLE_MODE(int spindle, double css_max) {
   canon.spindle[spindle].css_maximum = fabs(css_max);
}

template <class MSG=EMC_SPINDLE_ON>
auto SPINDLE_SPEED_(int s, int dir, double speed)
{
    auto emc_spindle_msg = std::make_unique<MSG>();

    flush_segments();
    if (dir != 0)
        canon.spindle[s].dir = dir;
    if (speed != 0)
    	canon.spindle[s].speed = fabs(speed); // interp will never send negative anyway ...

    emc_spindle_msg->spindle = s;
    if(canon.spindle[s].css_maximum) {
        if(canon.lengthUnits == CANON_UNITS_INCHES){
            canon.spindle[s].css_factor = 12 / (2 * M_PI) * canon.spindle[s].speed * TO_EXT_LEN(25.4);
        } else {
            canon.spindle[s].css_factor = 1000 / (2 * M_PI) * canon.spindle[s].speed * TO_EXT_LEN(1);
		}
		emc_spindle_msg->speed = canon.spindle[s].dir * canon.spindle[s].css_maximum;
		emc_spindle_msg->factor = canon.spindle[s].dir * canon.spindle[s].css_factor;
		emc_spindle_msg->xoffset = TO_EXT_LEN(canon.g5xOffset.x + canon.g92Offset.x + canon.toolOffset.tran.x);
    } else {
        emc_spindle_msg->speed = canon.spindle[s].dir * canon.spindle[s].speed;
     //   canon.css_numerator = 0; FIXME: Do we need this?
    }

    return std::move(emc_spindle_msg);
}

void START_SPINDLE_CLOCKWISE(int s, int wait_for_atspeed)
{
    auto msg = SPINDLE_SPEED_<>(s, 1, 0);
    msg->wait_for_spindle_at_speed = wait_for_atspeed;
    interp_list.append(std::move(msg));
}

void START_SPINDLE_COUNTERCLOCKWISE(int s, int wait_for_atspeed)
{
    auto msg = SPINDLE_SPEED_<>(s, -1, 0);
    msg->wait_for_spindle_at_speed = wait_for_atspeed;
    interp_list.append(std::move(msg));
}

void SET_SPINDLE_SPEED(int s, double speed_rpm)
{
    interp_list.append(SPINDLE_SPEED_<EMC_SPINDLE_SPEED>(s, 0, speed_rpm));
}

void STOP_SPINDLE_TURNING(int s)
{
    auto emc_spindle_off_msg = std::make_unique<EMC_SPINDLE_OFF>();

    flush_segments();
    emc_spindle_off_msg->spindle = s;
    interp_list.append(std::move(emc_spindle_off_msg));
    // Added by atp 6/1/18 not sure this is right. There is a problem that the _second_ S word starts the spindle without M3/M4
    canon.spindle[s].dir = 0;
}

void SPINDLE_RETRACT()
{
    /*! \todo FIXME-- unimplemented */
}

void ORIENT_SPINDLE(int s, double orientation, int mode)
{
    auto o = std::make_unique<EMC_SPINDLE_ORIENT>();

    flush_segments();
    o->spindle = s;
    o->orientation = orientation;
    o->mode = mode;
    interp_list.append(std::move(o));
}

void WAIT_SPINDLE_ORIENT_COMPLETE(int s, double timeout)
{
    auto o = std::make_unique<EMC_SPINDLE_WAIT_ORIENT_COMPLETE>();

    flush_segments();
    o->spindle = s;
    o->timeout = timeout;
    interp_list.append(std::move(o));
}

void USE_SPINDLE_FORCE(void)
{
    /*! \todo FIXME-- unimplemented */
}

void LOCK_SPINDLE_Z(void)
{
    /*! \todo FIXME-- unimplemented */
}

void USE_NO_SPINDLE_FORCE(void)
{
    /*! \todo FIXME-- unimplemented */
}

/* Tool Functions */

/* this is called with distances in external (machine) units */
void SET_TOOL_TABLE_ENTRY(int pocket, int toolno, const EmcPose& offset, double diameter,
                          double frontangle, double backangle, int orientation) {
    auto o = std::make_unique<EMC_TOOL_SET_OFFSET>();
    flush_segments();
    o->pocket = pocket;
    o->toolno = toolno;
    o->offset = offset;
    o->diameter = diameter;
    o->frontangle = frontangle;
    o->backangle = backangle;
    o->orientation = orientation;
    interp_list.append(std::move(o));
}

/*
  EMC has no tool length offset. To implement it, we save it here,
  and apply it when necessary
  */
void USE_TOOL_LENGTH_OFFSET(const EmcPose& offset)
{
    auto set_offset_msg = std::make_unique<EMC_TRAJ_SET_OFFSET>();

    flush_segments();

    /* convert to mm units for internal canonical use */
    canon.toolOffset.tran.x = FROM_PROG_LEN(offset.tran.x);
    canon.toolOffset.tran.y = FROM_PROG_LEN(offset.tran.y);
    canon.toolOffset.tran.z = FROM_PROG_LEN(offset.tran.z);
    canon.toolOffset.a = FROM_PROG_ANG(offset.a);
    canon.toolOffset.b = FROM_PROG_ANG(offset.b);
    canon.toolOffset.c = FROM_PROG_ANG(offset.c);
    canon.toolOffset.u = FROM_PROG_LEN(offset.u);
    canon.toolOffset.v = FROM_PROG_LEN(offset.v);
    canon.toolOffset.w = FROM_PROG_LEN(offset.w);

    /* append it to interp list so it gets updated at the right time, not at
       read-ahead time */
    set_offset_msg->offset.tran.x = TO_EXT_LEN(canon.toolOffset.tran.x);
    set_offset_msg->offset.tran.y = TO_EXT_LEN(canon.toolOffset.tran.y);
    set_offset_msg->offset.tran.z = TO_EXT_LEN(canon.toolOffset.tran.z);
    set_offset_msg->offset.a = TO_EXT_ANG(canon.toolOffset.a);
    set_offset_msg->offset.b = TO_EXT_ANG(canon.toolOffset.b);
    set_offset_msg->offset.c = TO_EXT_ANG(canon.toolOffset.c);
    set_offset_msg->offset.u = TO_EXT_LEN(canon.toolOffset.u);
    set_offset_msg->offset.v = TO_EXT_LEN(canon.toolOffset.v);
    set_offset_msg->offset.w = TO_EXT_LEN(canon.toolOffset.w);

    for (int s = 0; s < emcStatus->motion.traj.spindles; s++){
        if(canon.spindle[s].css_maximum) {
            SET_SPINDLE_SPEED(s, canon.spindle[s].speed);
        }
    }
    interp_list.append(std::move(set_offset_msg));
}

/* CHANGE_TOOL results from M6 */
void CHANGE_TOOL()
{
    auto load_tool_msg = std::make_unique<EMC_TOOL_LOAD>();

    flush_segments();

    /* optional move to tool change position.  This
     * is a mess because we really want a configurable chain
     * of events to happen when a tool change is called for.
     * Since they'll probably involve motion, we can't just
     * do it in HAL.  This is basic support for making one
     * move to a particular coordinate before the tool change
     * is called.  */
    
    if (have_tool_change_position) {
        double vel, acc, x, y, z, a, b, c, u, v, w;

        x = FROM_EXT_LEN(tool_change_position.tran.x);
        y = FROM_EXT_LEN(tool_change_position.tran.y);
        z = FROM_EXT_LEN(tool_change_position.tran.z);
        a = canon.endPoint.a;
        b = canon.endPoint.b;
        c = canon.endPoint.c;
        u = canon.endPoint.u;
        v = canon.endPoint.v;
        w = canon.endPoint.w;

        if (have_tool_change_position > 3) {
            a = FROM_EXT_ANG(tool_change_position.a);
            b = FROM_EXT_ANG(tool_change_position.b);
            c = FROM_EXT_ANG(tool_change_position.c);
        }

        if (have_tool_change_position > 6) {
            u = FROM_EXT_LEN(tool_change_position.u);
            v = FROM_EXT_LEN(tool_change_position.v);
            w = FROM_EXT_LEN(tool_change_position.w);
        }

        VelData veldata = getStraightVelocity(x, y, z, a, b, c, u, v, w);
        AccelData accdata = getStraightAcceleration(x, y, z, a, b, c, u, v, w);
        vel = veldata.vel;
        acc = accdata.acc;

        auto linearMoveMsg = std::make_unique<EMC_TRAJ_LINEAR_MOVE>();
        linearMoveMsg->feed_mode = canon.feed_mode;

        linearMoveMsg->end = to_ext_pose(x, y, z, a, b, c, u, v, w);

        linearMoveMsg->vel = linearMoveMsg->ini_maxvel = toExtVel(vel);
        linearMoveMsg->acc = toExtAcc(acc);
        linearMoveMsg->type = EMC_MOTION_TYPE_TOOLCHANGE;
	    linearMoveMsg->feed_mode = 0;
        linearMoveMsg->indexer_jnum = -1;

	    int old_feed_mode = canon.feed_mode;
	    if(canon.feed_mode)
	       STOP_SPEED_FEED_SYNCH();

        if(vel && acc)
            tag_and_send(std::move(linearMoveMsg), _tag);

        if(old_feed_mode)
            START_SPEED_FEED_SYNCH(canon.spindle_num, canon.linearFeedRate, 1);

        canonUpdateEndPoint(x, y, z, a, b, c, u, v, w);
    }

    /* regardless of optional moves above, we'll always send a load tool
       message */
    interp_list.append(std::move(load_tool_msg));
}

/* SELECT_TOOL results from Tn */
void SELECT_TOOL(int tool)
{
    auto prep_for_tool_msg = std::make_unique<EMC_TOOL_PREPARE>();

    prep_for_tool_msg->tool = tool;

    interp_list.append(std::move(prep_for_tool_msg));
}

/* CHANGE_TOOL_NUMBER results from M61 */
void CHANGE_TOOL_NUMBER(int pocket_number)
{
    auto emc_tool_set_number_msg = std::make_unique<EMC_TOOL_SET_NUMBER>();
    
    emc_tool_set_number_msg->tool = pocket_number;

    interp_list.append(std::move(emc_tool_set_number_msg));
}

void RELOAD_TOOLDATA(void)
{
    auto load_tool_table_msg = std::make_unique<EMC_TOOL_LOAD_TOOL_TABLE>();
    interp_list.append(std::move(load_tool_table_msg));
}

/* Misc Functions */

void CLAMP_AXIS(CANON_AXIS /*axis*/)
{
    /*! \todo FIXME-- unimplemented */
}

/*
  setString and addString initializes or adds src to dst, never exceeding
  dst's maxlen chars.
*/

static char *setString(char *dst, const char *src, int maxlen)
{
    dst[0] = 0;
    strncat(dst, src, maxlen - 1);
    dst[maxlen - 1] = 0;
    return dst;
}

static char *addString(char *dst, const char *src, int maxlen)
{
    int dstlen = strlen(dst);
    int srclen = strlen(src);
    int actlen;

    if (srclen >= maxlen - dstlen) {
	actlen = maxlen - dstlen - 1;
	dst[maxlen - 1] = 0;
    } else {
	actlen = srclen;
    }

    strncat(dst, src, actlen);

    return dst;
}

/*
  The probe file is opened with a hot-comment (PROBEOPEN <filename>),
  and the results of each probed point are written to that file.
  The file is closed with a (PROBECLOSE) comment.
*/

static FILE *probefile = NULL;

void COMMENT(const char *comment)
{
    // nothing need be done here, but you can play tricks with hot comments

    char msg[LINELEN];
    char probefilename[LINELEN];
    const char *ptr;

    // set RPY orientation for subsequent moves
    if (!strncmp(comment, "RPY", strlen("RPY"))) {
	PM_RPY rpy;
	// it's RPY <R> <P> <Y>
	if (3 !=
	    sscanf(comment, "%*s %lf %lf %lf", &rpy.r, &rpy.p, &rpy.y)) {
	    // print current orientation
	    printf("rpy = %f %f %f, quat = %f %f %f %f\n",
		   rpy.r, rpy.p, rpy.y, quat.s, quat.x, quat.y, quat.z);
	} else {
	    // set and print orientation
	    quat = rpy;
	    printf("rpy = %f %f %f, quat = %f %f %f %f\n",
		   rpy.r, rpy.p, rpy.y, quat.s, quat.x, quat.y, quat.z);
	}
	return;
    }
    // open probe output file
    if (!strncmp(comment, "PROBEOPEN", strlen("PROBEOPEN"))) {
	// position ptr to first char after PROBEOPEN
	ptr = &comment[strlen("PROBEOPEN")];
	// and step over white space to name, or NULL
	while (isspace(*ptr)) {
	    ptr++;
	}
	setString(probefilename, ptr, LINELEN);
	if (NULL == (probefile = fopen(probefilename, "wt"))) {
	    // pop up a warning message
	    setString(msg, "can't open probe file ", LINELEN);
	    addString(msg, probefilename, LINELEN);
	    MESSAGE(msg);
	    probefile = NULL;
	}
	return;
    }
    // close probe output file
    if (!strncmp(comment, "PROBECLOSE", strlen("PROBECLOSE"))) {
	if (probefile != NULL) {
	    fclose(probefile);
	    probefile = NULL;
	}
	return;
    }

    return;
}

// refers to feed rate
void FEED_OVERRIDE_ENABLE_(int mode)
{
    auto set_fo_enable_msg = std::make_unique<EMC_TRAJ_SET_FO_ENABLE>();
    flush_segments();
    
    set_fo_enable_msg->mode = mode;
    interp_list.append(std::move(set_fo_enable_msg));
}

void DISABLE_FEED_OVERRIDE()
{
    FEED_OVERRIDE_ENABLE_(0);
}

void ENABLE_FEED_OVERRIDE()
{
    FEED_OVERRIDE_ENABLE_(1);
}


//refers to adaptive feed override (HAL input, useful for EDM for example)
void ADAPTIVE_FEED_ENABLE_(int status)
{
    auto emcmotAdaptiveMsg = std::make_unique<EMC_MOTION_ADAPTIVE>();
    flush_segments();

    emcmotAdaptiveMsg->status = status;
    interp_list.append(std::move(emcmotAdaptiveMsg));
}

void DISABLE_ADAPTIVE_FEED()
{
    ADAPTIVE_FEED_ENABLE_(0);
}

void ENABLE_ADAPTIVE_FEED()
{
    ADAPTIVE_FEED_ENABLE_(1);
}

void SPEED_OVERRIDE_(int spindle, int mode)
{
    auto set_so_enable_msg = std::make_unique<EMC_TRAJ_SET_SO_ENABLE>();
    flush_segments();
    
    set_so_enable_msg->mode = mode;
    set_so_enable_msg->spindle = spindle;
    interp_list.append(std::move(set_so_enable_msg));
}
//refers to spindle speed
void DISABLE_SPEED_OVERRIDE(int spindle)
{
    SPEED_OVERRIDE_(spindle, 0);
}

void ENABLE_SPEED_OVERRIDE(int spindle)
{
    SPEED_OVERRIDE_(spindle, 1);
}

void FEED_HOLD_(int mode)
{
    auto set_feed_hold_msg = std::make_unique<EMC_TRAJ_SET_FH_ENABLE>();
    flush_segments();

    set_feed_hold_msg->mode = mode;
    interp_list.append(std::move(set_feed_hold_msg));
}
void ENABLE_FEED_HOLD()
{
    FEED_HOLD_(1);
}

void DISABLE_FEED_HOLD()
{
    FEED_HOLD_(0);
}

template <typename T, bool flush_segments_ = true>
void SIMPLE_COMMAND_()
{
    if (flush_segments_)
        flush_segments();

    interp_list.append(std::make_unique<T>());
}

void FLOOD_OFF()
{
    SIMPLE_COMMAND_<EMC_COOLANT_FLOOD_OFF>();
}

void FLOOD_ON()
{
    SIMPLE_COMMAND_<EMC_COOLANT_FLOOD_ON>();
}

void MESSAGE(char *s)
{
    auto operator_display_msg = std::make_unique<EMC_OPERATOR_DISPLAY>();

    flush_segments();
    strncpy(operator_display_msg->display, s, LINELEN);
    operator_display_msg->display[LINELEN - 1] = 0;
    interp_list.append(std::move(operator_display_msg));
}

static FILE *logfile = NULL;

void LOG(char *s) {
    flush_segments();
    if(logfile) { fprintf(logfile, "%s\n", s); fflush(logfile); }
    fprintf(stderr, "LOG(%s)\n", s);

}

void LOGOPEN(char *name) {
    if(logfile) fclose(logfile);
    logfile = fopen(name, "wt");
    fprintf(stderr, "LOGOPEN(%s) -> %p\n", name, logfile);
}

void LOGAPPEND(char *name) {
    if(logfile) fclose(logfile);
    logfile = fopen(name, "at");
    fprintf(stderr, "LOGAPPEND(%s) -> %p\n", name, logfile);
}


void LOGCLOSE() {
    if(logfile) fclose(logfile);
    logfile = NULL;
    fprintf(stderr, "LOGCLOSE()\n");
}

void MIST_OFF()
{
    SIMPLE_COMMAND_<EMC_COOLANT_MIST_OFF>();
}

void MIST_ON()
{
    SIMPLE_COMMAND_<EMC_COOLANT_MIST_ON>();
}

void PALLET_SHUTTLE()
{
    /*! \todo FIXME-- unimplemented */
}

void TURN_PROBE_OFF()
{
    // don't do anything-- this is called when the probing is done
}

void TURN_PROBE_ON()
{
    SIMPLE_COMMAND_<EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG, false>();
}

void UNCLAMP_AXIS(CANON_AXIS /*axis*/)
{
    /*! \todo FIXME-- unimplemented */
}

/* Program Functions */

void PROGRAM_STOP()
{
    /* 
       implement this as a pause. A resume will cause motion to proceed. */
    SIMPLE_COMMAND_<EMC_TASK_PLAN_PAUSE>();
}

void SET_BLOCK_DELETE(bool state)
{
    canon.block_delete = state; //state == ON, means we don't interpret lines starting with "/"
}

bool GET_BLOCK_DELETE()
{
    return canon.block_delete; //state == ON, means we  don't interpret lines starting with "/"
}


void SET_OPTIONAL_PROGRAM_STOP(bool state)
{
    canon.optional_program_stop = state; //state == ON, means we stop
}

bool GET_OPTIONAL_PROGRAM_STOP()
{
    return canon.optional_program_stop; //state == ON, means we stop
}

void OPTIONAL_PROGRAM_STOP()
{
    SIMPLE_COMMAND_<EMC_TASK_PLAN_OPTIONAL_STOP>();
}

void PROGRAM_END()
{
    flush_segments();
    SIMPLE_COMMAND_<EMC_TASK_PLAN_END, false>();
}

double GET_EXTERNAL_TOOL_LENGTH_XOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.tran.x);
}

double GET_EXTERNAL_TOOL_LENGTH_YOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.tran.y);
}

double GET_EXTERNAL_TOOL_LENGTH_ZOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.tran.z);
}

double GET_EXTERNAL_TOOL_LENGTH_AOFFSET()
{
    return TO_PROG_ANG(canon.toolOffset.a);
}

double GET_EXTERNAL_TOOL_LENGTH_BOFFSET()
{
    return TO_PROG_ANG(canon.toolOffset.b);
}

double GET_EXTERNAL_TOOL_LENGTH_COFFSET()
{
    return TO_PROG_ANG(canon.toolOffset.c);
}

double GET_EXTERNAL_TOOL_LENGTH_UOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.u);
}

double GET_EXTERNAL_TOOL_LENGTH_VOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.v);
}

double GET_EXTERNAL_TOOL_LENGTH_WOFFSET()
{
    return TO_PROG_LEN(canon.toolOffset.w);
}

/*
  INIT_CANON()
  Initialize canonical local variables to defaults
  */
void INIT_CANON()
{
    double units;

    chained_points.clear();

    // initialize locals to original values
    canon.xy_rotation = 0.0;
    canon.rotary_unlock_for_traverse = -1;
    canon.feed_mode = 0;
    canon.g5xOffset.x = 0.0;
    canon.g5xOffset.y = 0.0;
    canon.g5xOffset.z = 0.0;
    canon.g5xOffset.a = 0.0;
    canon.g5xOffset.b = 0.0;
    canon.g5xOffset.c = 0.0;
    canon.g5xOffset.u = 0.0;
    canon.g5xOffset.v = 0.0;
    canon.g5xOffset.w = 0.0;
    canon.g92Offset.x = 0.0;
    canon.g92Offset.y = 0.0;
    canon.g92Offset.z = 0.0;
    canon.g92Offset.a = 0.0;
    canon.g92Offset.b = 0.0;
    canon.g92Offset.c = 0.0;
    canon.g92Offset.u = 0.0;
    canon.g92Offset.v = 0.0;
    canon.g92Offset.w = 0.0;
    SELECT_PLANE(CANON_PLANE::XY);
    canonUpdateEndPoint(0, 0, 0, 0, 0, 0, 0, 0, 0);
    SET_NAIVECAM_TOLERANCE(0);
    for (int s = 0; s < EMCMOT_MAX_SPINDLES; s++) {
        canon.spindle[s].speed = 0.0;
        canon.spindle[s].synched = 0;
    }
    canon.optional_program_stop = ON; //set enabled by default (previous EMC behaviour)
    canon.block_delete = ON; //set enabled by default (previous EMC behaviour)
    canon.cartesian_move = 0;
    canon.angular_move = 0;
    canon.linearFeedRate = 0.0;
    canon.angularFeedRate = 0.0;
    ZERO_EMC_POSE(canon.toolOffset);

    /* 
       to set the units, note that GET_EXTERNAL_LENGTH_UNITS() returns
       traj->linearUnits, which is already set from the INI file in
       iniTraj(). This is a floating point number, in user units per mm. We
       can compare this against known values and set the symbolic values
       accordingly. If it doesn't match, we have an error. */
    units = GET_EXTERNAL_LENGTH_UNITS();
    if (fabs(units - 1.0 / 25.4) < 1.0e-3) {
	canon.lengthUnits = CANON_UNITS_INCHES;
    } else if (fabs(units - 1.0) < 1.0e-3) {
	canon.lengthUnits = CANON_UNITS_MM;
    } else {
	CANON_ERROR
	    ("non-standard length units, setting interpreter to mm");
	canon.lengthUnits = CANON_UNITS_MM;
    }
    /* Set blending tolerance default depending on units machine is based on*/
    if (canon.lengthUnits == CANON_UNITS_INCHES) {
        SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, .001);
    } else {
        SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS,  .001 * MM_PER_INCH);
    }
}

/* Sends error message */
void CANON_ERROR(const char *fmt, ...)
{
    va_list ap;
    auto operator_error_msg = std::make_unique<EMC_OPERATOR_ERROR>();

    flush_segments();

    if (fmt != NULL) {
	    va_start(ap, fmt);
	    vsnprintf(operator_error_msg->error, sizeof(operator_error_msg->error), fmt, ap);
	    va_end(ap);
    } else {
	    operator_error_msg->error[0] = 0;
    }

    interp_list.append(std::move(operator_error_msg));
}

/*
  GET_EXTERNAL_TOOL_TABLE(int pocket)

  Returns the tool table structure associated with pocket. Note that
  pocket can run from 0 (by definition, the spindle), to pocket CANON_POCKETS_MAX - 1.

  Tool table is always in machine units.

  */
CANON_TOOL_TABLE GET_EXTERNAL_TOOL_TABLE(int idx)
{
    CANON_TOOL_TABLE tdata;

    if (idx < 0 || idx >= CANON_POCKETS_MAX) {
        tdata.toolno = -1;
        tdata.pocketno = 0;
        ZERO_EMC_POSE(tdata.offset);
        tdata.frontangle = 0.0;
        tdata.backangle = 0.0;
        tdata.diameter = 0.0;
        tdata.orientation = 0;
    } else {
        if (tooldata_get(&tdata,idx) != IDX_OK) {
            fprintf(stderr,"UNEXPECTED idx %s %d\n",__FILE__,__LINE__);
        }
    }
    return tdata;
}

CANON_POSITION GET_EXTERNAL_POSITION()
{
    CANON_POSITION position;
    EmcPose pos;

    drop_segments();

    pos = emcStatus->motion.traj.position;

    if (GET_EXTERNAL_OFFSET_APPLIED() ) {
        EmcPose eoffset = GET_EXTERNAL_OFFSETS();
        pos.tran.x -= eoffset.tran.x;
        pos.tran.y -= eoffset.tran.y;
        pos.tran.z -= eoffset.tran.z;
        pos.a      -= eoffset.a;
        pos.b      -= eoffset.b;
        pos.c      -= eoffset.c;
        pos.u      -= eoffset.u;
        pos.v      -= eoffset.v;
        pos.w      -= eoffset.w;
    }

    // first update internal record of last position
    canonUpdateEndPoint(FROM_EXT_LEN(pos.tran.x), FROM_EXT_LEN(pos.tran.y), FROM_EXT_LEN(pos.tran.z),
                        FROM_EXT_ANG(pos.a), FROM_EXT_ANG(pos.b), FROM_EXT_ANG(pos.c),
                        FROM_EXT_LEN(pos.u), FROM_EXT_LEN(pos.v), FROM_EXT_LEN(pos.w));

    // now calculate position in program units, for interpreter
    position = unoffset_and_unrotate_pos(canon.endPoint);
    to_prog(position);

    return position;
}

CANON_POSITION GET_EXTERNAL_PROBE_POSITION()
{
    CANON_POSITION position;
    EmcPose pos;
    static CANON_POSITION last_probed_position;

    flush_segments();

    pos = emcStatus->motion.traj.probedPosition;

    // first update internal record of last position
    pos.tran.x = FROM_EXT_LEN(pos.tran.x);
    pos.tran.y = FROM_EXT_LEN(pos.tran.y);
    pos.tran.z = FROM_EXT_LEN(pos.tran.z);

    pos.a = FROM_EXT_ANG(pos.a);
    pos.b = FROM_EXT_ANG(pos.b);
    pos.c = FROM_EXT_ANG(pos.c);

    pos.u = FROM_EXT_LEN(pos.u);
    pos.v = FROM_EXT_LEN(pos.v);
    pos.w = FROM_EXT_LEN(pos.w);

    // now calculate position in program units, for interpreter
    position = unoffset_and_unrotate_pos(pos);
    to_prog(position);

    if (probefile != NULL) {
	if (last_probed_position != position) {
	    fprintf(probefile, "%f %f %f %f %f %f %f %f %f\n",
                    position.x, position.y, position.z,
                    position.a, position.b, position.c,
                    position.u, position.v, position.w);
	    last_probed_position = position;
	}
    }

    return position;
}

int GET_EXTERNAL_PROBE_TRIPPED_VALUE()
{
    return emcStatus->motion.traj.probe_tripped;
}

double GET_EXTERNAL_PROBE_VALUE()
{
    // only for analog non-contact probe, so force a 0
    return 0.0;
}

// feed rate wanted is in program units per minute
double GET_EXTERNAL_FEED_RATE()
{
    double feed;

    if (canon.feed_mode) {
        // We're in G95 "Units per Revolution" mode, so linearFeedRate
        // is the FPR and we should just return it, unchanged.
        feed = canon.linearFeedRate;
    } else {
        // We're in G94 "Units per Minute" mode so unhork linearFeedRate
        // before returning it, by converting from internal to program
        // units, and from "per second" to "per minute".
        feed = TO_PROG_LEN(canon.linearFeedRate);
        feed *= 60.0;
    }

    return feed;
}

// traverse rate wanted is in program units per minute
double GET_EXTERNAL_TRAVERSE_RATE()
{
    double traverse;

    // convert from external to program units
    traverse =
	TO_PROG_LEN(FROM_EXT_LEN(emcStatus->motion.traj.maxVelocity));

    // now convert from per-sec to per-minute
    traverse *= 60.0;

    return traverse;
}

double GET_EXTERNAL_LENGTH_UNITS(void)
{
    double u;

    u = emcStatus->motion.traj.linearUnits;

    if (u == 0) {
	CANON_ERROR("external length units are zero");
	return 1.0;
    } else {
	return u;
    }
}

double GET_EXTERNAL_ANGLE_UNITS(void)
{
    double u;

    u = emcStatus->motion.traj.angularUnits;

    if (u == 0) {
	CANON_ERROR("external angle units are zero");
	return 1.0;
    } else {
	return u;
    }
}

int GET_EXTERNAL_MIST()
{
    return emcStatus->io.coolant.mist;
}

int GET_EXTERNAL_FLOOD()
{
    return emcStatus->io.coolant.flood;
}

double GET_EXTERNAL_SPEED(int spindle)
{
    // speed is in RPMs everywhere
    return canon.spindle[spindle].speed;
}

CANON_DIRECTION GET_EXTERNAL_SPINDLE(int spindle)
{
    if (emcStatus->motion.spindle[spindle].speed == 0) {
	return CANON_STOPPED;
    }

    if (emcStatus->motion.spindle[spindle].speed >= 0.0) {
	return CANON_CLOCKWISE;
    }

    return CANON_COUNTERCLOCKWISE;
}

static char _parameter_file_name[LINELEN];

void SET_PARAMETER_FILE_NAME(const char *name)
{
  strncpy(_parameter_file_name, name, PARAMETER_FILE_NAME_LENGTH);
}

void GET_EXTERNAL_PARAMETER_FILE_NAME(char *file_name,	/* string: to copy
							   file name into */
				      int max_size)
{				/* maximum number of characters to copy */
    // Paranoid checks
    if (0 == file_name)
	return;

    if (max_size < 0)
	return;

    if (strlen(_parameter_file_name) < ((size_t) max_size))
	strcpy(file_name, _parameter_file_name);
    else
	file_name[0] = 0;
}

double GET_EXTERNAL_POSITION_X(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.x;
}

double GET_EXTERNAL_POSITION_Y(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.y;
}

double GET_EXTERNAL_POSITION_Z(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.z;
}

double GET_EXTERNAL_POSITION_A(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.a;
}

double GET_EXTERNAL_POSITION_B(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.b;
}

double GET_EXTERNAL_POSITION_C(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.c;
}

double GET_EXTERNAL_POSITION_U(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.u;
}

double GET_EXTERNAL_POSITION_V(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.v;
}

double GET_EXTERNAL_POSITION_W(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_POSITION();
    return position.w;
}

double GET_EXTERNAL_PROBE_POSITION_X(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.x;
}

double GET_EXTERNAL_PROBE_POSITION_Y(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.y;
}

double GET_EXTERNAL_PROBE_POSITION_Z(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.z;
}

double GET_EXTERNAL_PROBE_POSITION_A(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.a;
}

double GET_EXTERNAL_PROBE_POSITION_B(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.b;
}

double GET_EXTERNAL_PROBE_POSITION_C(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.c;
}

double GET_EXTERNAL_PROBE_POSITION_U(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.u;
}

double GET_EXTERNAL_PROBE_POSITION_V(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.v;
}

double GET_EXTERNAL_PROBE_POSITION_W(void)
{
    CANON_POSITION position;
    position = GET_EXTERNAL_PROBE_POSITION();
    return position.w;
}

CANON_MOTION_MODE GET_EXTERNAL_MOTION_CONTROL_MODE()
{
    return canon.motionMode;
}

double GET_EXTERNAL_MOTION_CONTROL_TOLERANCE()
{
    return TO_PROG_LEN(canon.motionTolerance);
}

double GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE()
{
    return TO_PROG_LEN(canon.naivecamTolerance);
}


CANON_UNITS GET_EXTERNAL_LENGTH_UNIT_TYPE()
{
    return canon.lengthUnits;
}

int GET_EXTERNAL_QUEUE_EMPTY(void)
{
    flush_segments();

    return emcStatus->motion.traj.queue == 0 ? 1 : 0;
}

// Returns the "home pocket" of the tool currently in the spindle, ie the
// pocket that the current tool was loaded from.  Returns 0 if there is no
// tool in the spindle.

int GET_EXTERNAL_TOOL_SLOT()
{
    int toolno = emcStatus->io.tool.toolInSpindle;

    return tooldata_find_index_for_tool(toolno);
}

// If the tool changer has prepped a pocket (after a Txxx command) and is
// ready to perform a tool change, return the currently prepped pocket
// number.  If the tool changer is idle (because no Txxx command has been
// run, or because an M6 tool change has completed), return -1.
int GET_EXTERNAL_SELECTED_TOOL_SLOT()
{
    return emcStatus->io.tool.pocketPrepped; //idx
}

int GET_EXTERNAL_TC_FAULT()
{
    return emcStatus->io.fault;
}

int GET_EXTERNAL_TC_REASON()
{
    return emcStatus->io.reason;
}

int GET_EXTERNAL_FEED_OVERRIDE_ENABLE()
{
    return emcStatus->motion.traj.feed_override_enabled;
}

int GET_EXTERNAL_SPINDLE_OVERRIDE_ENABLE(int spindle)
{
    return emcStatus->motion.spindle[spindle].spindle_override_enabled;
}

int GET_EXTERNAL_ADAPTIVE_FEED_ENABLE()
{
    return emcStatus->motion.traj.adaptive_feed_enabled;
}

int GET_EXTERNAL_FEED_HOLD_ENABLE()
{
    return emcStatus->motion.traj.feed_hold_enabled;
}

int GET_EXTERNAL_AXIS_MASK() {
    return emcStatus->motion.traj.axis_mask;
}

int GET_EXTERNAL_OFFSET_APPLIED(void) {
    return emcGetExternalOffsetApplied();
}

EmcPose GET_EXTERNAL_OFFSETS() {
    return emcGetExternalOffsets();
}

CANON_PLANE GET_EXTERNAL_PLANE()
{
    return canon.activePlane;
}

/* returns current value of the digital input selected by index.*/
int GET_EXTERNAL_DIGITAL_INPUT(int index, int /*def*/)
{
    if ((index < 0) || (index >= EMCMOT_MAX_DIO))
	return -1;

    if (emcStatus->task.input_timeout == 1)
	return -1;

#ifdef INPUT_DEBUG
    printf("GET_EXTERNAL_DIGITAL_INPUT called\n di[%d]=%d \n timeout=%d \n",index,emcStatus->motion.synch_di[index],emcStatus->task.input_timeout);
#endif
    return (emcStatus->motion.synch_di[index] != 0) ? 1 : 0;
}

double GET_EXTERNAL_ANALOG_INPUT(int index, double /*def*/)
{
/* returns current value of the analog input selected by index.*/
#ifdef INPUT_DEBUG
    printf("GET_EXTERNAL_ANALOG_INPUT called\n ai[%d]=%g \n timeout=%d \n",index,emcStatus->motion.analog_input[index],emcStatus->task.input_timeout);
#endif
    if ((index < 0) || (index >= EMCMOT_MAX_AIO))
	return -1;

    if (emcStatus->task.input_timeout == 1)
	return -1;

    return emcStatus->motion.analog_input[index];
}


USER_DEFINED_FUNCTION_TYPE USER_DEFINED_FUNCTION[USER_DEFINED_FUNCTION_NUM]
    = { 0 };

int USER_DEFINED_FUNCTION_ADD(USER_DEFINED_FUNCTION_TYPE func, int num)
{
    if (num < 0 || num >= USER_DEFINED_FUNCTION_NUM) {
	return -1;
    }

    USER_DEFINED_FUNCTION[num] = func;

    return 0;
}

void MOTION_OUTPUT_BIT_(int index, int start, int end, int now)
{
    auto dout_msg = std::make_unique<EMC_MOTION_SET_DOUT>();

    flush_segments();

    dout_msg->index = index;
    dout_msg->start = start;
    dout_msg->end = end;
    dout_msg->now = now;

    interp_list.append(std::move(dout_msg));

    return;
}

/*! \function SET_MOTION_OUTPUT_BIT

  sets a DIO pin
  this message goes to task, then to motion which sets the DIO 
  when the first motion starts.
  The pin gets set with value 1 at the begin of motion, and stays 1 at the end of motion
  (this behaviour can be changed if needed)
  
  warning: setting more then one for a motion segment will clear out the previous ones 
  (the TP doesn't implement a queue of these), 
  use SET_AUX_OUTPUT_BIT instead, that allows to set the value right away
*/
void SET_MOTION_OUTPUT_BIT(int index)
{
    MOTION_OUTPUT_BIT_(index, 
                       1,       // startvalue = 1
                       1,       // endvalue = 1, means it doesn't get reset after current motion
                       0);      // not immediate, but synched with motion (goes to the TP)
}

/*! \function CLEAR_MOTION_OUTPUT_BIT

  clears a DIO pin
  this message goes to task, then to motion which clears the DIO 
  when the first motion starts.
  The pin gets set with value 0 at the begin of motion, and stays 0 at the end of motion
  (this behaviour can be changed if needed)
  
  warning: setting more then one for a motion segment will clear out the previous ones 
  (the TP doesn't implement a queue of these), 
  use CLEAR_AUX_OUTPUT_BIT instead, that allows to set the value right away
*/
void CLEAR_MOTION_OUTPUT_BIT(int index)
{
    MOTION_OUTPUT_BIT_(index,
                       0,       // startvalue = 1
                       0,       // endvalue = 0, means it stays 0 after current motion
                       0);      // not immediate, but synched with motion (goes to the TP)
}

/*! \function SET_AUX_OUTPUT_BIT

  sets a DIO pin
  this message goes to task, then to motion which sets the DIO 
  right away.
  The pin gets set with value 1 at the begin of motion, and stays 1 at the end of motion
  (this behaviour can be changed if needed)
  you can use any number of these, as the effect is immediate  
*/
void SET_AUX_OUTPUT_BIT(int index)
{
    MOTION_OUTPUT_BIT_(index,
                       1,       // startvalue = 1
                       1,       // endvalue = 1, means it doesn't get reset after current motion
                       1);      // immediate, we don't care about syncing for AUX
}

/*! \function CLEAR_AUX_OUTPUT_BIT

  clears a DIO pin
  this message goes to task, then to motion which clears the DIO 
  right away.
  The pin gets set with value 0 at the begin of motion, and stays 0 at the end of motion
  (this behaviour can be changed if needed)
  you can use any number of these, as the effect is immediate  
*/
void CLEAR_AUX_OUTPUT_BIT(int index)
{
    MOTION_OUTPUT_BIT_(index,
                       0,       // startvalue = 1
                       0,       // endvalue = 0, means it stays 0 after current motion
                       1);      // immediate, we don't care about syncing for AUX
}

void MOTION_OUTPUT_VALUE_(int index, double start, double end, int now)
{
    auto aout_msg = std::make_unique<EMC_MOTION_SET_AOUT>();

    flush_segments();

    aout_msg->index = index;
    aout_msg->start = start;
    aout_msg->end = end;
    aout_msg->now = now;

    interp_list.append(std::move(aout_msg));
}

/*! \function SET_MOTION_OUTPUT_VALUE

  sets a AIO value, not used by the RS274 Interp,
  not fully implemented in the motion controller either
*/
void SET_MOTION_OUTPUT_VALUE(int index, double value)
{
    MOTION_OUTPUT_VALUE_(index,     // which output
                         value,     // start value
                         value,     // end value
                        0);         // immediate=1, or synched when motion start=0
}

/*! \function SET_AUX_OUTPUT_VALUE

  sets a AIO value, not used by the RS274 Interp,
  not fully implemented in the motion controller either
*/
void SET_AUX_OUTPUT_VALUE(int index, double value)
{
    MOTION_OUTPUT_VALUE_(index,     // which output
                         value,     // start value
                         value,	    // end value
                         1);        // immediate=1, or synched when motion start=0
}

/*! \function WAIT
   program execution and interpreting is stopped until the input selected by 
   index changed to the needed state (specified by wait_type).
   Return value: either wait_type if timeout didn't occur, or -1 otherwise. */

int WAIT(int index, /* index of the motion exported input */
         int input_type, /*DIGITAL_INPUT or ANALOG_INPUT */
	 int wait_type,  /* 0 - immediate, 1 - rise, 2 - fall, 3 - be high, 4 - be low */
	 double timeout) /* time to wait [in seconds], if the input didn't change the value -1 is returned */
{
    if (input_type == DIGITAL_INPUT) {
        if ((index < 0) || (index >= EMCMOT_MAX_DIO))
	    return -1;
    } else if (input_type == ANALOG_INPUT) {
        if ((index < 0) || (index >= EMCMOT_MAX_AIO))
	    return -1;
    }

    auto wait_msg = std::make_unique<EMC_AUX_INPUT_WAIT>();
 
    flush_segments();
 
    wait_msg->index = index;
    wait_msg->input_type = input_type;
    wait_msg->wait_type = wait_type;
    wait_msg->timeout = timeout;
 
    interp_list.append(std::move(wait_msg));
    return 0;
}

int UNLOCK_ROTARY(int line_number, int joint_num)
{
    auto m = std::make_unique<EMC_TRAJ_LINEAR_MOVE>();

    // first, set up a zero length move to interrupt blending and get to final position
    m->type = EMC_MOTION_TYPE_TRAVERSE;
    m->feed_mode = 0;
    m->end = to_ext_pose(canon.endPoint.x, canon.endPoint.y, canon.endPoint.z,
                         canon.endPoint.a, canon.endPoint.b, canon.endPoint.c,
                         canon.endPoint.u, canon.endPoint.v, canon.endPoint.w);
    m->vel = m->acc = 1; // nonzero but otherwise doesn't matter
    m->indexer_jnum = -1;

    // issue it
    int old_feed_mode = canon.feed_mode;
    if(canon.feed_mode)
	STOP_SPEED_FEED_SYNCH();
    interp_list.set_line_number(line_number);
    interp_list.append(std::move(m));
    
    // no need to update endpoint
    if(old_feed_mode)
	START_SPEED_FEED_SYNCH(canon.spindle_num, canon.linearFeedRate, 1);

    // now, the next move is the real indexing move, so be ready
    canon.rotary_unlock_for_traverse = joint_num;
    return 0;
}

int LOCK_ROTARY(int /*line_number*/, int /*joint_num*/) {
    canon.rotary_unlock_for_traverse = -1;
    return 0;
}

