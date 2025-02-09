/********************************************************************
* Description: canon.hh
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef CANON_HH
#define CANON_HH

#include <stdio.h>		// FILE
#include <vector>

#include "emcpos.h"
#include "emctool.h"
#include "canon_position.hh"
#include "emcmotcfg.h" // Just for EMCMOT_NUM_SPINDLES
#include "modal_state.hh"

/*
  This is the header file that all applications that use the
  canonical commands for three- to nine-axis machining should include.

  Three mutually orthogonal (in a right-handed system) X, Y, and Z axes
  are always present. In addition, there may be zero to three rotational
  axes: A (parallel to the X-axis), B (parallel to the Y-axis), and C
  (parallel to the Z-axis). Additionally there may be zero to three linear
  axes: U, V and W.

  In the functions that use rotational axes, the axis value is that of a
  wrapped linear axis, in degrees.

  It is assumed in these activities that the spindle tip is always at
  some location called the 'current location,' and the controller always
  knows where that is. It is also assumed that there is always a
  'selected plane' which must be the XY-plane, the YZ-plane, or the
  ZX-plane of the machine.
*/

enum CanonBool {
	OFF,
	ON
};

struct NURBS_PLANE_POINT {
	double NURBS_X,
	NURBS_Y;
};

struct NURBS_CONTROL_POINT {  // type for NURBS control points 
        double NURBS_X,      		// in questa struttura vengono dichiarati i seguenti elementi: coordinate X, Y dei punti di controllo           
               NURBS_Y,      		// Per l'algoritmo DE-Boor
               NURBS_W;					// Algoritmo di suddivisione
};

struct NURBS_G6_CONTROL_POINT {           /* type for NURBS G6 control points */
	double	NURBS_X,
		NURBS_Y,
		NURBS_R,	// this is the R value from gcode
		NURBS_K;
};

struct NURBS_G6_DPLANE_POINT {
        double DX,
	DY;
};	//è impiegata per salvare le derivate x'(u) y'(u).


enum class CANON_PLANE {
    XY = 1,
    YZ,
    XZ,
    UV,
    VW,
    UW,
};

enum CANON_UNITS
{
    CANON_UNITS_INCHES = 1,
    CANON_UNITS_MM,
    CANON_UNITS_CM,
};

enum CANON_MOTION_MODE
{
    CANON_EXACT_STOP = 1,
    CANON_EXACT_PATH,
    CANON_CONTINUOUS,
};

enum CANON_SPEED_FEED_MODE {
    CANON_SYNCHED = 1,
    CANON_INDEPENDENT,
};

enum CANON_DIRECTION {
    CANON_STOPPED = 1,
    CANON_CLOCKWISE,
    CANON_COUNTERCLOCKWISE,
};

enum CANON_FEED_REFERENCE {
    CANON_WORKPIECE = 1,
    CANON_XYZ,
};

enum CANON_SIDE
{
    CANON_SIDE_RIGHT = 1,
    CANON_SIDE_LEFT,
    CANON_SIDE_OFF,
};

enum CANON_AXIS
{
    CANON_AXIS_X = 1,
    CANON_AXIS_Y,
    CANON_AXIS_Z,
    CANON_AXIS_A,
    CANON_AXIS_B,
    CANON_AXIS_C,
    CANON_AXIS_U,
    CANON_AXIS_V,
    CANON_AXIS_W,
};

struct CANON_VECTOR {
    CANON_VECTOR() {
    } CANON_VECTOR(double _x, double _y, double _z) {
	x = _x;
	y = _y;
	z = _z;
    }
    double x, y, z;
};

typedef struct {
    int feed_mode;
    int synched;
    double speed;
    int dir;
    double css_maximum;
    double css_factor;
} CanonSpindle_t;

typedef struct CanonConfig_t {
    CanonConfig_t()
        : xy_rotation(0.0),
          rotary_unlock_for_traverse(-1),
          g5xOffset{},
          g92Offset{},
          endPoint{},
          lengthUnits(CANON_UNITS_INCHES),
          activePlane(CANON_PLANE::XY),
          toolOffset{},
          motionMode(CANON_EXACT_STOP),
          motionTolerance(0.0),
          naivecamTolerance(0.0),
          feed_mode(0),
          spindle_num(0),
          spindle{},
          linearFeedRate(0.0),
          angularFeedRate(0.0),
          optional_program_stop(false),
          block_delete(false),
          cartesian_move(0),
          angular_move(0)
    {}

    double xy_rotation;
    int rotary_unlock_for_traverse; // jointnumber or -1

    CANON_POSITION g5xOffset;
    CANON_POSITION g92Offset;
/*
  canonEndPoint is the last programmed end point, stored in case it's
  needed for subsequent calculations. It's in absolute frame, mm units.

  note that when segments are queued for the naive cam detector that the
  canonEndPoint may not be the last programmed endpoint.  get_last_pos()
  retrieves the xyz position after the last of the queued segments.  these
  are also in absolute frame, mm units.
  */
    CANON_POSITION endPoint;
    CANON_UNITS lengthUnits;
    CANON_PLANE activePlane;
/* Tool length offset is saved here */
    EmcPose toolOffset;
/* motion control mode is used to signify blended v. stop-at-end moves.
   Set to 0 (invalid) at start, so first call will send command out */
    CANON_MOTION_MODE motionMode;
/* motion path-following tolerance is used to set the max path-following
   deviation during CANON_CONTINUOUS.
   If this param is 0, then it will behave as emc always did, allowing
   almost any deviation trying to keep speed up. */
   double motionTolerance;
   double naivecamTolerance;
   int feed_mode;
   int spindle_num; //current spindle for spindle-synch motion
   CanonSpindle_t spindle[EMCMOT_MAX_SPINDLES];

/* Prepped tool is saved here */
//   int preppedTool;
/*
  Feed rate is saved here; values are in mm/sec or deg/sec.
  It will be initially set in INIT_CANON() below.
*/
    double linearFeedRate;
    double angularFeedRate;
/* optional program stop */
    bool optional_program_stop;
/* optional block delete */
    bool block_delete;
/* Used to indicate whether the current move is linear, angular, or 
   a combination of both. */
   //AJ says: linear means axes XYZ move (lines or even circles)
   //         angular means axes ABC move
    int cartesian_move;
    int angular_move;
} CanonConfig_t;

/* Initialization */

/* reads world model data into the canonical interface */
extern void INIT_CANON();

/* Representation */

extern void SET_G5X_OFFSET(int origin,
                           double x, double y, double z,
                           double a, double b, double c,
                           double u, double v, double w);

extern void SET_G92_OFFSET(double x, double y, double z,
                           double a, double b, double c,
                           double u, double v, double w);

extern void SET_XY_ROTATION(double t);

/* Offset the origin to the point with absolute coordinates x, y, z,
a, b, c, u, v, and w. Values of x, y, z, a, b, c, u, v, and w are real 
numbers. The units are whatever length units are being used at the time 
this command is given. */

extern void CANON_UPDATE_END_POINT(double x, double y, double z, 
				   double a, double b, double c,
				   double u, double v, double w);
/* Called from emctask to update the canon position during skipping through
   programs started with start-from-line > 0. */

extern void USE_LENGTH_UNITS(CANON_UNITS u);

/* Use the specified units for length. Conceptually, the units must
be either inches or millimeters. */

extern void SELECT_PLANE(CANON_PLANE pl);

/* Use the plane designated by selected_plane as the selected plane.
Conceptually, the selected_plane must be the XY-plane, the XZ-plane, or
the YZ-plane. */

/* Free Space Motion */

extern void SET_TRAVERSE_RATE(double rate);

/* Set the traverse rate that will be used when the spindle traverses. It
is expected that no cutting will occur while a traverse move is being
made. */

extern void STRAIGHT_TRAVERSE(int lineno,
                              double x, double y, double z,
                              double a, double b, double c,
                              double u, double v, double w);
/*

Move at traverse rate so that at any time during the move, all axes
have covered the same proportion of their required motion. The final
XYZ position is given by x, y, and z. If there is an a-axis, its final
position is given by a_position, and similarly for the b-axis and c-axis.
A more positive value of a rotational axis is in the counterclockwise
direction.

Clockwise or counterclockwise is from the point of view of the
workpiece. If the workpiece is fastened to a turntable, the turntable
will turn clockwise (from the point of view of the machinist or anyone
else not moving with respect to the machining center) in order to make
the tool move counterclockwise from the point of view of the
workpiece.

*/

/* Machining Attributes */

extern void SET_FEED_RATE(double rate);

/*

SET_FEED_RATE sets the feed rate that will be used when the spindle is
told to move at the currently set feed rate. The rate is either:
1. the rate of motion of the tool tip in the workpiece coordinate system,
   which is used when the feed_reference mode is "CANON_WORKPIECE", or
2. the rate of motion of the tool tip in the XYZ axis system, ignoring
   motion of other axes, which is used when the feed_reference mode is
   "CANON_XYZ".

The units of the rate are:

1. If the feed_reference mode is CANON_WORKPIECE:
length units (inches or millimeters according to the setting of
CANON_UNITS) per minute along the programmed path as seen by the
workpiece.

2. If the feed_reference mode is CANON_XYZ:
A. For motion including one rotational axis only: degrees per minute.
B. For motion of two or three rotational axes with X, Y, Z, U, V, and W 
   axes not moving, the rate is applied as follows. Let dA, dB, and dC 
   be the angles in degrees through which the A, B, and C axes, 
   respectively, must move. Let D = sqrt(dA*dA + dB*dB + dC*dC). 
   Conceptually, D is a measure of total angular motion, using the usual 
   Euclidean metric. Let T be the amount of time required to move through 
   D degrees at the current feed rate in degrees per minute. The 
   rotational axes should be moved in coordinated linear motion so that 
   the elapsed time from the start to the end of the motion is T plus any 
   time required for acceleration or deceleration.
C. For motion of secondary linear axes (U, V, and/or W) with X, Y, and Z 
   axes not moving (with or without simultaneous rotational axis motion): 
   length units (inches or millimeters according to the setting of 
   CANON_UNITS) per minute in the UVW cartesian system.
D. For motion involving one or more of the XYZ axes (with or without
   simultaneous motion of other axes): length units (inches or
   millimeters according to the setting of CANON_UNITS) per minute
   along the programmed XYZ path.

*/

extern void SET_FEED_REFERENCE(CANON_FEED_REFERENCE reference);

/*

This sets the feed_reference mode to either CANON_WORKPIECE or
CANON_XYZ.

The CANON_WORKPIECE mode is more natural and general, since the rate
at which the tool passes through the material must be controlled for
safe and effective machining. For machines with more than the three
standard XYZ axes, however, computing the feed rate may be
time-consuming because the trajectories that result from motion in
four or more axes may be complex. Computation of path lengths when
only XYZ motion is considered is quite simple for the two standard
motion types (straight lines and helical arcs).

Some programming languages (rs274kt, in particular) use CANON_XYZ
mode. In these languages, the task of dealing with the rate at which
the tool tip passes through material is pushed back on the NC-program
generator, where the computation of path lengths is (almost always in
1995) an off-line activity where speed of calculation is not critical.

In CANON_WORKPIECE mode, some motions cannot be carried out as fast as
the programmed feed rate would require because axis motions tend to
cancel each other. For example, an arc in the YZ-plane can exactly
cancel a rotation around the A-axis, so that the location of the tool
tip with respect to the workpiece does not change at all during the
motion; in this case, the motion should take no time, which is
impossible at any finite rate of axis motion. In such cases, the axes
should be moved as fast as possible consistent with accurate
machining.

It would be possible to omit the SET_FEED_REFERENCE command from the
canonical commands and operate always in one mode or the other,
letting the interpreter issue SET_FEED_RATE commands, if necessary to
compensate if the NC language being interpreted used the other mode.

This would create two disadvantages when the feed_reference mode
assumed by the canonical commands differed from that assumed by the NC
language being interpreted:

1. The output code could have a lot of SET_FEED_RATE commands not
found in the input code; this is a relatively minor consideration.

2. If the interpreter reads a program in language which uses the
CANON_XYZ mode and writes canonical commands in the CANON_WORKPIECE
mode, both the interpreter and the executor of the output canonical
commands would have to perform a lot of complex calculations. With the
SET_FEED_REFERENCE command available, both do only simple calculations
for the same motions.

*/

extern void SET_FEED_MODE(int spindle, int mode);

/* This sets the feed mode: 0 for feed in units per minute, and 1 for feed in
 * units per revolution.  In units per revolution mode, the values are in
 * inches per revolution (G20 in effect) or mm per minute (G21 in effect)
 * The spindle number indicates which spindle the movement is synchronised to */

extern void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode, double tolerance);

extern void SET_NAIVECAM_TOLERANCE(double tolerance);

/*

This sets the motion control mode to one of: CANON_EXACT_STOP,
CANON_EXACT_PATH, or CANON_CONTINUOUS.

For CANON_CONTINUOUS another parameter defines the maximum path deviation. 
If tolerance=0 then any path deviation may occur, speed is maximized.

*/

extern void SET_CUTTER_RADIUS_COMPENSATION(double radius);

/* Set the radius to use when performing cutter radius compensation. */

extern void START_CUTTER_RADIUS_COMPENSATION(int direction);

/* Conceptually, the direction must be left (meaning the cutter
stays to the left of the programmed path) or right. */

extern void STOP_CUTTER_RADIUS_COMPENSATION();

/* Do not apply cutter radius compensation when executing spindle
translation commands. */

/* used for threading */
extern void START_SPEED_FEED_SYNCH(int spindle, double feed_per_revolution, bool velocity_mode);
extern void STOP_SPEED_FEED_SYNCH();


/* Machining Functions */

extern void ARC_FEED(int lineno,
                     double first_end, double second_end,
		     double first_axis, double second_axis, int rotation,
		     double axis_end_point, 
                     double a, double b, double c,
                     double u, double v, double w);

/* Move in a helical arc from the current location at the existing feed
rate. The axis of the helix is parallel to the x, y, or z axis,
according to which one is perpendicular to the selected plane. The
helical arc may degenerate to a circular arc if there is no motion
parallel to the axis of the helix.

1. If the selected plane is the xy-plane:
A. first_end is the x-coordinate of the end of the arc.
B. second_end is the y-coordinate of the end of the arc.
C. first_axis is the x-coordinate of the axis (center) of the arc.
D. second_axis is the y-coordinate of the axis.
E. axis_end_point is the z-coordinate of the end of the arc.

2. If the selected plane is the yz-plane:
A. first_end is the y-coordinate of the end of the arc.
B. second_end is the z-coordinate of the end of the arc.
C. first_axis is the y-coordinate of the axis (center) of the arc.
D. second_axis is the z-coordinate of the axis.
E. axis_end_point is the x-coordinate of the end of the arc.

3. If the selected plane is the zx-plane:
A. first_end is the z-coordinate of the end of the arc.
B. second_end is the x-coordinate of the end of the arc.
C. first_axis is the z-coordinate of the axis (center) of the arc.
D. second_axis is the x-coordinate of the axis.
E. axis_end_point is the y-coordinate of the end of the arc.

If rotation is positive, the arc is traversed counterclockwise as
viewed from the positive end of the coordinate axis perpendicular to
the currently selected plane. If rotation is negative, the arc is
traversed clockwise. If rotation is 0, first_end and second_end must
be the same as the corresponding coordinates of the current point and
no arc is made (but there may be translation parallel to the axis
perpendicular to the selected plane and motion along the rotational axes).
If rotation is 1, more than 0 but not more than 360 degrees of arc
should be made. In general, if rotation is n, the amount of rotation
in the arc should be more than ([n-1] x 360) but not more than (n x
360).

The radius of the helix is determined by the distance from the current
location to the axis of helix or by the distance from the end location
to the axis of the helix. It is recommended that the executing system
verify that the two radii are the same (within some tolerance) at the
beginning of executing this function.

While the XYZ motion is going on, move the rotational axes so that
they have always covered the same proportion of their total motion as
a point moving along the arc has of its total motion.

*/

extern void STRAIGHT_FEED(int lineno,
                          double x, double y, double z,
                          double a, double b, double c,
                          double u, double v, double w);

/* Additional functions needed to calculate nurbs G5 points */

extern std::vector<unsigned int> nurbs_G5_knot_vector_creator(unsigned int n, unsigned int k);

extern double nurbs_G5_Nmix(unsigned int i, unsigned int k, double u, const std::vector<unsigned int>& knot_vector);

extern double nurbs_G5_Rden(double u, unsigned int k,
                  const std::vector<NURBS_CONTROL_POINT>& nurbs_control_points,
                  const std::vector<unsigned int>& knot_vector);

extern NURBS_PLANE_POINT nurbs_G5_point(double u, unsigned int k, 
                  const std::vector<NURBS_CONTROL_POINT>& nurbs_control_points,
                  const std::vector<unsigned int>& knot_vector);

extern NURBS_PLANE_POINT nurbs_G5_tangent(double u, unsigned int k,
                  const std::vector<NURBS_CONTROL_POINT>& nurbs_control_points,
                  const std::vector<unsigned int>& knot_vector);

/* Additional functions needed to calculate nurbs points G_6_2*/

extern std::vector<double> nurbs_g6_knot_vector_creator(unsigned int n, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points);

std::vector<double> nurbs_interval_span_knot_vector_creator(unsigned int n, unsigned int k, const std::vector<double>& knot_vector_);

extern double nurbs_lderv(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

extern double nurbs_Sa1_b1_length_(double a1, double b1, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

extern std::vector<double> nurbs_lenght_vector_creator(unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_, const std::vector<double>& span_knot_vector);

extern double nurbs_lenght_tot(int j, const std::vector<double>& span_knot_vector, const std::vector<double>& lenght_vector);

extern double nurbs_lenght_l_u(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_, const std::vector<double>& span_knot_vector, const std::vector<double>& lenght_vector);

extern std::vector<double> nurbs_Du_span_knot_vector_creator(unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_, const std::vector<double>& span_knot_vector);

extern std::vector<double> nurbs_costant_crator(const std::vector<double>& span_knot_vector, const std::vector<double>& lenght_vector, const std::vector<double>& Du_span_knot_vector);

extern double nurbs_uj_l(double l, const std::vector<double>& span_knot_vector, const std::vector<double>& lenght_vector, const std::vector<double>& nurbs_costant);

/* Funzioni di supporto all'algoritmo di suddivisione */
extern  std::vector<double> nurbs_G6_new_control_point_nurbs1(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_ );

extern std::vector<double> nurbs_G6_new_control_point_nurbs2(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_ );

extern std::vector<double> nurbs_G6_knot_vector_new_creator_sgment(unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points);
/* ... */

extern double nurbs_G6_Nmix(unsigned int i, unsigned int k, double u, const std::vector<double>& knot_vector_);

extern std::vector< std::vector<double> > nurbs_G6_Nmix_creator(double u, unsigned int k, double n, const std::vector<double>& knot_vector_);

extern double nurbs_G6_Rden(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

extern NURBS_PLANE_POINT nurbs_G6_point(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

extern double nurbs_G6_Rdenx(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_, const std::vector< std::vector<double> >& A6);

extern NURBS_PLANE_POINT nurbs_G6_pointx(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_, const std::vector< std::vector<double> >& A6);

extern NURBS_PLANE_POINT nurbs_G6_point_x(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

extern NURBS_PLANE_POINT nurbs_G6_tangent_x(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_);

/*Funzioni di supporto all'algoritmo di suddivisione********************************************************************/
extern  std::vector<double> nurbs_G6_new_control_point_nurbs1(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_ );

extern std::vector<double> nurbs_G6_new_control_point_nurbs2(double u, unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, const std::vector<double>& knot_vector_ );

extern std::vector<double> nurbs_G6_knot_vector_new_creator_sgment(unsigned int k, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points);

/* End of NURBS functions*/


/* Canon calls */

extern void NURBS_G5_FEED(int lineno, const std::vector<NURBS_CONTROL_POINT>& nurbs_control_points, unsigned int nurbs_order, CANON_PLANE plane);
/* Move at the feed rate along an approximation of a NURBS with a variable number
 * of control points
 */

extern void NURBS_G6_FEED(int lineno, const std::vector<NURBS_G6_CONTROL_POINT>& nurbs_control_points, unsigned int k, double feedrate, int l, CANON_PLANE plane);
// this is for G6xx

extern double alpha_finder(double dx, double dy);


/********************************************************************************************************************/

/* Move at existing feed rate so that at any time during the move,
all axes have covered the same proportion of their required motion.
The meanings of the parameters is the same as for STRAIGHT_TRAVERSE.*/

extern void RIGID_TAP(int lineno,
                      double x, double y, double z, double scale);

/* Move linear and synced with the previously set pitch.
Only linear moves are allowed, axes A,B,C are not allowed to move.*/


extern void STRAIGHT_PROBE(int lineno,
                           double x, double y, double z,
                           double a, double b, double c,
                           double u, double v, double w, unsigned char probe_type);

/* Perform a probing operation. This is a temporary addition to the
canonical machining functions and its semantics are not defined.
When the operation is finished, all axes should be back where they
started. */

extern void STOP();

/* stop motion after current feed */

extern void DWELL(double seconds);

/* freeze x,y,z for a time */

/* Spindle Functions */

extern void SET_SPINDLE_MODE(int spindle, double mode);
extern void SPINDLE_RETRACT_TRAVERSE();

/* Retract the spindle at traverse rate to the fully retracted position. */

extern void START_SPINDLE_CLOCKWISE(int spindle, int wait_for_atspeed = 1);

/* Turn the spindle clockwise at the currently set speed rate. If the
spindle is already turning that way, this command has no effect. */

extern void START_SPINDLE_COUNTERCLOCKWISE(int spindle, int wait_for_atspeed = 1);

/* Turn the spindle counterclockwise at the currently set speed rate. If
the spindle is already turning that way, this command has no effect. */

extern void SET_SPINDLE_SPEED(int spindle, double r);

/* Set the spindle speed that will be used when the spindle is turning.
This is usually given in rpm and refers to the rate of spindle
rotation. If the spindle is already turning and is at a different
speed, change to the speed given with this command. */

extern void STOP_SPINDLE_TURNING(int spindle);

/* Stop the spindle from turning. If the spindle is already stopped, this
command may be given, but it will have no effect. */

extern void SPINDLE_RETRACT();
extern void ORIENT_SPINDLE(int spindle, double orientation, int mode);
extern void WAIT_SPINDLE_ORIENT_COMPLETE(int spindle, double timeout);
extern void LOCK_SPINDLE_Z();
extern void USE_SPINDLE_FORCE();
extern void USE_NO_SPINDLE_FORCE();

/* Tool Functions */
extern void SET_TOOL_TABLE_ENTRY(int pocket, int toolno, const EmcPose& offset, double diameter,
                                 double frontangle, double backangle, int orientation);
extern void USE_TOOL_LENGTH_OFFSET(const EmcPose& offset);

extern void CHANGE_TOOL();

/* It is assumed that each cutting tool in the machine is assigned to a
slot (intended to correspond to a slot number in a tool carousel).
This command results in the tool currently in the spindle (if any)
being returned to its slot, and the tool from the slot designated by
slot_number (if any) being inserted in the spindle.

If there is no tool in the slot designated by the slot argument, there
will be no tool in the spindle after this command is executed and no
error condition will result in the controller. Similarly, if there is
no tool in the spindle when this command is given, no tool will be
returned to the carousel and no error condition will result in the
controller, whether or not a tool was previously selected in the
program.

It is expected that when the machine tool controller is initialized,
the designated slot for a tool already in the spindle will be
established. This may be done in any manner deemed fit, including
(for, example) recording that information in a persistent, crash-proof
location so it is always available from the last time the machine was
run, or having the operator enter it. It is expected that the machine
tool controller will remember that information as long as it is
not re-initialized; in particular, it will be remembered between
programs.

For the purposes of this command, the tool includes the tool holder.

For machines which can carry out a select_tool command separately from
a change_tool command, the select_tool command must have been given
before the change_tool command, and the value of slot must be the slot
number of the selected tool. */

extern void SELECT_TOOL(int tool);

extern void CHANGE_TOOL_NUMBER(int number);
extern void RELOAD_TOOLDATA(void);

/* In extension to the comment above - for CHANGE_TOOL, sometimes on 
startup one would want to tell emc2 what tool it has loaded. As the last
toolnumber before shutdown isn't currently written, there is no provision
to allow emc2 to safely restart knowing what tool is in the spindle.
Using CHANGE_TOOL_NUMBER one can tell emc2 (without any physical action)
to set the mapping of the currently loaded tool to a certain number */

/* Miscellaneous Functions */

extern void CLAMP_AXIS(CANON_AXIS axis);

/* Clamp the given axis. If the machining center does not have a clamp
for that axis, this command should result in an error condition in the
controller.

An attempt to move an axis while it is clamped should result in an
error condition in the controller. */

extern void COMMENT(const char *s);

/* This function has no physical effect. If commands are being printed or
logged, the comment command is printed or logged, including the string
which is the value of comment_text. This serves to allow formal
comments at specific locations in programs or command files. */

/* used for EDM adaptive moves with motion internal feed override (0..1) */
extern void DISABLE_ADAPTIVE_FEED();
extern void ENABLE_ADAPTIVE_FEED();

/* used to deactivate user control of feed override */
extern void DISABLE_FEED_OVERRIDE();
extern void ENABLE_FEED_OVERRIDE();

/* used to deactivate user control of spindle speed override */
extern void DISABLE_SPEED_OVERRIDE(int spindle);
extern void ENABLE_SPEED_OVERRIDE(int spindle);

/* used to deactivate user control of feed hold */
extern void DISABLE_FEED_HOLD();
extern void ENABLE_FEED_HOLD();


extern void FLOOD_OFF();
/* Turn flood coolant off. */
extern void FLOOD_ON();
/* Turn flood coolant on. */

extern void MESSAGE(char *s);

extern void LOG(char *s);
extern void LOGOPEN(char *s);
extern void LOGAPPEND(char *s);
extern void LOGCLOSE();

extern void MIST_OFF();
/* Turn mist coolant off. */

extern void MIST_ON();
/* Turn mist coolant on. */

extern void PALLET_SHUTTLE();

/* If the machining center has a pallet shuttle mechanism (a mechanism
which switches the position of two pallets), this command should cause
that switch to be made. If either or both of the pallets are missing,
this will not result in an error condition in the controller.

If the machining center does not have a pallet shuttle, this command
should result in an error condition in the controller. */

extern void TURN_PROBE_OFF();
extern void TURN_PROBE_ON();

extern void UNCLAMP_AXIS(CANON_AXIS axis);

/* Unclamp the given axis. If the machining center does not have a clamp
for that axis, this command should result in an error condition in the
controller. */

/* NURB Functions */
extern void NURB_KNOT_VECTOR();	/* double knot values, -1.0 signals done */
extern void NURB_CONTROL_POINT(int i, double x, double y, double z,
			       double w);
extern void NURB_FEED(double sStart, double sEnd);


/* Block delete */
extern void SET_BLOCK_DELETE(bool enabled);
/* Command to set the internal reference of block delete.
The ON value for enabled will cause the interpreter to discard lines
that start with the "/" character. */

extern bool GET_BLOCK_DELETE(void);
/* Command to get the internal reference of optional block delete. */


/* Program Functions */
extern void OPTIONAL_PROGRAM_STOP();
/* If the machining center has an optional stop switch, and it is on
when this command is read from a program, stop executing the program
at this point, but be prepared to resume with the next line of the
program. If the machining center does not have an optional stop
switch, or commands are being executed with a stop after each one
already (such as when the interpreter is being used with keyboard
input), this command has no effect. */

extern void SET_OPTIONAL_PROGRAM_STOP(bool state);
/* Command to set the internal reference of optional program stop.
Any non-zero value for state will cause the execution to stop on
optional stops. */

extern bool GET_OPTIONAL_PROGRAM_STOP();
/* Command to get the internal reference of optional program stop. */

extern void PROGRAM_END();
/* If a program is being read, stop executing the program and be prepared
to accept a new program or to be shut down. */

extern void PROGRAM_STOP();
/* If this command is read from a program, stop executing the program at
this point, but be prepared to resume with the next line of the
program. If commands are being executed with a stop after each one
already (such as when the interpreter is being used with keyboard
input), this command has no effect. */


/* Commands to set/reset output bits and analog values */
extern void SET_MOTION_OUTPUT_BIT(int index);
extern void CLEAR_MOTION_OUTPUT_BIT(int index);
extern void SET_AUX_OUTPUT_BIT(int index);
extern void CLEAR_AUX_OUTPUT_BIT(int index);

extern void SET_MOTION_OUTPUT_VALUE(int index, double value);
extern void SET_AUX_OUTPUT_VALUE(int index, double value);

/* Commands to wait for, query input bits and analog values */

#define DIGITAL_INPUT 1
#define ANALOG_INPUT 0

#define WAIT_MODE_IMMEDIATE	0
#define WAIT_MODE_RISE 		1
#define WAIT_MODE_FALL		2
#define WAIT_MODE_HIGH		3
#define WAIT_MODE_LOW		4

extern int WAIT(int index, /* index of the motion exported input */
		int input_type, /* 1=DIGITAL_INPUT or 0=ANALOG_INPUT */
	        int wait_type, /* 0 - immediate, 1 - rise, 2 - fall, 3 - be high, 4 - be low */
		double timeout); /* time to wait [in seconds], if the input didn't change the value -1 is returned */
/* WAIT - program execution is stopped until the input selected by index 
   changed to the needed state (specified by wait_type).
   Return value: either wait_type if timeout didn't occur, or -1 otherwise. */

/* tell canon the next move needs the rotary to be unlocked */
extern int UNLOCK_ROTARY(int line_no, int joint_num);

/* tell canon that is no longer the case */
extern int LOCK_ROTARY(int line_no, int joint_num);

/*************************************************************************/

/* Canonical "Give me information" functions for the interpreter to call

In general, returned values are valid only if any canonical do it commands
that may have been called for have been executed to completion. If a function
returns a valid value regardless of execution, that is noted in the comments
below.

*/

/* The interpreter is not using this function
// Returns the system angular unit factor, in units / degree
extern double GET_EXTERNAL_ANGLE_UNIT_FACTOR();
*/

// Returns the system feed rate
extern double GET_EXTERNAL_FEED_RATE();

// Returns the system value for flood coolant, zero = off, non-zero = on
extern int GET_EXTERNAL_FLOOD();

/* The interpreter is not using this function
// Returns the system length unit factor, in units / mm
extern double GET_EXTERNAL_LENGTH_UNIT_FACTOR();
*/

// Returns the system length unit type
CANON_UNITS GET_EXTERNAL_LENGTH_UNIT_TYPE();

extern double GET_EXTERNAL_LENGTH_UNITS();
extern double GET_EXTERNAL_ANGLE_UNITS();

// Returns the system value for mist coolant, zero = off, non-zero = on
extern int GET_EXTERNAL_MIST();

// Returns the current motion control mode
extern CANON_MOTION_MODE GET_EXTERNAL_MOTION_CONTROL_MODE();

// Returns the current motion path-following tolerance
extern double GET_EXTERNAL_MOTION_CONTROL_TOLERANCE();

// Returns the current motion naive CAM tolerance
extern double GET_EXTERNAL_MOTION_CONTROL_NAIVECAM_TOLERANCE();

/* The interpreter is not using these six GET_EXTERNAL_ORIGIN functions

// returns the current a-axis origin offset
extern double GET_EXTERNAL_ORIGIN_A();

// returns the current b-axis origin offset
extern double GET_EXTERNAL_ORIGIN_B();

// returns the current c-axis origin offset
extern double GET_EXTERNAL_ORIGIN_C();

// returns the current x-axis origin offset
extern double GET_EXTERNAL_ORIGIN_X();

// returns the current y-axis origin offset
extern double GET_EXTERNAL_ORIGIN_Y();

// returns the current z-axis origin offset
extern double GET_EXTERNAL_ORIGIN_Z();

*/

// returns nothing but copies the name of the parameter file into
// the filename array, stopping at max_size if the name is longer
// An empty string may be placed in filename.
extern void GET_EXTERNAL_PARAMETER_FILE_NAME(char *filename, int max_size);

extern void SET_PARAMETER_FILE_NAME(const char *name);

// returns the currently active plane
extern CANON_PLANE GET_EXTERNAL_PLANE();

// returns the current a-axis position
extern double GET_EXTERNAL_POSITION_A();

// returns the current b-axis position
extern double GET_EXTERNAL_POSITION_B();

// returns the current c-axis position
extern double GET_EXTERNAL_POSITION_C();

// returns the current x-axis position
extern double GET_EXTERNAL_POSITION_X();

// returns the current y-axis position
extern double GET_EXTERNAL_POSITION_Y();

// returns the current z-axis position
extern double GET_EXTERNAL_POSITION_Z();

// returns the current u-axis position
extern double GET_EXTERNAL_POSITION_U();

// returns the current v-axis position
extern double GET_EXTERNAL_POSITION_V();

// returns the current w-axis position
extern double GET_EXTERNAL_POSITION_W();


// Returns the position of the specified axis at the last probe trip,
// in the current work coordinate system.
extern double GET_EXTERNAL_PROBE_POSITION_A();
extern double GET_EXTERNAL_PROBE_POSITION_B();
extern double GET_EXTERNAL_PROBE_POSITION_C();
extern double GET_EXTERNAL_PROBE_POSITION_X();
extern double GET_EXTERNAL_PROBE_POSITION_Y();
extern double GET_EXTERNAL_PROBE_POSITION_Z();
extern double GET_EXTERNAL_PROBE_POSITION_U();
extern double GET_EXTERNAL_PROBE_POSITION_V();
extern double GET_EXTERNAL_PROBE_POSITION_W();

// Returns the value for any analog non-contact probing.
extern double GET_EXTERNAL_PROBE_VALUE();

// Whether the probe changed state during the last probing move
extern int GET_EXTERNAL_PROBE_TRIPPED_VALUE();

// Returns zero if queue is not empty, non-zero if the queue is empty
// This always returns a valid value
extern int GET_EXTERNAL_QUEUE_EMPTY();

// Returns the system value for spindle speed in rpm
extern double GET_EXTERNAL_SPEED(int spindle);

// Returns the system value for direction of spindle turning
extern CANON_DIRECTION GET_EXTERNAL_SPINDLE(int spindle);

// returns current tool length offset
extern double GET_EXTERNAL_TOOL_LENGTH_XOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_YOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_ZOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_AOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_BOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_COFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_UOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_VOFFSET();
extern double GET_EXTERNAL_TOOL_LENGTH_WOFFSET();

// Returns the system value for the carousel slot in which the tool
// currently in the spindle belongs. Return value zero means there is no
// tool in the spindle.
extern int GET_EXTERNAL_TOOL_SLOT();

// Returns the system value for the selected slot. That one will be the next
// valid tool after a toolchange (m6). Return value -1 means there is no
// selected tool.
extern int GET_EXTERNAL_SELECTED_TOOL_SLOT();

// Returns the CANON_TOOL_TABLE structure associated with the tool
// in the given pocket
extern CANON_TOOL_TABLE GET_EXTERNAL_TOOL_TABLE(int pocket);

// return the value of iocontrol's toolchanger-fault pin
extern int GET_EXTERNAL_TC_FAULT();

// return the value of iocontrol's toolchanger-reason pin
int GET_EXTERNAL_TC_REASON();

// Returns the system traverse rate
extern double GET_EXTERNAL_TRAVERSE_RATE();

// Returns the enabled/disabled status for feed override, spindle
// override, adaptive feed, and feed hold
extern int GET_EXTERNAL_FEED_OVERRIDE_ENABLE();
extern int GET_EXTERNAL_SPINDLE_OVERRIDE_ENABLE(int spindle);
extern int GET_EXTERNAL_ADAPTIVE_FEED_ENABLE();
extern int GET_EXTERNAL_FEED_HOLD_ENABLE();

// Functions to query digital/analog Inputs
/* def is a default value which should be returned by canon functions
   that can't actually read external hardware - simulators and such */
extern int GET_EXTERNAL_DIGITAL_INPUT(int index, int def);
/* returns current value of the digital input selected by index.*/

extern double GET_EXTERNAL_ANALOG_INPUT(int index, double def);
/* returns current value of the analog input selected by index.*/

// Returns the mask of axes present in the system
extern int GET_EXTERNAL_AXIS_MASK();


#define PARAMETER_FILE_NAME_LENGTH 100

#define USER_DEFINED_FUNCTION_NUM 100
typedef void (*USER_DEFINED_FUNCTION_TYPE) (int num, double arg1,
					    double arg2);
extern USER_DEFINED_FUNCTION_TYPE
    USER_DEFINED_FUNCTION[USER_DEFINED_FUNCTION_NUM];
extern int USER_DEFINED_FUNCTION_ADD(USER_DEFINED_FUNCTION_TYPE func,
				     int num);
				     
/* to be called by emcTaskPlanExecute when done interpreting.  This causes the
 * last segment to be output, if it has been held to do segment merging */
extern void FINISH(void);

// to be called when there is an abort, to dump the last segment instead of adding
// it to the interp list in certain cases
extern void ON_RESET(void);

// expose CANON_ERROR
extern void CANON_ERROR(const char *fmt, ...) __attribute__((format(printf,1,2)));

extern int     GET_EXTERNAL_OFFSET_APPLIED();
extern EmcPose GET_EXTERNAL_OFFSETS();
extern void UPDATE_TAG(const StateTag& tag);

#endif				/* ifndef CANON_HH */
