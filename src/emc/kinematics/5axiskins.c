/********************************************************************
* Description: 5axiskins.c
*   kinematics for XYZBC 5 axis bridge mill
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2007 Chris Radek
*
* Notes:
*  1) pivot_length hal pin must agree with mechanical
*     design (including vismach simulation) and augmented
*     with current tool z offset
*     (typ: mechanical_pivot_length + motion.tooloffset.z)
*  2) C axis: spherical coordinates aziumthal angle (t or theta)
*     projection of radius to xy plane
*  3) B axis: spherical coordinates polar angle (p or phi)
*     wrt z axis
*  4) W axis: tool motion. Negative values increase tool radial
*     motion example: drilling into body at b,c angles
*  5) W axis motion is incorporated into the motion of the
*     joints used for X,Y,Z positioning and no motor or
*     hal pin connections are required for the joint specified
*     as JW.  However, a joint must be configured for W to
*     support display of the W axis letter value for
*     complicated reasons. (motion/control.c computes joint
*     positions only for the number of configured kinematic
*     joints (NO_OF_KINS_JOINTS) and the joint positions
*     are needed to display axis letters via inverse
*     kinematics.
*  6) If no coordinates module parameter is supplied, kins
*     will use the required coordinates XYZBCW mapped
*     to joints 0..5 in sequence.
*  7) Multiple joints may be assigned to an axis letter
*     with the module coordinates parameter
*  8) If a coordinates module parameter is supplied,
*     the kins will map coordinate letters in sequence
*     to joint numbers beginning with joint 0.
*  9) Coordinates XYZBCW are required, AUV may be used
*     if specified with the coordinates parameter and will
*     be mapped one-to-one with the assigned joint.
* 10) The direction of the tilt axis is the opposite of the 
*     conventional axis direction. See 
*     https://linuxcnc.org/docs/html/gcode/machining-center.html
********************************************************************/

// non-required coordinates (A,U,V) can be set by using
// the module coordinates parameter
#define REQUIRED_COORDINATES "XYZBCW"

#define DEFAULT_PIVOT_LENGTH 250

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "rtapi_ctype.h"
#include "kinematics.h"
#include "posemath.h"
#include "switchkins.h"

struct haldata {
    hal_float_t *pivot_length;
} *haldata;
static int fiveaxis_max_joints;

static PmCartesian s2r(double r, double t, double p) {
    // s2r: spherical coordinates to cartesian coordinates
    // r       = length of vector
    // p=phi   = angle of vector wrt z axis
    // t=theta = angle of vector projected onto xy plane
    //           (projection length in xy plane is r*sin(p)
    PmCartesian c;
    t = TO_RAD*t; p = TO_RAD*p; // degrees to radians

    c.x = r * sin(p) * cos(t);
    c.y = r * sin(p) * sin(t);
    c.z = r * cos(p);

    return c;
} //s2r()

// assignments of principal joints to axis letters:
// (-1 means not defined (yet))
static int JX = -1;
static int JY = -1;
static int JZ = -1;
static int JA = -1;
static int JB = -1;
static int JC = -1;
static int JU = -1;
static int JV = -1;
static int JW = -1;

static int fiveaxis_KinematicsForward(const double *joints,
                                      EmcPose * pos,
                                      const KINEMATICS_FORWARD_FLAGS * fflags,
                                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    (void)fflags;
    (void)iflags;
    PmCartesian r = s2r(*(haldata->pivot_length) + joints[JW],
                        joints[JC],
                        180.0 - joints[JB]);

    // Note: 'principal' joints are used
    pos->tran.x = joints[JX] + r.x;
    pos->tran.y = joints[JY] + r.y;
    pos->tran.z = joints[JZ] + *(haldata->pivot_length) + r.z;
    pos->b      = joints[JB];
    pos->c      = joints[JC];
    pos->w      = joints[JW];

    // optional letters (specify with coordinates module parameter)
    pos->a = (JA != -1)? joints[JA] : 0;
    pos->u = (JU != -1)? joints[JU] : 0;
    pos->v = (JV != -1)? joints[JV] : 0;

    return 0;
} //fiveaxis_KinematicsForward()

static int fiveaxis_KinematicsInverse(const EmcPose * pos,
                                      double *joints,
                                      const KINEMATICS_INVERSE_FLAGS * iflags,
                                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)iflags;
    (void)fflags;
    PmCartesian r = s2r(*(haldata->pivot_length) + pos->w,
                        pos->c,
                        180.0 - pos->b);

    EmcPose P;  // computed position
    P.tran.x = pos->tran.x - r.x;
    P.tran.y = pos->tran.y - r.y;
    P.tran.z = pos->tran.z - *(haldata->pivot_length) - r.z;

    P.b = pos->b;
    P.c = pos->c;
    P.w = pos->w;

    // optional letters (specify with coordinates module parameter)
    P.a = (JA != -1)? pos->a : 0;
    P.u = (JU != -1)? pos->u : 0;
    P.v = (JV != -1)? pos->v : 0;

    // update joints with support for
    // multiple-joints per-coordinate letter:
    // based on computed position
    position_to_mapped_joints(fiveaxis_max_joints,
                              &P,
                              joints);
    return 0;
} // fiveaxis_kinematicsInverse()

int fiveaxis_KinematicsSetup(const  int   comp_id,
                             const  char* coordinates,
                             kparms*      kp)
{
    int result=0;
    int i,jno;
    int axis_idx_for_jno[EMCMOT_MAX_JOINTS];
    int minjoints = strlen(kp->required_coordinates);
    fiveaxis_max_joints = strlen(coordinates); // allow for dup coords

    if (fiveaxis_max_joints > kp->max_joints) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "ERROR %s: coordinates=%s requires %d joints, max joints=%d\n",
             kp->kinsname,
             coordinates,
             fiveaxis_max_joints,
             kp->max_joints);
        goto error;
    }

    if (map_coordinates_to_jnumbers(coordinates,
                                    kp->max_joints,
                                    kp->allow_duplicates,
                                    axis_idx_for_jno)) {
       goto error;
    }
    // require all chars in reqd_coordinates (order doesn't matter)
    for (i=0; i < minjoints; i++) {
        char  reqd_char;
        reqd_char = *(kp->required_coordinates + i);
        if (   !strchr(coordinates,toupper(reqd_char))
            && !strchr(coordinates,tolower(reqd_char)) ) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                 "ERROR %s:\nrequired  coordinates:%s\n"
                           "specified coordinates:%s\n",
                 kp->kinsname, kp->required_coordinates, coordinates);
            goto error;
        }
    }
    // assign principal joint numbers (first found in coordinates map)
    // duplicates are handled by position_to_mapped_joints()
    for (jno=0; jno<EMCMOT_MAX_JOINTS; jno++) {
        if (axis_idx_for_jno[jno] == 0) {if (JX == -1) JX=jno;}
        if (axis_idx_for_jno[jno] == 1) {if (JY == -1) JY=jno;}
        if (axis_idx_for_jno[jno] == 2) {if (JZ == -1) JZ=jno;}
        if (axis_idx_for_jno[jno] == 3) {if (JA == -1) JA=jno;}
        if (axis_idx_for_jno[jno] == 4) {if (JB == -1) JB=jno;}
        if (axis_idx_for_jno[jno] == 5) {if (JC == -1) JC=jno;}
        if (axis_idx_for_jno[jno] == 6) {if (JU == -1) JU=jno;}
        if (axis_idx_for_jno[jno] == 7) {if (JV == -1) JV=jno;}
        if (axis_idx_for_jno[jno] == 8) {if (JW == -1) JW=jno;}
    }

    haldata = hal_malloc(sizeof(struct haldata));

    result = hal_pin_float_newf(HAL_IN,&(haldata->pivot_length),comp_id,
                                "%s.pivot-length",kp->halprefix);
    if(result < 0) goto error;

    *haldata->pivot_length = DEFAULT_PIVOT_LENGTH;

    rtapi_print("Kinematics Module %s\n",__FILE__);
    rtapi_print("  module name = %s\n"
                "  coordinates = %s  Requires: [KINS]JOINTS>=%d\n"
                "  sparm       = %s\n",
                kp->kinsname,
                coordinates,fiveaxis_max_joints,
                kp->sparm?kp->sparm:"NOTSPECIFIED");
    rtapi_print("  default pivot-length = %.3f\n",*haldata->pivot_length);

    return 0;

error:
    return -1;
} // fiveaxis_KinematicsSetup()

int switchkinsSetup(kparms* kp,
                    KS* kset0, KS* kset1, KS* kset2,
                    KF* kfwd0, KF* kfwd1, KF* kfwd2,
                    KI* kinv0, KI* kinv1, KI* kinv2
                   )
{
    kp->kinsname    = "5axiskins"; // !!! must agree with filename
    kp->halprefix   = "5axiskins"; // hal pin names
    kp->required_coordinates = REQUIRED_COORDINATES;
    kp->allow_duplicates     = 1;
    kp->max_joints           = EMCMOT_MAX_JOINTS;

    if (kp->sparm && strstr(kp->sparm,"identityfirst")) {
        rtapi_print("\n!!! switchkins-type 0 is IDENTITY\n");
        *kset0 = identityKinematicsSetup;
        *kfwd0 = identityKinematicsForward;
        *kinv0 = identityKinematicsInverse;

        *kset1 = fiveaxis_KinematicsSetup;
        *kfwd1 = fiveaxis_KinematicsForward;
        *kinv1 = fiveaxis_KinematicsInverse;
    } else {
        rtapi_print("\n!!! switchkins-type 0 is %s\n",kp->kinsname);
        *kset0 = fiveaxis_KinematicsSetup;
        *kfwd0 = fiveaxis_KinematicsForward;
        *kinv0 = fiveaxis_KinematicsInverse;

        *kset1 = identityKinematicsSetup;
        *kfwd1 = identityKinematicsForward;
        *kinv1 = identityKinematicsInverse;
    }
    *kset2 = userkKinematicsSetup;
    *kfwd2 = userkKinematicsForward;
    *kinv2 = userkKinematicsInverse;

    return 0;
} // switchkinsSetup()
