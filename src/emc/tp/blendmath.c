/********************************************************************
* Description: blendmath.c
*   Circular arc blend math functions
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*   
* Copyright (c) 2014 All rights reserved.
*
* Last change:
********************************************************************/

#include "posemath.h"
#include "tc_types.h"
#include "tc.h"
#include "tp_types.h"
#include "rtapi_math.h"
#include "spherical_arc.h"
#include "blendmath.h"
#include "tp_debug.h"

/** @section utilityfuncs Utility functions */

/**
 * Find the maximum angle allowed between "tangent" segments.
 * @param v speed of motion in worst case (i.e. at max feed).
 * @param acc magnitude of acceleration allowed during "kink".
 *
 * Since we are discretized by a timestep, the maximum allowable
 * "kink" in a trajectory is bounded by normal acceleration. A small
 * kink will effectively be one step along the tightest radius arc
 * possible at a given speed.
 */
double findMaxTangentAngle(double v_plan, double acc_limit, double cycle_time)
{
    //Find acc hiccup we're allowed to get
    //TODO somewhat redundant with findKinkAccel, should refactor
    double acc_margin = BLEND_ACC_RATIO_NORMAL * BLEND_KINK_FACTOR * acc_limit;
    double dx = v_plan / cycle_time;
    if (dx > 0.0) {
        return (acc_margin / dx);
    } else {
        tp_debug_print(" Velocity or period is negative!\n");
        //Should not happen...
        return TP_ANGLE_EPSILON;
    }
}


/**
 * Find the acceleration required to create a specific change in path
 * direction, assuming constant speed.
 * This determines how much of a "spike" in acceleration will occur due to a
 * slight mismatch between tangent directions at the start / end of a segment.
 */
double findKinkAccel(double kink_angle, double v_plan, double cycle_time)
{
    double dx = v_plan / cycle_time;
    if (dx > 0.0) {
        return (dx * kink_angle);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "dx < 0 in KinkAccel\n");
        return 0;
    }
}


/**
 * Sign function that returns a valid numerical result for sign(0), rather than NaN.
 */
double fsign(double f)
{
    if (f>0) {
        return 1.0;
    } else if (f < 0) {
        return -1.0;
    } else {
        //Technically this should be NAN but that's a useless result for tp purposes
        return 0;
    }
}

/** negate a value (or not) based on a bool parameter */
static inline double negate(double f, int neg)
{
    return (neg) ? -f : f;
}

/** Clip the input at the specified minimum (in place). */
int clip_min(double * const x, double min) {
    if ( *x < min ) {
        *x = min;
        return 1;
    }
    return 0;
}


/** Clip the input at the specified maximum (in place). */
int clip_max(double * const x, double max) {
    if ( *x > max ) {
        *x = max;
        return 1;
    }
    return 0;
}

/**
 * Saturate a value x to be within +/- max.
 */
double saturate(double x, double max) {
    if ( x > max ) {
        return max;
    }
    else if ( x < (-max) ) {
        return -max;
    }
    else {
        return x;
    }
}

/**
 * Saturate a value x to be within max and min.
 */
double bisaturate(double x, double max, double min) {
    if ( x > max ) {
        return max;
    }
    else if ( x < min ) {
        return min;
    }
    else {
        return x;
    }
}


/**
 * Apply bounds to a value x.
 */
inline double bound(double x, double max, double min) {
    if ( x > max ) {
        return max;
    }
    else if ( x < (min) ) {
        return min;
    }
    else {
        return x;
    }
}


/** In-place saturation function */
int sat_inplace(double * const x, double max) {
    if ( *x > max ) {
        *x = max;
        return 1;
    }
    else if ( *x < -max ) {
        *x = -max;
        return -1;
    }
    return 0;
}

#if 0
static int pmCirclePrint(PmCircle const * const circ) {
    tp_debug_print(" center = %f %f %f\n",
            circ->center.x,
            circ->center.y,
            circ->center.z);
    tp_debug_print(" radius = %f\n", circ->radius);
    tp_debug_print(" spiral = %f\n", circ->spiral);
    tp_debug_print(" angle = %f\n", circ->angle);
    //TODO add other debug data here as needed
    return TP_ERR_OK;
}
#endif

/**
 * @section geomfuncs Geometry check functions
 */

/**
 * Calculate the best-fit circle to the spiral segment.
 * @param circ spiral to be approximated
 * @param base_pt the point about which the circle is fit
 * @param u_tan tangent unit vector at the base point of the approximation
 * @param[out] center_out displaced center for circular approximation
 * @param[out] radius_out adjusted radius
 *
 * The adjusted center for the circle fit is found by displacing the center
 * along the spiral tangent by the spiral coefficient. The adjusted radius is
 * the distance between the base point and this new center.
 *
 */
static inline int findSpiralApproximation(PmCircle const * const circ,
        PmCartesian const * const base_pt,
        PmCartesian const * const u_tan,
        PmCartesian * const center_out,
        double * const radius_out)
{
    double dr = circ->spiral / circ->angle;

    /*tp_debug_print("In findSpiralApproximation\n");*/
    /*tp_debug_print(" dr = %f\n",dr);*/
    /*tp_debug_print(" utan = %f %f %f\n",*/
            /*u_tan->x,*/
            /*u_tan->y,*/
            /*u_tan->z);*/
    pmCartScalMult(u_tan, dr, center_out);
    /*tp_debug_print(" circcenter = %f %f %f\n",*/
            /*circ->center.x,*/
            /*circ->center.y,*/
            /*circ->center.z);*/
    pmCartCartAddEq(center_out, &circ->center);

    PmCartesian r_adjust;
    pmCartCartSub(base_pt, center_out, &r_adjust);
    pmCartMag(&r_adjust, radius_out);

    tp_debug_print(" adjusted center = %f %f %f\n",
            center_out->x,
            center_out->y,
            center_out->z);

    tp_debug_print(" adjusted radius = %f\n", *radius_out);

    return TP_ERR_OK;
}


/**
 * Calculate the angle to trim from a circle based on the blend geometry.
 *
 * @param P intersection point
 * @param arc_center calculated center of blend arc
 * @param center actual center of circle (not spiral approximated center)
 * @return trim angle
 */
static inline double findTrimAngle(PmCartesian const * const P,
        PmCartesian const * const arc_center,
        PmCartesian const * const center)
{
    //Define vectors relative to circle center
    PmCartesian u_P;
    pmCartCartSub(P, center, &u_P);
    pmCartUnitEq(&u_P);

    PmCartesian u_arccenter;
    pmCartCartSub(arc_center, center, &u_arccenter);
    pmCartUnitEq(&u_arccenter);

    double dot;
    pmCartCartDot(&u_arccenter, &u_P, &dot);
    double dphi = acos(saturate(dot,1.0));
    tp_debug_print(" dphi = %g\n",dphi);
    return dphi;
}


/**
 * Verify that a blend arc is tangent to a circular arc.
 */
int checkTangentAngle(PmCircle const * const circ, SphericalArc const * const arc, BlendGeom3 const * const geom, BlendParameters const * const param, double cycle_time, int at_end)
{
    (void)geom;
    // Debug Information to diagnose tangent issues
    PmCartesian u_circ, u_arc;
    arcTangent(arc, &u_arc, at_end);

    if (at_end) {
        pmCircleTangentVector(circ, 0, &u_circ);
    } else {
        pmCircleTangentVector(circ, circ->angle, &u_circ);
    }

    pmCartUnitEq(&u_arc);

    // Find angle between tangent unit vectors
    double dot;
    pmCartCartDot(&u_circ, &u_arc, &dot);
    double blend_angle = acos(saturate(dot,1.0));

    // Check against the maximum allowed tangent angle for the given velocity and acceleration
    double angle_max = findMaxTangentAngle(param->v_plan, param->a_max, cycle_time);

    tp_debug_print("tangent angle = %f, max = %f\n",
            blend_angle,
            angle_max);

    tp_debug_print("circ_tan = [%g %g %g]\n",
            u_circ.x,
            u_circ.y,
            u_circ.z);
    tp_debug_print("arc_tan = [%g %g %g]\n",
            u_arc.x,
            u_arc.y,
            u_arc.z);

    PmCartesian diff;
    pmCartCartSub(&u_arc,&u_circ,&diff);
    tp_debug_print("diff = [%g %g %g]\n",
            diff.x,
            diff.y,
            diff.z);

    if (blend_angle > angle_max) {
        tp_debug_print("angle too large\n");
        return TP_ERR_FAIL;
    }

    return TP_ERR_OK;
}


/**
 * Checks if two UNIT vectors are parallel to the given angle tolerance (in radians).
 * @warning tol depends on the small angle approximation and will not be
 * accurate for angles larger than about 10 deg. This function is meant for
 * small tolerances!
 */
int pmCartCartParallel(PmCartesian const * const u1,
                       PmCartesian const * const u2,
                       double tol)
{
    double d_diff;
    {
        PmCartesian u_diff;
        pmCartCartSub(u1, u2, &u_diff);
        pmCartMagSq(&u_diff, &d_diff);
    }

    tp_debug_json_start(pmCartCartParallel);
    tp_debug_json_double(d_diff);
    tp_debug_json_end();

    return d_diff < tol;
}

/**
 * Checks if two UNIT vectors are anti-parallel to the given angle tolerance (in radians).
 * @warning tol depends on the small angle approximation and will not be
 * accurate for angles larger than about 10 deg. This function is meant for
 * small tolerances!
 */
int pmCartCartAntiParallel(PmCartesian const * const u1,
                           PmCartesian const * const u2,
                           double tol)
{
    double d_sum;
    {
        PmCartesian u_sum;
        pmCartCartAdd(u1, u2, &u_sum);
        pmCartMagSq(&u_sum, &d_sum);
    }

    tp_debug_json_start(pmCartCartAntiParallel);
    tp_debug_json_double(d_sum);
    tp_debug_json_end();

    return d_sum < tol;
}


/**
 * Check if two cartesian vectors are parallel or anti-parallel
 * The input tolerance specifies what the maximum angle between the
 * lines containing two vectors is. Note that vectors pointing in
 * opposite directions are still considered parallel, since their
 * containing lines are parallel.
 * @param u1 input unit vector 1
 * @param u2 input unit vector 2
 * @pre BOTH u1 and u2 must be unit vectors or calculation may be skewed.
 */
int pmUnitCartsColinear(PmCartesian const * const u1,
        PmCartesian const * const u2)
{
    return pmCartCartParallel(u1, u2, TP_ANGLE_EPSILON_SQ) || pmCartCartAntiParallel(u1, u2, TP_ANGLE_EPSILON_SQ);
}


/**
 * Somewhat redundant function to calculate the segment intersection angle.
 * The intersection angle is half of the supplement of the "divergence" angle
 * between unit vectors. If two unit vectors are pointing in the same
 * direction, then the intersection angle is PI/2. This is based on the
 * simple_tp formulation for tolerances.
 */
int findIntersectionAngle(PmCartesian const * const u1,
        PmCartesian const * const u2, double * const theta)
{
    double dot;
    pmCartCartDot(u1, u2, &dot);

    if (dot > 1.0 || dot < -1.0) {
        tp_debug_print("dot product %.16g outside domain of acos! u1 = %.16g %.16g %.16g, u2 = %.16g %.16g %.16g\n",
                dot,
                u1->x,
                u1->y,
                u1->z,
                u2->x,
                u2->y,
                u2->z);
        sat_inplace(&dot,1.0);
    }

    *theta = acos(-dot)/2.0;
    return TP_ERR_OK;
}


/** Calculate the minimum of the three values in a PmCartesian. */
double pmCartMin(PmCartesian const * const in)
{
    return fmin(fmin(in->x,in->y),in->z);
}


/**
 * Calculate the diameter of a circle incscribed on a central cross section of a 3D
 * rectangular prism.
 *
 * @param normal normal direction of plane slicing prism.
 * @param extents distance from center to one corner of the prism.
 * @param diameter diameter of inscribed circle on cross section.
 *
 */
int calculateInscribedDiameter(PmCartesian const * const normal,
        PmCartesian const * const bounds, double * const diameter)
{
    if (!normal ) {
        return TP_ERR_MISSING_INPUT;
    }

    double n_mag;
    pmCartMagSq(normal, &n_mag);
    double mag_err = fabs(1.0 - n_mag);
    if (mag_err > pmSqrt(TP_POS_EPSILON)) {
        /*rtapi_print_msg(RTAPI_MSG_ERR,"normal vector <%.12g,%.12f,%.12f> has magnitude error = %e\n",*/
                /*normal->x,*/
                /*normal->y,*/
                /*normal->z,*/
                /*mag_err);*/
        return TP_ERR_FAIL;
    }

    PmCartesian planar_x,planar_y,planar_z;

    //Find perpendicular component of unit directions
    // FIXME Assumes normal is unit length
    
    /* This section projects the X / Y / Z unit vectors onto the plane
     * containing the motions. The operation is done "backwards" here due to a
     * quirk with posemath. 
     *
     */
    pmCartScalMult(normal, -normal->x, &planar_x);
    pmCartScalMult(normal, -normal->y, &planar_y);
    pmCartScalMult(normal, -normal->z, &planar_z);

    planar_x.x += 1.0;
    planar_y.y += 1.0;
    planar_z.z += 1.0;

    pmCartAbs(&planar_x, &planar_x);
    pmCartAbs(&planar_y, &planar_y);
    pmCartAbs(&planar_z, &planar_z);

    // Crude way to prevent divide-by-zero-error
    planar_x.x = fmax(planar_x.x,TP_POS_EPSILON);
    planar_y.y = fmax(planar_y.y,TP_POS_EPSILON);
    planar_z.z = fmax(planar_z.z,TP_POS_EPSILON);

    double x_scale, y_scale, z_scale;
    pmCartMag(&planar_x, &x_scale);
    pmCartMag(&planar_y, &y_scale);
    pmCartMag(&planar_z, &z_scale);

    double x_extent=0, y_extent=0, z_extent=0;
    if (bounds->x != 0) {
        x_extent = bounds->x / x_scale;
    }
    if (bounds->y != 0) {
        y_extent = bounds->y / y_scale;
    }
    if (bounds->z != 0) {
        z_extent = bounds->z / z_scale;
    }

    // Find the highest value to start from
    *diameter = fmax(fmax(x_extent, y_extent),z_extent);

    // Only for active axes, find the minimum extent
    if (bounds->x != 0) {
        *diameter = fmin(*diameter, x_extent);
    }
    if (bounds->y != 0) {
        *diameter = fmin(*diameter, y_extent);
    }
    if (bounds->z != 0) {
        *diameter = fmin(*diameter, z_extent);
    }

    return TP_ERR_OK;
}



int findAccelScale(PmCartesian const * const acc,
        PmCartesian const * const bounds,
        PmCartesian * const scale)
{
    if (!acc || !bounds ) {
        return TP_ERR_MISSING_INPUT;
    }

    if (!scale ) {
        return TP_ERR_MISSING_OUTPUT;
    }

    // Find the scale of acceleration vs. machine accel bounds
    if (bounds->x != 0) {
    scale->x = fabs(acc->x / bounds->x);
    } else {
        scale->x = 0;
    }
    if (bounds->y != 0) {
    scale->y = fabs(acc->y / bounds->y);
    } else {
        scale->y = 0;
    }

    if (bounds->z != 0) {
    scale->z = fabs(acc->z / bounds->z);
    } else {
        scale->z = 0;
    }

    return TP_ERR_OK;
}




/** Find real roots of a quadratic equation in standard form. */
int quadraticFormula(double A, double B, double C, double * const root0,
        double * const root1)
{
    double disc = pmSq(B) - 4.0 * A * C;
    if (disc < 0) {
        tp_debug_print("discriminant %.12g < 0, A=%.12g, B=%.12g,C=%.12g\n", disc, A, B, C);
        return TP_ERR_FAIL;
    }
    double t1 = pmSqrt(disc);
    if (root0) {
        *root0 = ( -B + t1) / (2.0 * A);
    }
    if (root1) {
        *root1 = ( -B - t1) / (2.0 * A);
    }
    return TP_ERR_OK;
}

/**
 * @section blending blend math functions
 */

/**
 * Setup common geom parameters based on trajectory segments.
 * This function populates the geom structure and "input" fields of
 * the blend parameter structure. It returns an error if the segments
 * are not coplanar, or if one or both segments is not a circular arc.
 *
 * @param geom Stores simplified geometry used to calculate blend params.
 * @param prev_tc first linear move to blend
 * @param tc second linear move to blend
 */
int blendGeom3Init(BlendGeom3 * const geom,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc)
{
    geom->v_max1 = prev_tc->maxvel;
    geom->v_max2 = tc->maxvel;

    // Get tangent unit vectors to each arc at the intersection point
    int res_u1 = tcGetEndTangentUnitVector(prev_tc, &geom->u_tan1);
    int res_u2 = tcGetStartTangentUnitVector(tc, &geom->u_tan2);

    // Initialize u1 and u2 by assuming they match the tangent direction
    geom->u1 = geom->u_tan1;
    geom->u2 = geom->u_tan2;

    int res_intersect = tcGetIntersectionPoint(prev_tc, tc, &geom->P);

    tp_debug_print("Intersection point P = %f %f %f\n",
            geom->P.x,
            geom->P.y,
            geom->P.z);

    // Find angle between tangent vectors
    int res_angle = findIntersectionAngle(&geom->u_tan1,
            &geom->u_tan2,
            &geom->theta_tan);

    // Test for intersection angle errors
    if(PM_PI / 2.0 - geom->theta_tan < TP_ANGLE_EPSILON) {
        tp_debug_print("Intersection angle too close to pi/2, can't compute normal\n");
        return TP_ERR_TOLERANCE;
    }

    if(geom->theta_tan < TP_ANGLE_EPSILON) {
        tp_debug_print("Intersection angle too small for arc fit\n");
        return TP_ERR_TOLERANCE;
    }

    blendCalculateNormals3(geom);

    return res_u1 |
        res_u2 |
        res_intersect |
        res_angle;
}


/**
 * Initialize common fields in parameters structure.
 *
 * @param geom Stores simplified geometry used to calculate blend params.
 * @param param Abstracted parameters for blending calculations
 * @param acc_bound maximum X, Y, Z machine acceleration
 * @param vel_bound maximum X, Y, Z machine velocity
 * @param maxFeedScale maximum allowed feed override (set in INI)
 */
int blendParamKinematics(BlendGeom3 * const geom,
        BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale)
{

    // KLUDGE: common operations, but not exactly kinematics
    param->phi = (PM_PI - param->theta * 2.0);

    double nominal_tolerance;
    tcFindBlendTolerance(prev_tc, tc, &param->tolerance, &nominal_tolerance);

    // Calculate max acceleration based on plane containing lines
    int res_dia = calculateInscribedDiameter(&geom->binormal, acc_bound, &param->a_max);

    // Store max normal acceleration
    param->a_n_max = param->a_max * BLEND_ACC_RATIO_NORMAL;
    tp_debug_print("a_max = %f, a_n_max = %f\n", param->a_max,
            param->a_n_max);

    // Find the nominal velocity for the blend segment with no overrides
    double v_req_prev = tcGetMaxTargetVel(prev_tc, 1.0);
    double v_req_this = tcGetMaxTargetVel(tc, 1.0);
    tp_debug_print("vr_prev = %f, vr_this = %f\n", v_req_prev, v_req_this);
    param->v_req = fmax(v_req_prev, v_req_this);

    // Find the worst-case velocity we should reach for either segment
    param->v_goal = fmax(tcGetMaxTargetVel(prev_tc, maxFeedScale),
            tcGetMaxTargetVel(tc, maxFeedScale));

    // Calculate the maximum planar velocity
    double v_planar_max = 0;
    //FIXME sloppy handling of return value
    res_dia |= calculateInscribedDiameter(&geom->binormal, vel_bound, &v_planar_max);
    tp_debug_print("v_planar_max = %f\n", v_planar_max);

    // Clip the angle at a reasonable value (less than 90 deg), to prevent div by zero
    double phi_effective = fmin(param->phi, PM_PI * 0.49);

    // Copy over maximum velocities, clipping velocity to place altitude within base
    double v_max1 = fmin(prev_tc->maxvel, tc->maxvel / cos(phi_effective));
    double v_max2 = fmin(tc->maxvel, prev_tc->maxvel / cos(phi_effective));

    tp_debug_print("v_max1 = %f, v_max2 = %f\n", v_max1, v_max2);

    // Get "altitude"
    double v_area = v_max1 * v_max2 / 2.0 * sin(param->phi);
    tp_debug_print("phi = %f\n", param->phi);
    tp_debug_print("v_area = %f\n", v_area);

    // Get "base" of triangle
    PmCartesian tmp1, tmp2, diff;
    pmCartScalMult(&geom->u1, v_max1, &tmp1);
    pmCartScalMult(&geom->u2, v_max2, &tmp2);
    pmCartCartSub(&tmp2, &tmp1, &diff);
    double base;
    pmCartMag(&diff, &base);
    tp_debug_print("v_base = %f\n", base);

    double v_max_alt = 2.0 * v_area / base;

    // Can't do altitude-based velocity calculation if we have arcs
    if (prev_tc->motion_type != TC_LINEAR || tc->motion_type != TC_LINEAR) {
        v_max_alt = 0.0;
    }

    tp_debug_print("v_max_alt = %f\n", v_max_alt);
    double v_max = fmax(v_max_alt, v_planar_max);

    tp_debug_print("v_max = %f\n", v_max);
    param->v_goal = fmin(param->v_goal, v_max);

    tp_debug_print("v_goal = %f, max scale = %f\n", param->v_goal, maxFeedScale);

    return res_dia;
}

/**
 * Setup blend parameters based on a line and an arc.
 * This function populates the geom structure and "input" fields of
 * the blend parameter structure. It returns an error if the segments
 * are not coplanar, or if one or both segments is not a circular arc.
 *
 * @param geom Stores simplified geometry used to calculate blend params.
 * @param param Abstracted parameters for blending calculations
 * @param prev_tc first linear move to blend
 * @param tc second linear move to blend
 * @param acc_bound maximum X, Y, Z machine acceleration
 * @param vel_bound maximum X, Y, Z machine velocity
 * @param maxFeedScale maximum allowed feed override (set in INI)
 */
int blendInit3FromLineArc(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale)
{

    if (tc->motion_type != TC_CIRCULAR || prev_tc->motion_type != TC_LINEAR) {
        return TP_ERR_INPUT_TYPE;
    }

    int res_init = blendGeom3Init(geom, prev_tc, tc);
    if (res_init != TP_ERR_OK) {
        return res_init;
    }

    //Fit spiral approximation
    findSpiralApproximation(&tc->coords.circle.xyz,
            &geom->P,
            &geom->u_tan2,
            &geom->center2,
            &geom->radius2);
    // Handle convexity
    param->convex2 = arcConvexTest(&geom->center2, &geom->P, &geom->u_tan1, true);
    tp_debug_print("circ2 convex: %d\n",
            param->convex2);

    //Identify max angle for first arc by blend limits
    // TODO better name?
    double blend_angle_2 = param->convex2 ? geom->theta_tan : PM_PI / 2.0;

    param->phi2_max = fmin(tc->coords.circle.xyz.angle / 3.0, blend_angle_2);
    param->theta = geom->theta_tan;

    if (param->convex2) {
        PmCartesian blend_point;
        pmCirclePoint(&tc->coords.circle.xyz,
                param->phi2_max / 2.0,
                &blend_point);
        //Create new unit vector based on secant line
        // Direction is away from P (at start of segment)
        pmCartCartSub(&blend_point, &geom->P,  &geom->u2);
        pmCartUnitEq(&geom->u2);
        //Reduce theta proportionally to the angle between the secant and the normal
        param->theta = fmin(param->theta, geom->theta_tan - param->phi2_max / 4.0);
    }

    tp_debug_print("phi2_max = %f\n", param->phi2_max);
    blendGeom3Print(geom);

    // Check that we're not below the minimum intersection angle (making too tight an arc)
    // FIXME make this an INI setting?
    const double theta_min = PM_PI / 6.0;
    if (param->theta < theta_min) {
        tp_debug_print("theta = %f < min %f, aborting arc...\n",
                param->theta,
                theta_min);
    }

    tp_debug_print("theta = %f\n", param->theta);

    param->phi = (PM_PI - param->theta * 2.0);

    param->L1 = fmin(prev_tc->target, prev_tc->nominal_length / 2.0);

    if (param->convex2) {
        //use half of the length of the chord
        param->L2 = sin(param->phi2_max/4.0) * geom->radius2;
    } else {
        param->L2 = param->phi2_max * geom->radius2;
    }

    tp_debug_print("L1 = %f, L2 = %f\n", param->L1, param->L2);

    // Setup common parameters
    int res_kin = blendParamKinematics(geom,
            param,
            prev_tc,
            tc,
            acc_bound,
            vel_bound,
            maxFeedScale);

    return res_kin;
}

int blendInit3FromArcLine(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale)
{

    if (tc->motion_type != TC_LINEAR || prev_tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_INPUT_TYPE;
    }

    int res_init = blendGeom3Init(geom, prev_tc, tc);
    if (res_init != TP_ERR_OK) {
        return res_init;
    }

    findSpiralApproximation(&prev_tc->coords.circle.xyz,
            &geom->P,
            &geom->u_tan1,
            &geom->center1,
            &geom->radius1);

    param->convex1 = arcConvexTest(&geom->center1, &geom->P, &geom->u_tan2, false);
    tp_debug_print("circ1 convex: %d\n",
            param->convex1);

    //Identify max angle for first arc by blend limits
    // TODO better name?
    double blend_angle_1 = param->convex1 ? geom->theta_tan : PM_PI / 2.0;

    param->phi1_max = fmin(prev_tc->coords.circle.xyz.angle * 2.0 / 3.0, blend_angle_1);
    param->theta = geom->theta_tan;

    // Build the correct unit vector for the linear approximation
    if (param->convex1) {
        PmCartesian blend_point;
        pmCirclePoint(&prev_tc->coords.circle.xyz,
                prev_tc->coords.circle.xyz.angle - param->phi1_max / 2.0 ,
                &blend_point);
        //Create new unit vector based on secant line
        // Direction is toward P (at end of segment)
        pmCartCartSub(&geom->P, &blend_point, &geom->u1);
        pmCartUnitEq(&geom->u1);

        //Reduce theta proportionally to the angle between the secant and the normal
        param->theta = fmin(param->theta, geom->theta_tan - param->phi1_max / 4.0);
    }

    blendGeom3Print(geom);
    tp_debug_print("phi1_max = %f\n", param->phi1_max);

    // Check that we're not below the minimum intersection angle (making too tight an arc)
    // FIXME make this an INI setting?
    const double theta_min = PM_PI / 6.0;
    if (param->theta < theta_min) {
        tp_debug_print("theta = %f < min %f, aborting arc...\n",
                param->theta,
                theta_min);
    }

    tp_debug_print("theta = %f\n", param->theta);

    // Use end radius here
    param->L1 = param->phi1_max * (geom->radius1);
    param->L2 = tc->nominal_length / 2.0;

    if (param->convex1) {
        //use half of the length of the chord
        param->L1 = sin(param->phi1_max/4.0) * geom->radius1;
    }
    tp_debug_print("L1 = %f, L2 = %f\n", param->L1, param->L2);

    // Setup common parameters
    int res_kin = blendParamKinematics(geom,
            param,
            prev_tc,
            tc,
            acc_bound,
            vel_bound,
            maxFeedScale);

    return res_kin;
}


/**
 * Setup blend parameters based on two circular arc segments.
 * This function populates the geom structure and "input" fields of
 * the blend parameter structure. It returns an error if the segments
 * are not coplanar, or if one or both segments is not a circular arc.
 *
 * @param geom Stores simplified geometry used to calculate blend params.
 * @param param Abstracted parameters for blending calculations
 * @param prev_tc first linear move to blend
 * @param tc second linear move to blend
 * @param acc_bound maximum X, Y, Z machine acceleration
 * @param vel_bound maximum X, Y, Z machine velocity
 * @param maxFeedScale maximum allowed feed override (set in INI)
 */
int blendInit3FromArcArc(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale)
{
    if (tc->motion_type != TC_CIRCULAR || prev_tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }

    int res_init = blendGeom3Init(geom, prev_tc, tc);
    if (res_init != TP_ERR_OK) {
        return res_init;
    }

    findSpiralApproximation(&prev_tc->coords.circle.xyz,
            &geom->P,
            &geom->u_tan1,
            &geom->center1,
            &geom->radius1);

    findSpiralApproximation(&tc->coords.circle.xyz,
            &geom->P,
            &geom->u_tan2,
            &geom->center2,
            &geom->radius2);


    //Do normal calculation here since we need this information for accel / vel limits
    blendCalculateNormals3(geom);

    // Get intersection point from circle start
    pmCirclePoint(&tc->coords.circle.xyz, 0.0, &geom->P);
    tp_debug_print("Intersection point P = %f %f %f\n",
            geom->P.x,
            geom->P.y,
            geom->P.z);

    param->convex1 = arcConvexTest(&geom->center1, &geom->P, &geom->u_tan2, false);
    param->convex2 = arcConvexTest(&geom->center2, &geom->P, &geom->u_tan1, true);
    tp_debug_print("circ1 convex: %d, circ2 convex: %d\n",
            param->convex1,
            param->convex2);

    //Identify max angle for first arc by blend limits
    // TODO better name?
    double blend_angle_1 = param->convex1 ? geom->theta_tan : PM_PI / 2.0;
    double blend_angle_2 = param->convex2 ? geom->theta_tan : PM_PI / 2.0;

    param->phi1_max = fmin(prev_tc->coords.circle.xyz.angle * 2.0 / 3.0, blend_angle_1);
    param->phi2_max = fmin(tc->coords.circle.xyz.angle / 3.0, blend_angle_2);

    param->theta = geom->theta_tan;

    // Build the correct unit vector for the linear approximation
    if (param->convex1) {
        PmCartesian blend_point;
        pmCirclePoint(&prev_tc->coords.circle.xyz,
                prev_tc->coords.circle.xyz.angle - param->phi1_max / 2.0,
                &blend_point);
        //Create new unit vector based on secant line
        // Direction is toward P (at end of segment)
        pmCartCartSub(&geom->P, &blend_point, &geom->u1);
        pmCartUnitEq(&geom->u1);

        //Reduce theta proportionally to the angle between the secant and the normal
        param->theta = fmin(param->theta, geom->theta_tan - param->phi1_max / 4.0);

    }

    if (param->convex2) {
        PmCartesian blend_point;
        pmCirclePoint(&tc->coords.circle.xyz,
                param->phi2_max / 2.0,
                &blend_point);
        //Create new unit vector based on secant line
        // Direction is away from P (at start of segment)
        pmCartCartSub(&blend_point, &geom->P,  &geom->u2);
        pmCartUnitEq(&geom->u2);

        //Reduce theta proportionally to the angle between the secant and the normal
        param->theta = fmin(param->theta, geom->theta_tan - param->phi2_max / 4.0);
    }
    blendGeom3Print(geom);


    // Check that we're not below the minimum intersection angle (making too tight an arc)
    // FIXME make this an INI setting?
    const double theta_min = PM_PI / 12.0;
    if (param->theta < theta_min) {
        tp_debug_print("theta = %f < min %f, aborting arc...\n",
                param->theta,
                theta_min);
        return TP_ERR_FAIL;
    }

    tp_debug_print("theta = %f\n", param->theta);

    param->phi = (PM_PI - param->theta * 2.0);

    param->L1 = param->phi1_max * geom->radius1;
    param->L2 = param->phi2_max * geom->radius2;

    if (param->convex1) {
        //use half of the length of the chord
        param->L1 = sin(param->phi1_max/4.0) * geom->radius1;
    }
    if (param->convex2) {
        //use half of the length of the chord
        param->L2 = sin(param->phi2_max/4.0) * geom->radius2;
    }
    tp_debug_print("L1 = %f, L2 = %f\n", param->L1, param->L2);
    tp_debug_print("phi1_max = %f\n",param->phi1_max);
    tp_debug_print("phi2_max = %f\n",param->phi2_max);

    // Setup common parameters
    int res_kin = blendParamKinematics(geom,
            param,
            prev_tc,
            tc,
            acc_bound,
            vel_bound,
            maxFeedScale);

    return res_kin;
}

/**
 * Setup blend parameters based on two linear segments.
 * This function populates the geom structure and "input" fields of the blend parameter structure based.
 * @param geom Stores simplified geometry used to calculate blend params.
 * @param param Abstracted parameters for blending calculations
 * @param prev_tc first linear move to blend
 * @param tc second linear move to blend
 * @param acc_bound maximum X, Y, Z machine acceleration
 * @param vel_bound maximum X, Y, Z machine velocity
 * @param maxFeedScale maximum allowed feed override (set in INI)
 */
int blendInit3FromLineLine(BlendGeom3 * const geom, BlendParameters * const param,
        TC_STRUCT const * const prev_tc,
        TC_STRUCT const * const tc,
        PmCartesian const * const acc_bound,
        PmCartesian const * const vel_bound,
        double maxFeedScale)
{

    if (tc->motion_type != TC_LINEAR || prev_tc->motion_type != TC_LINEAR) {
        return TP_ERR_FAIL;
    }

    int res_init = blendGeom3Init(geom, prev_tc, tc);
    if (res_init != TP_ERR_OK) {
        return res_init;
    }

    param->theta = geom->theta_tan;

    tp_debug_print("theta = %f\n", param->theta);

    param->phi = (PM_PI - param->theta * 2.0);

    blendGeom3Print(geom);

    //Nominal length restriction prevents gobbling too much of parabolic blends
    param->L1 = fmin(prev_tc->target, prev_tc->nominal_length * BLEND_DIST_FRACTION);
    param->L2 = tc->target * BLEND_DIST_FRACTION;
    tp_debug_print("prev. nominal length = %f, next nominal_length = %f\n",
            prev_tc->nominal_length, tc->nominal_length);
    tp_debug_print("L1 = %f, L2 = %f\n", param->L1, param->L2);

    // Setup common parameters
    int res_kin = blendParamKinematics(geom,
            param,
            prev_tc,
            tc,
            acc_bound,
            vel_bound,
            maxFeedScale);

    return res_kin;
}


/**
 * Calculate plane normal and binormal based on unit direction vectors.
 */
int blendCalculateNormals3(BlendGeom3 * const geom)
{

    int err_cross = pmCartCartCross(&geom->u_tan1,
            &geom->u_tan2,
            &geom->binormal);
    int err_unit_b = pmCartUnitEq(&geom->binormal);

    tp_debug_print("binormal = [%f %f %f]\n", geom->binormal.x,
            geom->binormal.y,
            geom->binormal.z);

    pmCartCartSub(&geom->u_tan2, &geom->u_tan1, &geom->normal);
    int err_unit_n = pmCartUnitEq(&geom->normal);

    tp_debug_print("normal = [%f %f %f]\n", geom->normal.x,
            geom->normal.y,
            geom->normal.z);
    return (err_cross || err_unit_b || err_unit_n);
}

/**
 * Compute blend parameters based on line data.
 * Blend arc parameters such as radius and velocity are calculated here. These
 * parameters are later used to create the actual arc geometry in other
 * functions.
 */
int blendComputeParameters(BlendParameters * const param)
{

    // Find maximum distance h from arc center to intersection point
    double h_tol = param->tolerance / (1.0 - sin(param->theta));

    // Find maximum distance along lines allowed by tolerance
    double d_tol = cos(param->theta) * h_tol;
    tp_debug_print(" d_tol = %f\n", d_tol);

    // Find minimum distance by blend length constraints
    double d_lengths = fmin(param->L1, param->L2);
    double d_geom = fmin(d_lengths, d_tol);
    // Find radius from the limiting length
    double R_geom = tan(param->theta) * d_geom;

    // Find maximum velocity allowed by accel and radius
    double v_normal = pmSqrt(param->a_n_max * R_geom);
    tp_debug_print("v_normal = %f\n", v_normal);

    param->v_plan = fmin(v_normal, param->v_goal);

    /*Get the limiting velocity of the equivalent parabolic blend. We use the
     * time it would take to do a "stock" parabolic blend as a metric for how
     * much of the segment to consume. A long segment will have a high
     * "triangle" velocity, so the radius will only be as large as is needed to
     * reach the cornering speed. A short segment will have a low triangle
     * velocity, much lower than the actual curvature limit, which can be used
     * to calculate an equivalent blend radius.
     * */
    double a_parabolic = param->a_max * 0.5;
    double v_triangle = pmSqrt(2.0 * a_parabolic  * d_geom);
    double t_blend = fmin(v_triangle, param->v_plan) / (a_parabolic);
    double s_blend = t_blend * param->v_plan;
    double R_blend = fmin(s_blend / param->phi, R_geom);   //Clamp by limiting radius

    param->R_plan = fmax(pmSq(param->v_plan) / param->a_n_max, R_blend);
    param->d_plan = param->R_plan / tan(param->theta);

    tp_debug_print("v_plan = %f\n", param->v_plan);
    tp_debug_print("R_plan = %f\n", param->R_plan);
    tp_debug_print("d_plan = %f\n", param->d_plan);

    /* "Actual" velocity means the velocity when feed override is 1.0.  Recall
     * that v_plan may be greater than v_req by the max feed override. If our
     * worst-case planned velocity is higher than the requested velocity, then
     * clip at the requested velocity. This allows us to increase speed above
     * the feed override limits.
     */
    if (param->v_plan > param->v_req) {
        param->v_actual = param->v_req;
    } else {
        param->v_actual = param->v_plan;
    }

    // Store arc length of blend arc for future checks
    param->s_arc = param->R_plan * param->phi;

    if (param->R_plan < TP_POS_EPSILON) {
        tp_debug_print("#Blend radius too small, aborting arc\n");
        return TP_ERR_FAIL;
    }

    if (param->s_arc < TP_MIN_ARC_LENGTH) {
        tp_debug_print("#Blend arc length too small, aborting arc\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}


/** Check if the previous line segment will be consumed based on the blend arc parameters. */
int blendCheckConsume(BlendParameters * const param,
        BlendPoints3 const * const points,
        TC_STRUCT const * const prev_tc, int gap_cycles)
{
    //Initialize values
    param->consume = 0;
    param->line_length = 0;
    if (!prev_tc) {
        return -1;
    }

    if (prev_tc->motion_type != TC_LINEAR) {
        return 0;
    }

    //Check for segment length limits
    double L_prev = prev_tc->target - points->trim1;
    double prev_seg_time = L_prev / param->v_plan;

    bool can_consume = tcCanConsume(prev_tc);
    param->consume = (prev_seg_time < gap_cycles * prev_tc->cycle_time && can_consume);
    if (param->consume) {
        tp_debug_print("consuming prev line, L_prev = %g\n",
                L_prev);
        param->line_length = L_prev;
    }
    return 0;
}


/**
 * Compute spherical arc points based on blend arc data.
 * Once blend parameters are computed, the three arc points are calculated
 * here.
 */
int blendFindPoints3(BlendPoints3 * const points, BlendGeom3 const * const geom,
        BlendParameters const * const param)
{
    // Find center of blend arc along normal vector
    double center_dist = param->R_plan / sin(param->theta);
    tp_debug_print("center_dist = %f\n", center_dist);

    pmCartScalMult(&geom->normal, center_dist, &points->arc_center);
    pmCartCartAddEq(&points->arc_center, &geom->P);
    tp_debug_print("arc_center = %f %f %f\n",
            points->arc_center.x,
            points->arc_center.y,
            points->arc_center.z);

    // Start point is d_plan away from intersection P in the
    // negative direction of u1
    pmCartScalMult(&geom->u1, -param->d_plan, &points->arc_start);
    pmCartCartAddEq(&points->arc_start, &geom->P);
    tp_debug_print("arc_start = %f %f %f\n",
            points->arc_start.x,
            points->arc_start.y,
            points->arc_start.z);

    // End point is d_plan away from intersection P in the
    // positive direction of u1
    pmCartScalMult(&geom->u2, param->d_plan, &points->arc_end);
    pmCartCartAddEq(&points->arc_end, &geom->P);
    tp_debug_print("arc_end = %f %f %f\n",
            points->arc_end.x,
            points->arc_end.y,
            points->arc_end.z);

    //For line case, just copy over d_plan since it's the same
    points->trim1 = param->d_plan;
    points->trim2 = param->d_plan;

    return TP_ERR_OK;
}


/**
 * Take results of line blend calculation and project onto circular arc and line
 */
int blendLineArcPostProcess(BlendPoints3 * const points, BlendPoints3 const * const points_in,
        BlendParameters * const param, BlendGeom3 const * const geom,
        PmCartLine const * const line1, PmCircle const * const circ2)
{
    (void)points_in;
    (void)line1;
    (void)circ2;

    // Define distances from actual center to circle centers
    double d2 = negate(param->R_plan, param->convex2) + geom->radius2;
    tp_debug_print("d2 = %f\n", d2);

    //Get unit vector normal to line in plane, towards arc center
    PmCartesian n1;
    pmCartCartCross(&geom->binormal, &geom->u1, &n1);
    pmCartUnitEq(&n1);

    tp_debug_print("n1 = %f %f %f\n",
            n1.x,
            n1.y,
            n1.z);

    PmCartesian r_PC2;
    pmCartCartSub(&geom->center2, &geom->P, &r_PC2);

    double c2_u,c2_n; //Components of C2-P on u1 and n1
    pmCartCartDot(&r_PC2, &geom->u1, &c2_u);
    pmCartCartDot(&r_PC2, &n1, &c2_n);

    tp_debug_print("c2_u = %f, c2_n = %f\n",
            c2_u,
            c2_n);

    double d_L; // badly named distance along line to intersection
    double A = 1;
    double B = 2.0 * c2_u;
    double C = pmSq(c2_u) - pmSq(d2) + pmSq(param->R_plan - c2_n);
    double root0,root1;
    int res_dist = quadraticFormula(A, B, C, &root0, &root1);
    if (res_dist) {
        return TP_ERR_FAIL;
    }

    tp_debug_print("root0 = %f, root1 = %f\n", root0,
            root1);
    d_L = fmin(fabs(root0),fabs(root1));
    if (d_L < 0) {
        tp_debug_print("d_L can't be < 0, aborting...\n");
        return TP_ERR_FAIL;
    }

    PmCartesian C_u, C_n;

    pmCartScalMult(&geom->u1, -d_L, &C_u);
    pmCartScalMult(&n1, param->R_plan, &C_n);

    PmCartesian r_PC;
    //Continue with correct solution, get actual center
    pmCartCartAdd(&C_u, &C_n, &r_PC);
    pmCartCartAdd(&geom->P, &r_PC, &points->arc_center);
    tp_debug_print("arc center = %f %f %f\n",
            points->arc_center.x,
            points->arc_center.y,
            points->arc_center.z);

    //Verify tolerances
    double h;
    pmCartMag(&r_PC, &h);
    tp_debug_print("center_dist = %f\n", h);

    double T_final = h - param->R_plan;
    tp_debug_print("T_final = %f\n",T_final);
    if (T_final > param->tolerance) {
        tp_debug_print("Projected circle T (%f) exceeds tolerance %f, aborting blend arc\n",
                T_final,
                param->tolerance);
        return TP_ERR_FAIL;
    }

    points->trim1 = d_L;

    points->trim2 = findTrimAngle(&geom->P,
            &points->arc_center,
            &geom->center2);

    return TP_ERR_OK;
}


/**
 * Take results of line blend calculation and project onto circular arc and line
 */
int blendArcLinePostProcess(BlendPoints3 * const points,
        BlendPoints3 const * const points_in,
        BlendParameters * const param,
        BlendGeom3 const * const geom,
        PmCircle const * const circ1,
        PmCartLine const * const line2)
{
    (void)points_in;
    (void)circ1;
    (void)line2;

    // Define distance from actual arc center to circle center
    double d1 = negate(param->R_plan, param->convex1) + geom->radius1;
    tp_debug_print("d1 = %f\n", d1);

    //Get unit vector normal to line in plane, towards arc center
    PmCartesian n2;
    pmCartCartCross(&geom->binormal, &geom->u2, &n2);
    pmCartUnitEq(&n2);

    tp_debug_print("n2 = %f %f %f\n",
            n2.x,
            n2.y,
            n2.z);

    PmCartesian r_PC1;
    pmCartCartSub(&geom->center1, &geom->P, &r_PC1);
    double c1_u, c1_n; //Components of C1-P on u2 and n2
    pmCartCartDot(&r_PC1, &geom->u2, &c1_u);
    pmCartCartDot(&r_PC1, &n2, &c1_n);

    double d_L; // badly named distance along line to intersection
    double A = 1;
    double B = 2.0 * c1_u;
    double C = pmSq(c1_u) - pmSq(d1) + pmSq(param->R_plan - c1_n);
    double root0,root1;
    int res_dist = quadraticFormula(A, B, C, &root0, &root1);
    if (res_dist) {
        return TP_ERR_FAIL;
    }

    tp_debug_print("root0 = %f, root1 = %f\n", root0,
            root1);
    d_L = fmin(fabs(root0),fabs(root1));
    if (d_L < 0) {
        tp_debug_print("d_L can't be < 0, aborting...\n");
        return TP_ERR_FAIL;
    }

    PmCartesian C_u, C_n;

    pmCartScalMult(&geom->u2, d_L, &C_u);
    pmCartScalMult(&n2, param->R_plan, &C_n);

    PmCartesian r_PC;
    //Continue with correct solution, get actual center
    pmCartCartAdd(&C_u, &C_n, &r_PC);
    pmCartCartAdd(&geom->P, &r_PC, &points->arc_center);
    tp_debug_print("arc center = %f %f %f\n",
            points->arc_center.x,
            points->arc_center.y,
            points->arc_center.z);

    //Verify tolerances
    double h;
    pmCartMag(&r_PC, &h);
    tp_debug_print("center_dist = %f\n", h);

    double T_final = h - param->R_plan;
    if (T_final > param->tolerance) {
        tp_debug_print("Projected circle T (%f) exceeds tolerance %f, aborting blend arc\n",
                T_final,
                param->tolerance);
        return TP_ERR_FAIL;
    }
    tp_debug_print("T_final = %f\n",T_final);

    points->trim1 = findTrimAngle(&geom->P,
            &points->arc_center,
            &geom->center1);

    points->trim2 = d_L;

    return TP_ERR_OK;
}


/**
 * "Post-process" results from linear approximation to fit the circular segments.
 * This step handles the projection from the linear approximation of each
 * circle. Given the solved radius and tolerance, this function updates the
 * points structure with the exact trim angles for each segment.
 */
int blendArcArcPostProcess(BlendPoints3 * const points, BlendPoints3 const * const points_in,
        BlendParameters * const param, BlendGeom3 const * const geom,
        PmCircle const * const circ1, PmCircle const * const circ2)
{
    (void)points_in;
    (void)circ1;
    (void)circ2;

    // Create "shifted center" approximation of spiral circles
    // TODO refers to u1 instead of utan?
    // Define distances from actual center to adjusted circle centers
    double d1 = negate(param->R_plan, param->convex1) + geom->radius1;
    double d2 = negate(param->R_plan, param->convex2) + geom->radius2;
    tp_debug_print("d1 = %f, d2 = %f\n", d1, d2);

    //Find "x" distance between C1 and C2
    PmCartesian r_C1C2;
    pmCartCartSub(&geom->center2, &geom->center1, &r_C1C2);
    double c2x;
    pmCartMag(&r_C1C2, &c2x);

    // Compute the new center location

    double Cx = (-pmSq(d1) + pmSq(d2)-pmSq(c2x)) / (-2.0 * c2x);
    double Cy = pmSqrt(pmSq(d1) - pmSq(Cx));

    tp_debug_print("Cx = %f, Cy = %f\n",Cx,Cy);

    // Find the basis vector uc from center1 to center2
    PmCartesian uc;
    //TODO catch failures here
    int norm_err = pmCartUnit(&r_C1C2, &uc);
    if (norm_err) {
        return TP_ERR_FAIL;
    }

    // Find the basis vector perpendicular to the binormal and uc
    PmCartesian nc;
    pmCartCartCross(&geom->binormal, &uc, &nc);

    //Check if nc is in the same half-plane as the intersection normal. if not,
    //we need to flip it around to choose the correct solution.
    double dot1;
    pmCartCartDot(&geom->normal, &nc, &dot1);
    if (dot1 < 0) {
        pmCartNegEq(&nc);
    }
    norm_err = pmCartUnitEq(&nc);
    if (norm_err) {
        return TP_ERR_FAIL;
    }

    //Find components of center position wrt circle 1 center.
    PmCartesian c_x, c_y;
    pmCartScalMult(&uc, Cx, &c_x);
    pmCartScalMult(&nc, Cy, &c_y);

    //Get vector from P to first center
    PmCartesian r_PC1;
    pmCartCartSub(&geom->center1, &geom->P, &r_PC1);

    // Get "test vectors, relative distance from solution center to P
    PmCartesian test1, test2;
    pmCartCartAdd(&r_PC1, &c_x, &test1);
    test2=test1;

    //Add and subtract c_y component to get equivalent of two Y solutions
    pmCartCartAddEq(&test1, &c_y);
    pmCartCartSubEq(&test2, &c_y);

    double mag1,mag2;
    pmCartMag(&test1, &mag1);
    pmCartMag(&test2, &mag2);

    if (mag2 < mag1)
    {
        //negative solution is closer
        pmCartNegEq(&c_y);
    }

    //Continue with correct solution, get actual center
    PmCartesian r_C1C;
    pmCartCartAdd(&c_x, &c_y, &r_C1C);
    pmCartCartAdd(&geom->center1, &r_C1C, &points->arc_center);
    tp_debug_print("arc center = %f %f %f\n",
            points->arc_center.x,
            points->arc_center.y,
            points->arc_center.z);

    //Find components of center position wrt circle 2 center.
    PmCartesian r_C2C;
    pmCartCartSub(&points->arc_center, &geom->center2, &r_C2C);

    PmCartesian r_PC;
    pmCartCartSub(&points->arc_center, &geom->P, &r_PC);

    //Verify tolerances
    double h;
    pmCartMag(&r_PC, &h);
    tp_debug_print("center_dist = %f\n", h);

    double T_final = h - param->R_plan;
    if (T_final > param->tolerance) {
        tp_debug_print("Projected circle T (%f) exceeds tolerance %f, aborting blend arc\n",
                T_final,
                param->tolerance);
        return TP_ERR_FAIL;
    }
    tp_debug_print("T_final = %f\n",T_final);

    points->trim1 = findTrimAngle(&geom->P,
            &points->arc_center,
            &geom->center1);
    points->trim2 = findTrimAngle(&geom->P,
            &points->arc_center,
            &geom->center2);

    tp_debug_print("trim1 = %f, trim2 = %f\n",
            points->trim1,
            points->trim2);

    return TP_ERR_OK;
}


/**
 * Setup the spherical arc struct based on the blend arc data.
 */
int arcFromBlendPoints3(SphericalArc * const arc, BlendPoints3 const * const points,
        BlendGeom3 const * const geom, BlendParameters const * const param)
{
    // If we consume the previous line, the remaining line length gets added here
    arc->uTan = geom->u_tan1;
    arc->line_length = param->line_length;
    arc->binormal = geom->binormal;

    // Create the arc from the processed points
    return arcInitFromPoints(arc, &points->arc_start,
            &points->arc_end, &points->arc_center);
}

int blendGeom3Print(BlendGeom3 const * const geom)
{
    (void)geom;
    tp_debug_print("u1 = %f %f %f\n",
            geom->u1.x,
            geom->u1.y,
            geom->u1.z);

    tp_debug_print("u2 = %f %f %f\n",
            geom->u2.x,
            geom->u2.y,
            geom->u2.z);
    return 0;
}

int blendPoints3Print(BlendPoints3 const * const points)
{
    (void)points;
    tp_debug_print("arc_start = %f %f %f\n",
            points->arc_start.x,
            points->arc_start.y,
            points->arc_start.z);

    tp_debug_print("arc_center = %f %f %f\n",
            points->arc_center.x,
            points->arc_center.y,
            points->arc_center.z);

    tp_debug_print("arc_end = %f %f %f\n",
            points->arc_end.x,
            points->arc_end.y,
            points->arc_end.z);

    return 0;

}

double pmCartAbsMax(PmCartesian const * const v)
{
    return fmax(fmax(fabs(v->x),fabs(v->y)),fabs(v->z));
}


PmCircleLimits pmCircleActualMaxVel(PmCircle const * circle,
        double v_max,
        double a_max)
{
    double a_n_max_cutoff = BLEND_ACC_RATIO_NORMAL * a_max;

    double eff_radius = pmCircleEffectiveMinRadius(circle);
    // Find the acceleration necessary to reach the maximum velocity
    double a_n_vmax = pmSq(v_max) / fmax(eff_radius, DOUBLE_FUZZ);
    // Find the maximum velocity that still obeys our desired tangential / total acceleration ratio
    double v_max_cutoff = pmSqrt(a_n_max_cutoff * eff_radius);

    double v_max_actual = v_max;
    double acc_ratio_tan = BLEND_ACC_RATIO_TANGENTIAL;

    if (a_n_vmax > a_n_max_cutoff) {
        v_max_actual = v_max_cutoff;
    } else {
        acc_ratio_tan = pmSqrt(1.0 - pmSq(a_n_vmax / a_max));
    }

    tp_debug_json_start(pmCircleActualMaxVel);
    tp_debug_json_double(eff_radius);
    tp_debug_json_double(v_max);
    tp_debug_json_double(v_max_cutoff);
    tp_debug_json_double(a_n_max_cutoff);
    tp_debug_json_double(a_n_vmax);
    tp_debug_json_double(acc_ratio_tan);
    tp_debug_json_end();

    PmCircleLimits limits = {
        v_max_actual,
        acc_ratio_tan
    };

    return limits;
}


/** @section spiralfuncs Functions to approximate spiral arc length */

/**
 * Intermediate function to find the angle for a parameter from 0..1 along the
 * spiral arc.
 */
static int pmCircleAngleFromParam(PmCircle const * const circle,
        SpiralArcLengthFit const * const fit,
        double t,
        double * const angle)
{
    if (fit->spiral_in) {
        t = 1.0 - t;
    }
    //TODO error or cleanup input to prevent param outside 0..1
    double s_in = t * fit->total_planar_length;

    // Quadratic formula to invert arc length -> angle

    double A = fit->b0;
    double B = fit->b1;
    double C = -s_in;

    double disc = pmSq(B) - 4.0 * A * C ;
    if (disc < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "discriminant %f is negative in angle calculation\n",disc);
        return TP_ERR_FAIL;
    }

    /*
     * Stability of inverting the arc-length relationship.
     * Since the b1 coefficient is analogous to arc radius, we can be
     * reasonably assured that it will be large enough not to cause numerical
     * errors. If this is not the case, then the arc itself is degenerate (very
     * small radius), and this condition should be caught well before here.
     *
     * Since an arc with a very small spiral coefficient will have a small b0
     * coefficient in the fit, we use the Citardauq Formula to ensure that the
     * positive root does not lose precision due to subtracting near-similar values.
     *
     * For more information, see:
     * http://people.csail.mit.edu/bkph/articles/Quadratics.pdf
     */

    double angle_out = (2.0 * C) / ( -B - pmSqrt(disc));

    if (fit->spiral_in) {
        // Spiral fit assumes that we're spiraling out, so
        // parameterize from opposite end
        angle_out = circle->angle - angle_out;
    }

    *angle = angle_out;
    return TP_ERR_OK;
}


static void printSpiralArcLengthFit(SpiralArcLengthFit const * const fit)
{
    (void)fit;
    tp_debug_print("Spiral fit: b0 = %.12f, b1 = %.12f, length = %.12f, spiral_in = %d\n",
            fit->b0,
            fit->b1,
            fit->total_planar_length,
            fit->spiral_in);
}

/**
 * Approximate the arc length function of a general spiral.
 *
 * The closed-form arc length of a general archimedean spiral is rather
 * computationally messy to work with. 
 * See http://mathworld.wolfram.com/ArchimedesSpiral.html for the actual form.
 *
 * The simplification here is made possible by a few assumptions:
 *  1) That the spiral starts with a nonzero radius
 *  2) The spiral coefficient (i.e. change in radius / angle) is not too large
 *  3) The spiral coefficient has some minimum magnitude ("perfect" circles are handled as a special case)
 *
 * The 2nd-order fit below works by matching slope at the start and end of the
 * arc length vs. angle curve. This completely specifies the 2nd order fit.
 * Also, this fit predicts a total arc length >= the true arc length, which
 * means the true speed along the curve will be the same or slower than the
 * nominal speed.
 */
int findSpiralArcLengthFit(PmCircle const * const circle,
        SpiralArcLengthFit * const fit)
{
    // Additional data for arc length approximation
    double spiral_coef = circle->spiral / circle->angle;
    double min_radius = circle->radius;

    if (fsign(circle->spiral) < 0.0) {
        // Treat as positive spiral, parameterized in opposite
        // direction
        spiral_coef*=-1.0;
        // Treat final radius as starting radius for fit, so we add the
        // negative spiral term to get the minimum radius
        //
        min_radius+=circle->spiral;
        fit->spiral_in = true;
    } else {
        fit->spiral_in = false;
    }
    tp_debug_print("radius = %.12f, angle = %.12f\n", min_radius, circle->angle);
    tp_debug_print("spiral_coef = %.12f\n", spiral_coef);


    //Compute the slope of the arc length vs. angle curve at the start and end of the segment
    double slope_start = pmSqrt(pmSq(min_radius) + pmSq(spiral_coef));
    double slope_end = pmSqrt(pmSq(min_radius + spiral_coef * circle->angle) + pmSq(spiral_coef));

    fit->b0 = (slope_end - slope_start) / (2.0 * circle->angle);
    fit->b1 = slope_start;

    fit->total_planar_length = fit->b0 * pmSq(circle->angle) + fit->b1 * circle->angle;
    printSpiralArcLengthFit(fit);

    // Check against start and end angle
    double angle_end_chk = 0.0;
    int res_angle = pmCircleAngleFromParam(circle, fit, 1.0, &angle_end_chk);
    if (res_angle != TP_ERR_OK) {
        //TODO better error message
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Spiral fit failed\n");
        return TP_ERR_FAIL;
    }

    // Check fit against angle
    double fit_err = angle_end_chk - circle->angle;
    if (fabs(fit_err) > TP_ANGLE_EPSILON) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Spiral fit angle difference is %e, maximum allowed is %e\n",
                fit_err,
                TP_ANGLE_EPSILON);
        return TP_ERR_FAIL;
    }

    return TP_ERR_OK;
}


/**
 * Compute the angle around a circular segment from the total progress along
 * the curve.
 */
int pmCircleAngleFromProgress(PmCircle const * const circle,
        SpiralArcLengthFit const * const fit,
        double progress,
        double * const angle)
{
    double h2;
    pmCartMagSq(&circle->rHelix, &h2);
    double s_end = pmSqrt(pmSq(fit->total_planar_length) + h2);
    // Parameterize by total progress along helix
    double t = progress / s_end;
    return pmCircleAngleFromParam(circle, fit, t, angle);
}


/**
 * Find the effective minimum radius for acceleration calculations.
 * The radius of curvature of a spiral is larger than the circle of the same
 * radius.
 */
double pmCircleEffectiveMinRadius(PmCircle const * circle)
{
    double dr = circle->spiral / circle->angle;
    double h2;
    pmCartMagSq(&circle->rHelix, &h2);

    // Exact representation of spiral arc length flattened into
    double n_inner = pmSq(dr) + pmSq(circle->radius);
    double den = n_inner+pmSq(dr);
    double num = pmSqrt(n_inner * n_inner * n_inner);
    double r_spiral = num / den;

    // Curvature of helix, assuming that helical motion is independent of plane motion
    double effective_radius = h2 / r_spiral + r_spiral;

    return effective_radius;
}

