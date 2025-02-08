/********************************************************************
 * Description: spherical_arc.c
 *
 * A simple spherical linear interpolation library and related functions.
 *
 * Author: Robert W. Ellenberg
 * License: GPL Version 2
 * System: Linux
 *    
 * Copyright (c) 2014 All rights reserved.
 *
 ********************************************************************/

#include "posemath.h"
#include "spherical_arc.h"
#include "tp_types.h"
#include "rtapi_math.h"

#include "tp_debug.h"

int arcInitFromPoints(SphericalArc * const arc, PmCartesian const * const start,
        PmCartesian const * const end,
        PmCartesian const * const center)
{
#ifdef ARC_PEDANTIC
    if (!P0 || !P1 || !center)
        return TP_ERR_MISSING_INPUT;
    
    if (!arc)
        return TP_ERR_MISSING_OUTPUT;
#endif

    // Store the start, end, and center
    arc->start = *start;
    arc->end = *end;
    arc->center = *center;

    pmCartCartSub(start, center, &arc->rStart);
    pmCartCartSub(end, center, &arc->rEnd);

    // Find the radii at start and end. These are identical for a perfect spherical arc
    double radius0, radius1;
    pmCartMag(&arc->rStart, &radius0);
    pmCartMag(&arc->rEnd, &radius1);

    tp_debug_print("radii are %g and %g\n",
            radius0,
            radius1);

    if (radius0 < ARC_MIN_RADIUS || radius1 < ARC_MIN_RADIUS) {
        tp_debug_print("radius below min radius %f, aborting arc\n",
                ARC_MIN_RADIUS);
        return TP_ERR_RADIUS_TOO_SMALL;
    }

    // Choose initial radius as nominal radius
    arc->radius = radius0;

    // Get unit vectors from center to start and center to end
    PmCartesian u0, u1;
    pmCartScalMult(&arc->rStart, 1.0 / radius0, &u0);
    pmCartScalMult(&arc->rEnd, 1.0 / radius1, &u1);

    // Find arc angle
    double dot;
    pmCartCartDot(&u0, &u1, &dot);
    arc->angle = acos(dot);
    tp_debug_print("spherical arc angle = %f\n", arc->angle);

    // Store spiral factor as radial difference. Archimedean spiral coef. a = spiral / angle
    arc->spiral = (radius1 - radius0 );

    if (arc->angle < ARC_MIN_ANGLE) {
        tp_debug_print("angle %f below min angle %f, aborting arc\n",
                arc->angle,
                ARC_MIN_ANGLE);
        return TP_ERR_GEOM;
    }

    // Store sin of arc angle since it is reused many times for SLERP
    arc->Sangle = sin(arc->angle);

    return TP_ERR_OK;
}

int arcPoint(SphericalArc const * const arc, double progress, PmCartesian * const out)
{
    //TODO pedantic

    //Convert progress to actual progress around the arc
    double net_progress = progress - arc->line_length;
    if (net_progress <= 0.0 && arc->line_length > 0) {
        tc_debug_print("net_progress = %f, line_length = %f\n", net_progress, arc->line_length);
        //Get position on line (not actually an angle in this case)
        pmCartScalMult(&arc->uTan, net_progress, out);
        pmCartCartAdd(out, &arc->start, out);
    } else {
        double angle_in = net_progress / arc->radius;
        tc_debug_print("angle_in = %f, angle_total = %f\n", angle_in, arc->angle);
        double scale0 = sin(arc->angle - angle_in) / arc->Sangle;
        double scale1 = sin(angle_in) / arc->Sangle;

        PmCartesian interp0,interp1;
        pmCartScalMult(&arc->rStart, scale0, &interp0);
        pmCartScalMult(&arc->rEnd, scale1, &interp1);

        pmCartCartAdd(&interp0, &interp1, out);
        pmCartCartAdd(&arc->center, out, out);
    }
    return TP_ERR_OK;
}

int arcLength(SphericalArc const * const arc, double * const length)
{
    *length = arc->radius * arc->angle + arc->line_length;
    tp_debug_print("arc length = %g\n", *length);
    return TP_ERR_OK;
}

int arcFromLines(SphericalArc * const arc, PmCartLine const * const line1,
        PmCartLine const * const line2, double radius,
        double blend_dist, double center_dist, PmCartesian * const start, PmCartesian * const end, int consume) {
    (void)radius;

    PmCartesian center, normal, binormal;

    // Pointer to middle point of line segment pair
    PmCartesian const * const middle = &line1->end;
    //TODO assert line1 end = line2 start?

    //Calculate the normal direction of the arc from the difference
    //between the unit vectors
    pmCartCartSub(&line2->uVec, &line1->uVec, &normal);
    pmCartUnitEq(&normal);
    pmCartScalMultEq(&normal, center_dist);
    pmCartCartAdd(middle, &normal, &center);

    //Calculate the binormal (vector perpendicular to the plane of the
    //arc)
    pmCartCartCross(&line1->uVec, &line2->uVec, &binormal);
    pmCartUnitEq(&binormal);

    // Start point is blend_dist away from middle point in the
    // negative direction of line1
    pmCartScalMult(&line1->uVec, -blend_dist, start);
    pmCartCartAdd(start, middle, start);

    // End point is blend_dist away from middle point in the positive
    // direction of line2
    pmCartScalMult(&line2->uVec, blend_dist, end);
    pmCartCartAddEq(end, middle);

    //Handle line portion of line-arc
    arc->uTan = line1->uVec;
    if (consume) {
        arc->line_length = line1->tmag - blend_dist;
    } else {
        arc->line_length = 0;
    }

    return arcInitFromPoints(arc, start, end, &center);
}

int arcConvexTest(PmCartesian const * const center,
        PmCartesian const * const P, PmCartesian const * const uVec, int reverse_dir)
{
    //Check if an arc-line intersection is concave or convex
    double dot;
    PmCartesian diff;
    pmCartCartSub(P, center, &diff);
    pmCartCartDot(&diff, uVec, &dot);

    tp_debug_print("convex test: dot = %f, reverse_dir = %d\n", dot, reverse_dir);
    int convex = (reverse_dir != 0) ^ (dot < 0);
    return convex;
}

int arcTangent(SphericalArc const * const arc, PmCartesian * const tan, int at_end)
{
    PmCartesian r_perp;
    PmCartesian r_tan;

    if (at_end) {
        r_perp = arc->rEnd;
    } else {
        r_perp = arc->rStart;
    }

    pmCartCartCross(&arc->binormal, &r_perp, &r_tan);
    //Get spiral component
    double dr = arc->spiral / arc->angle;

    //Get perpendicular component due to spiral
    PmCartesian d_perp;
    pmCartUnit(&r_perp, &d_perp);
    pmCartScalMultEq(&d_perp, dr);
    //TODO error checks
    pmCartCartAdd(&d_perp, &r_tan, tan);
    pmCartUnitEq(tan);

    return TP_ERR_OK;
}
