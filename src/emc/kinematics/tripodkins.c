/********************************************************************
* Description: tripodkins.c
*   Kinematics for 3 axis Tripod machine
*
*   Derived from a work by Fred Proctor
*
* Author: 
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

/*
  These kinematics are for a tripod with point vertices.

  Vertices A, B, and C are the base, and vertex D is the controlled point.
  Three tripod strut lengths AD, BD, and CD are the joints that move
  point D around.

  Point A is the origin, with coordinates (0, 0, 0). Point B lies on the
  x axis, with coordinates (Bx, 0, 0). Point C lies in the xy plane, with
  coordinates (Cx, Cy, 0). Point D has coordinates (Dx, Dy, Dz).

  The controlled Cartesian values are Dx, Dy, and Dz. A frame attached to
  D, say with x parallel to AD and y in the plane ABD, would change its
  orientation as the strut lengths changed. The orientation of this frame
  relative to the world frame is not computed.

  With respect to the kinematics functions,

  pos->tran.x = Dx
  pos->tran.y = Dy
  pos->tran.z = Dz
  pos->a,b,c  = 0

  joints[0] = AD
  joints[1] = BD
  joints[2] = CD

  The inverse kinematics have no singularities. Any values for Dx, Dy, and
  Dz will yield numerical results. Of course, these may be beyond the
  strut length limits, but there are no singular effects like infinite speed.

  The forward kinematics has a singularity due to the triangle inequalities
  for triangles ABD, BCD, and CAD. When any of these approach the limit,
  Dz is zero and D lies in the base plane.

  The forward kinematics flags, referred to in kinematicsForward and
  set in kinematicsInverse, let the forward kinematics select between
  the positive and negative values of Dz for given strut values.
  Dz > 0 is "above", Dz < 0 is "below". Dz = 0 is the singularity.

  fflags == 0 selects Dz > 0,
  fflags != 0 selects Dz < 0.

  The inverse kinematics flags let the inverse kinematics select between
  multiple valid solutions of strut lengths for given Cartesian values
  for D. There are no multiple solutions: D constrains the strut lengths
  completely. So, the inverse flags are ignored.
 */

#include "kinematics.h"             /* these decls */
#include "rtapi_math.h"

/* ident tag */
#ifndef __GNUC__
#ifndef __attribute__
#define __attribute__(x)
#endif
#endif

#include "hal.h"
struct haldata {
    hal_float_t *bx, *cx, *cy;
} *haldata = 0;

#define Bx (*(haldata->bx))
#define Cx (*(haldata->cx))
#define Cy (*(haldata->cy))

#define sq(x) ((x)*(x))

/*
  forward kinematics takes three strut lengths and computes Dx, Dy, and Dz
  pos->tran.x,y,z, respectively. The forward flag is used to resolve
  D above/below the xy plane. The inverse flags are not set since there
  are no ambiguities going from world to joint coordinates.

  The forward kins are derived as follows:

  1. Let x, y, z be Dx, Dy, Dz to save pixels. Cartesian displacement from
  D to A, B, and C gives

  AD^2 = x^2 + y^2 + z^2
  BD^2 = (x - Bx)^2 + y^2 + z^2
  CD^2 = (x - Cx)^2 + (y - Cy)^2 + z^2

  This yields

  I.   P = x^2 + y^2 + z^2
  II.  Q = x^2 + y^2 + z^2 + sx
  III. R = x^2 + y^2 + z^2 + tx + uy

  Where

  P = AD^2,
  Q = BD^2 - Bx^2
  R = CD^2 - Cx^2 - Cy^2
  s = -2Bx
  t = -2Cx
  u = -2Cy

  II - I gives Q - P = sx, so x = (Q - P)/s, s != 0. The constraint on s
  means that Bx != 0, or points A and B can't be the same.

  III - II gives R - Q = (t - s)x + uy, so y = (R - Q - (t - s)x)/u, u != 0.
  The constraint on u means that Cy != 0, or points A B C can't be collinear.

  Substituting x, y into I gives z = sqrt(P - x^2 - y^2), which has two
  solutions. Positive means the tripod is above the xy plane, negative
  means below.
*/
int kinematicsForward(const double * joints,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    (void)iflags;
#define AD (joints[0])
#define BD (joints[1])
#define CD (joints[2])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)
  double P, Q, R;
  double s, t, u;

  P = sq(AD);
  Q = sq(BD) - sq(Bx);
  R = sq(CD) - sq(Cx) - sq(Cy);
  s = -2.0 * Bx;
  t = -2.0 * Cx;
  u = -2.0 * Cy;

  if (s == 0.0) {
    /* points A and B coincident. Fix Bx, #defined up top. */
    return -1;
  }
  Dx = (Q - P) / s;

  if (u == 0.0) {
    /* points A B C are colinear. Fix Cy, #defined up top. */
    return -1;
  }
  Dy = (R - Q - (t - s) * Dx) / u;
  Dz = P - sq(Dx) - sq(Dy);
  if (Dz < 0.0) {
    /* triangle inequality violated */
    return -1;
  }
  Dz = sqrt(Dz);
  if (*fflags) {
    Dz = -Dz;
  }

  pos->a = 0.0;
  pos->b = 0.0;
  pos->c = 0.0;

  return 0;

#undef AD
#undef BD
#undef CD
#undef Dx
#undef Dy
#undef Dz
}

int kinematicsInverse(const EmcPose * pos,
                      double * joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)iflags;
#define AD (joints[0])
#define BD (joints[1])
#define CD (joints[2])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)

  AD = sqrt(sq(Dx) + sq(Dy) + sq(Dz));
  BD = sqrt(sq(Dx - Bx) + sq(Dy) + sq(Dz));
  CD = sqrt(sq(Dx - Cx) + sq(Dy - Cy) + sq(Dz));

  *fflags = 0;
  if (Dz < 0.0) {
    *fflags = 1;
  }

  return 0;

#undef AD
#undef BD
#undef CD
#undef Dx
#undef Dy
#undef Dz
}

KINEMATICS_TYPE kinematicsType()
{
  return KINEMATICS_BOTH;
}

#ifdef MAIN

#include <stdio.h>
#include <string.h>

/*
  Interactive testing of kins.

  Syntax: a.out <Bx> <Cx> <Cy>
*/
int main(int argc, char *argv[])
{
#ifndef BUFFERLEN
#define BUFFERLEN 256
#endif
  char buffer[BUFFERLEN];
  char cmd[BUFFERLEN];
  EmcPose pos, vel;
  double joints[3]={0.0,0.0,0.0}, jointvels[3]={0.0,0.0,0.0};
  char inverse;
  char flags;
  KINEMATICS_FORWARD_FLAGS fflags;

  inverse = 0;			/* forwards, by default */
  flags = 0;			/* didn't provide flags */
  fflags = 0;			/* above xy plane, by default */
  if (argc != 4 ||
      1 != sscanf(argv[1], "%lf", &Bx) ||
      1 != sscanf(argv[2], "%lf", &Cx) ||
      1 != sscanf(argv[3], "%lf", &Cy)) {
    fprintf(stderr, "syntax: %s Bx Cx Cy\n", argv[0]);
    return 1;
  }

  while (! feof(stdin)) {
    if (inverse) {
	printf("inv> ");
    }
    else {
	printf("fwd> ");
    }
    fflush(stdout);

    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }
    if (1 != sscanf(buffer, "%s", cmd)) {
      continue;
    }

    if (! strcmp(cmd, "quit")) {
      break;
    }
    if (! strcmp(cmd, "i")) {
      inverse = 1;
      continue;
    }
    if (! strcmp(cmd, "f")) {
      inverse = 0;
      continue;
    }
    if (! strcmp(cmd, "ff")) {
      if (1 != sscanf(buffer, "%*s %d", &fflags)) {
	printf("need forward flag\n");
      }
      continue;
    }

    if (inverse) {		/* inverse kins */
      if (3 != sscanf(buffer, "%lf %lf %lf", 
		      &pos.tran.x,
		      &pos.tran.y,
		      &pos.tran.z)) {
	printf("need X Y Z\n");
	continue;
      }
      if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	printf("inverse kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	  printf("forward kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	}
      }
    }
    else {			/* forward kins */
      if (flags) {
	if (4 != sscanf(buffer, "%lf %lf %lf %d", 
			&joints[0],
			&joints[1],
			&joints[2],
			&fflags)) {
	  printf("need 3 strut values and flag\n");
	  continue;
	}
      }
      else {
	if (3 != sscanf(buffer, "%lf %lf %lf", 
			&joints[0],
			&joints[1],
			&joints[2])) {
	  printf("need 3 strut values\n");
	  continue;
	}
      }
      if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	printf("forward kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	  printf("inverse kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	}
      }
    }
  } /* end while (! feof(stdin)) */

  return 0;
}

#endif /* MAIN */

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);

MODULE_LICENSE("GPL");



int comp_id;
int rtapi_app_main(void) {
    int res = 0;

    comp_id = hal_init("tripodkins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));
    if(!haldata) goto error;

    if((res = hal_pin_float_new("tripodkins.Bx", HAL_IO, &(haldata->bx), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("tripodkins.Cx", HAL_IO, &(haldata->cx), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("tripodkins.Cy", HAL_IO, &(haldata->cy), comp_id)) < 0) goto error;

    Bx = Cx = Cy = 1.0;
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
