/********************************************************************
* Description: genhexkins.c
*
*   Kinematics for a generalised hexapod machine
*
*   Derived from a work by R. Brian Register
*
* Adapting Author: Andrew Kyrychenko
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*********************************************************************

  These are the forward and inverse kinematic functions for a class of
  machines referred to as "Stewart Platforms".

  The functions are general enough to be configured for any platform
  configuration.  In the functions "genhexKinematicsForward" and
  "genhexKinematicsInverse" are arrays "a[i]" and "b[i]".  The values stored
  in these arrays correspond to the positions of the ends of the i'th
  strut. The value stored in a[i] is the position of the end of the i'th
  strut attached to the platform, in platform coordinates. The value
  stored in b[i] is the position of the end of the i'th strut attached
  to the base, in base (world) coordinates.

  The default values for base and platform joints positions are defined
  in the header file genhexkins.h.  The actual values for a particular
  machine can be adjusted by hal parameters:

  genhexkins.base.N.x
  genhexkins.base.N.y
  genhexkins.base.N.z - base joint coordinates.

  genhexkins.platform.N.x
  genhexkins.platform.N.y
  genhexkins.platform.N.z - platform joint coordinates.

  genhexkins.spindle-offset - added to Z coordinates of all joints to
                              change the machine origin. Facilitates
                              adjusting spindle position.

  genhexkins.tool-offset - tool length offset (TCP offset along Z),
                           implements RTCP function when connected to
                           motion.tooloffset.Z.

  To avoid joints jump change tool offset (G43, G49) only when the
  platform is not tilted (A = B = 0).

  Some hexapods use non-captive screw actuators and universal (cardanic)
  joints, thus the strut lengths depend on orientation of joints axes.
  Strut length correction is implemented to compensate for this.
  The calculations use orientation (unit vectors) of base and platform
  joint axes and the lead of actuator screws:

  genhexkins.base-n.N.x
  genhexkins.base-n.N.y
  genhexkins.base-n.N.z - unit vectors of base joint axes;

  genhexkins.platform-n.N.x
  genhexkins.platform-n.N.y
  genhexkins.platform-n.N.z - unit vectors of platform joint axes
                              in platform CS.
  genhexkins.screw-lead - lead of strut actuator screw, positive for
                          right-hand thread. Default is 0 (strut length
                          correction disabled).
  genhexkins.correction.N - pins showing current values of strut length
                            correction.

  The genhexKinematicsInverse function solves the inverse kinematics using
  a closed form algorithm.  The inverse kinematics problem is given
  the pose of the platform and returns the strut lengths. For this
  problem there is only one solution that is always returned correctly.

  The genhexKinematicsForward function solves the forward kinematics using
  an iterative algorithm.  Due to the iterative nature of this algorithm
  the genhexKinematicsForward function requires an initial value to begin the
  iterative routine and then converges to the "nearest" solution. The
  forward kinematics problem is given the strut lengths and returns the
  pose of the platform.  For this problem there arein multiple
  solutions.  The genhexKinematicsForward function will return only one of
  these solutions which will be the solution nearest to the initial
  value given.  It is possible that there are no solutions "near" the
  given initial value and the iteration will not converge and no
  solution will be returned.  Assuming there is a solution "near" the
  initial value, the function will always return one correct solution
  out of the multiple possible solutions.

  Hal pins to control and observe forward kinematics iterations:

  genhexkins.convergence-criterion - minimum error value that ends
                    iterations with converged solution;

  genhexkins.limit-iterations - limit of iterations, if exceeded
                    iterations stop with no convergence;

  genhexkins.max-error - maximum error value, if exceeded iterations
                    stop with no convergence;

  genhexkins.last-iterations - number of iterations spent for the
                    last forward kinematics solution;

  genhexkins.max-iterations - maximum number of iterations spent for
                    a converged solution during current session.

 ----------------------------------------------------------------------------*/

#include "rtapi.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "posemath.h"
#include "hal.h"
#include "genhexkins.h"
#include "motion.h"
#include "kinematics.h"             /* these decls, KINEMATICS_FORWARD_FLAGS */
#include "switchkins.h"

static struct haldata {
    hal_float_t *basex[NUM_STRUTS];
    hal_float_t *basey[NUM_STRUTS];
    hal_float_t *basez[NUM_STRUTS];
    hal_float_t *platformx[NUM_STRUTS];
    hal_float_t *platformy[NUM_STRUTS];
    hal_float_t *platformz[NUM_STRUTS];
    hal_float_t *basenx[NUM_STRUTS];
    hal_float_t *baseny[NUM_STRUTS];
    hal_float_t *basenz[NUM_STRUTS];
    hal_float_t *platformnx[NUM_STRUTS];
    hal_float_t *platformny[NUM_STRUTS];
    hal_float_t *platformnz[NUM_STRUTS];
    hal_float_t *correction[NUM_STRUTS];
    hal_float_t *screw_lead;
    hal_u32_t   *last_iter;
    hal_u32_t   *max_iter;
    hal_u32_t   *iter_limit;
    hal_float_t *max_error;
    hal_float_t *conv_criterion;
    hal_float_t *tool_offset;
    hal_float_t *spindle_offset;
    hal_bit_t   *fwd_kins_fail;

    hal_float_t *gui_x;
    hal_float_t *gui_y;
    hal_float_t *gui_z;
    hal_float_t *gui_a;
    hal_float_t *gui_b;
    hal_float_t *gui_c;

} *haldata;

static int genhex_gui_forward_kins(EmcPose *pos)
{
    *haldata->gui_x = pos->tran.x;
    *haldata->gui_y = pos->tran.y;
    *haldata->gui_z = pos->tran.z;
    *haldata->gui_a = pos->a;
    *haldata->gui_b = pos->b;
    *haldata->gui_c = pos->c;
    return 0;
} // genhex_gui_forward_kins

/******************************* MatInvert() ***************************/

/*-----------------------------------------------------------------------------
 This is a function that inverts a 6x6 matrix.
-----------------------------------------------------------------------------*/

static int MatInvert(double J[][NUM_STRUTS], double InvJ[][NUM_STRUTS])
{
  double JAug[NUM_STRUTS][12], m, temp;
  int j, k, n;

  /* This function determines the inverse of a 6x6 matrix using
     Gauss-Jordan elimination */

  /* Augment the Identity matrix to the Jacobian matrix */

  for (j=0; j<=5; ++j){
    for (k=0; k<=5; ++k){     /* Assign J matrix to first 6 columns of AugJ */
      JAug[j][k] = J[j][k];
    }
    for(k=6; k<=11; ++k){    /* Assign I matrix to last six columns of AugJ */
      if (k-6 == j){
        JAug[j][k]=1;
      }
      else{
        JAug[j][k]=0;
      }
    }
  }

  /* Perform Gauss elimination */
  for (k=0; k<=4; ++k){               /* Pivot        */
    if ((JAug[k][k]< 0.01) && (JAug[k][k] > -0.01)){
      for (j=k+1;j<=5; ++j){
        if ((JAug[j][k]>0.01) || (JAug[j][k]<-0.01)){
          for (n=0; n<=11;++n){
            temp = JAug[k][n];
            JAug[k][n] = JAug[j][n];
            JAug[j][n] = temp;
          }
          break;
        }
      }
    }
    for (j=k+1; j<=5; ++j){            /* Pivot */
      m = -JAug[j][k] / JAug[k][k];
      for (n=0; n<=11; ++n){
        JAug[j][n]=JAug[j][n] + m*JAug[k][n];   /* (Row j) + m * (Row k) */
        if ((JAug[j][n] < 0.000001) && (JAug[j][n] > -0.000001)){
          JAug[j][n] = 0;
        }
      }
    }
  }

  /* Normalization of Diagonal Terms */
  for (j=0; j<=5; ++j){
    m=1/JAug[j][j];
    for(k=0; k<=11; ++k){
      JAug[j][k] = m * JAug[j][k];
    }
  }

  /* Perform Gauss Jordan Steps */
  for (k=5; k>=0; --k){
    for(j=k-1; j>=0; --j){
      m = -JAug[j][k]/JAug[k][k];
      for (n=0; n<=11; ++n){
        JAug[j][n] = JAug[j][n] + m * JAug[k][n];
      }
    }
  }

  /* Assign last 6 columns of JAug to InvJ */
  for (j=0; j<=5; ++j){
    for (k=0; k<=5; ++k){
      InvJ[j][k] = JAug[j][k+6];

    }
  }

  return 0;         /* FIXME-- check divisors for 0 above */
} // MatInvert()

/******************************** MatMult() *********************************/

/*---------------------------------------------------------------------------
  This function simply multiplies a 6x6 matrix by a 1x6 vector
  ---------------------------------------------------------------------------*/

static void MatMult(double J[][6], const double x[], double Ans[])
{
  int j, k;
  for (j=0; j<=5; ++j){
    Ans[j] = 0;
    for (k=0; k<=5; ++k){
      Ans[j] = J[j][k]*x[k]+Ans[j];
    }
  }
} // MatMult()

/* declare arrays for base and platform coordinates */
static PmCartesian b[NUM_STRUTS];
static PmCartesian a[NUM_STRUTS];

/* declare base and platform joint axes vectors */

static PmCartesian nb1[NUM_STRUTS];
static PmCartesian na0[NUM_STRUTS];

/************************genhex_read_hal_pins**************************/

static int genhex_read_hal_pins(void) {
    int t;

  /* set the base and platform coordinates from hal pin values */
    for (t = 0; t < NUM_STRUTS; t++) {
        b[t].x   = *haldata->basex[t];
        b[t].y   = *haldata->basey[t];
        b[t].z   = *haldata->basez[t] + *haldata->spindle_offset + *haldata->tool_offset;
        a[t].x   = *haldata->platformx[t];
        a[t].y   = *haldata->platformy[t];
        a[t].z   = *haldata->platformz[t] + *haldata->spindle_offset + *haldata->tool_offset;

        nb1[t].x = *haldata->basenx[t];
        nb1[t].y = *haldata->baseny[t];
        nb1[t].z = *haldata->basenz[t];
        na0[t].x = *haldata->platformnx[t];
        na0[t].y = *haldata->platformny[t];
        na0[t].z = *haldata->platformnz[t];

    }
    return 0;
} // genhex_read_hal_pins()

/***************************StrutLengthCorrection***************************/

static int StrutLengthCorrection(const PmCartesian * StrutVectUnit,
                                 const PmRotationMatrix * RMatrix,
                                 const int strut_number,
                                 double * correction)
{
  PmCartesian nb2, nb3, na1, na2;
  double dotprod;

  /* define base joints axis vectors */
  pmCartCartCross(&nb1[strut_number], StrutVectUnit, &nb2);
  pmCartCartCross(StrutVectUnit, &nb2, &nb3);
  pmCartUnitEq(&nb3);

  /* define platform joints axis vectors */
  pmMatCartMult(RMatrix, &na0[strut_number], &na1);
  pmCartCartCross(&na1, StrutVectUnit, &na2);
  pmCartUnitEq(&na2);

  /* define dot product */
  pmCartCartDot(&nb3, &na2, &dotprod);

  *correction = *haldata->screw_lead * asin(dotprod) / PM_2_PI;

  return 0;
} // StrutLengthCorrection()


/**************** genhexKinematicsForward() *****************/
static int genhexKinematicsForward(const double * joints,
                                   EmcPose * pos,
                                   const KINEMATICS_FORWARD_FLAGS * fflags,
                                   KINEMATICS_INVERSE_FLAGS * iflags)
{
  (void)fflags;
  (void)iflags;
  PmCartesian aw;
  PmCartesian InvKinStrutVect,InvKinStrutVectUnit;
  PmCartesian q_trans, RMatrix_a, RMatrix_a_cross_Strut;

  double Jacobian[NUM_STRUTS][NUM_STRUTS];
  double InverseJacobian[NUM_STRUTS][NUM_STRUTS];
  double InvKinStrutLength, StrutLengthDiff[NUM_STRUTS];
  double delta[NUM_STRUTS];
  double conv_err = 1.0;
  double corr;

  PmRotationMatrix RMatrix;
  PmRpy q_RPY;

  int iterate = 1;
  int i;
  unsigned iteration = 0;

  genhex_read_hal_pins();

  /* abort on obvious problems, like joints <= 0 */
  /* FIXME-- should check against triangle inequality, so that joints
     are never too short to span shared base and platform sides */
  if (joints[0] <= 0.0 ||
      joints[1] <= 0.0 ||
      joints[2] <= 0.0 ||
      joints[3] <= 0.0 ||
      joints[4] <= 0.0 ||
      joints[5] <= 0.0) {
      return -1;
  }

  /* assign a,b,c to roll, pitch, yaw angles */
  q_RPY.r = pos->a * PM_PI / 180.0;
  q_RPY.p = pos->b * PM_PI / 180.0;
  q_RPY.y = pos->c * PM_PI / 180.0;

  /* Assign translation values in pos to q_trans */
  q_trans.x = pos->tran.x;
  q_trans.y = pos->tran.y;
  q_trans.z = pos->tran.z;

  /* Enter Newton-Raphson iterative method   */
  while (iterate) {
    /* check for large error and return error flag if no convergence */
    if ((conv_err > +(*haldata->max_error)) ||
        (conv_err < -(*haldata->max_error))) {
      /* we can't converge */
      *haldata->fwd_kins_fail = 1;
      return -2;
    };

    iteration++;

    /* check iteration to see if the kinematics can reach the
       convergence criterion and return error flag if it can't */
    if (iteration > *haldata->iter_limit) {
      /* we can't converge */
      *haldata->fwd_kins_fail = 1;
      return -5;
    }

    /* Convert q_RPY to Rotation Matrix */
    pmRpyMatConvert(&q_RPY, &RMatrix);

    /* compute StrutLengthDiff[] by running inverse kins on Cartesian
     estimate to get joint estimate, subtract joints to get joint deltas,
     and compute inv J while we're at it */
    for (i = 0; i < NUM_STRUTS; i++) {
      pmMatCartMult(&RMatrix, &a[i], &RMatrix_a);
      pmCartCartAdd(&q_trans, &RMatrix_a, &aw);
      pmCartCartSub(&aw, &b[i], &InvKinStrutVect);
      if (0 != pmCartUnit(&InvKinStrutVect, &InvKinStrutVectUnit)) {
        *haldata->fwd_kins_fail = 1;
        return -1;
      }
      pmCartMag(&InvKinStrutVect, &InvKinStrutLength);

      if (*haldata->screw_lead != 0.0) {
        /* enable strut length correction */
        StrutLengthCorrection(&InvKinStrutVectUnit, &RMatrix, i, &corr);
        /* define corrected joint lengths */
        InvKinStrutLength += corr;
      }

      StrutLengthDiff[i] = InvKinStrutLength - joints[i];

      /* Determine RMatrix_a_cross_strut */
      pmCartCartCross(&RMatrix_a, &InvKinStrutVectUnit, &RMatrix_a_cross_Strut);

      /* Build Inverse Jacobian Matrix */
      InverseJacobian[i][0] = InvKinStrutVectUnit.x;
      InverseJacobian[i][1] = InvKinStrutVectUnit.y;
      InverseJacobian[i][2] = InvKinStrutVectUnit.z;
      InverseJacobian[i][3] = RMatrix_a_cross_Strut.x;
      InverseJacobian[i][4] = RMatrix_a_cross_Strut.y;
      InverseJacobian[i][5] = RMatrix_a_cross_Strut.z;
    }

    /* invert Inverse Jacobian */
    MatInvert(InverseJacobian, Jacobian);

    /* multiply Jacobian by LegLengthDiff */
    MatMult(Jacobian, StrutLengthDiff, delta);

    /* subtract delta from last iterations pos values */
    q_trans.x -= delta[0];
    q_trans.y -= delta[1];
    q_trans.z -= delta[2];
    q_RPY.r   -= delta[3];
    q_RPY.p   -= delta[4];
    q_RPY.y   -= delta[5];

    /* determine value of conv_error (used to determine if no convergence) */
    conv_err = 0.0;
    for (i = 0; i < NUM_STRUTS; i++) {
      conv_err += fabs(StrutLengthDiff[i]);
    }

    /* enter loop to determine if a strut needs another iteration */
    iterate = 0;            /*assume iteration is done */
    for (i = 0; i < NUM_STRUTS; i++) {
      if (fabs(StrutLengthDiff[i]) > *haldata->conv_criterion) {
    iterate = 1;
      }
    }
  } /* exit Newton-Raphson Iterative loop */

  /* assign r,p,y to a,b,c */
  pos->a = q_RPY.r * 180.0 / PM_PI;
  pos->b = q_RPY.p * 180.0 / PM_PI;
  pos->c = q_RPY.y * 180.0 / PM_PI;

  /* assign q_trans to pos */
  pos->tran.x = q_trans.x;
  pos->tran.y = q_trans.y;
  pos->tran.z = q_trans.z;

  *haldata->last_iter = iteration;

  if (iteration > *haldata->max_iter){
    *haldata->max_iter = iteration;
  }
  *haldata->fwd_kins_fail = 0;

  genhex_gui_forward_kins(pos);

  return 0;
} // genhexKinematicsForward()


/************************ genhexKinematicsInverse() ************************/
/* the inverse kinematics take world coordinates and determine joint values,
   given the inverse kinematics flags to resolve any ambiguities. The forward
   flags are set to indicate their value appropriate to the world coordinates
   passed in. */

static int genhexKinematicsInverse(const EmcPose * pos,
                                   double * joints,
                                   const KINEMATICS_INVERSE_FLAGS * iflags,
                                   KINEMATICS_FORWARD_FLAGS * fflags)
{
  (void)iflags;
  (void)fflags;

  PmCartesian aw, temp;
  PmCartesian InvKinStrutVect, InvKinStrutVectUnit;
  PmRotationMatrix RMatrix;
  PmRpy rpy;
  int i;
  double InvKinStrutLength, corr;

  genhex_read_hal_pins();

  /* define Rotation Matrix */
  rpy.r = pos->a * PM_PI / 180.0;
  rpy.p = pos->b * PM_PI / 180.0;
  rpy.y = pos->c * PM_PI / 180.0;
  pmRpyMatConvert(&rpy, &RMatrix);

  /* enter for loop to calculate joints (strut lengths) */
  for (i = 0; i < NUM_STRUTS; i++) {
    /* convert location of platform strut end from platform
       to world coordinates */
    pmMatCartMult(&RMatrix, &a[i], &temp);
    pmCartCartAdd(&pos->tran, &temp, &aw);

    /* define strut lengths */
    pmCartCartSub(&aw, &b[i], &InvKinStrutVect);
    pmCartMag(&InvKinStrutVect, &InvKinStrutLength);

    if (*haldata->screw_lead != 0.0) {
      /* enable strut length correction */
      /* define unit strut vector */
      if (0 != pmCartUnit(&InvKinStrutVect, &InvKinStrutVectUnit)) {
          return -1;
      }
      /* define correction value and corrected joint lengths */
      StrutLengthCorrection(&InvKinStrutVectUnit, &RMatrix, i, &corr);
      *haldata->correction[i] = corr;
      InvKinStrutLength += corr;
    }

    joints[i] = InvKinStrutLength;
  }

  return 0;
} //genhexKinematicsInverse()

static
int genhexKinematicsSetup(const  int   comp_id,
                          const  char* coordinates,
                          kparms*      kp)
{
    (void)coordinates;
    int i,res=0;

    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) {
        rtapi_print_msg(RTAPI_MSG_ERR,"genhexKinematicsSetup: hal_malloc fail\n");
        return -1;
    }

    for (i = 0; i < kp->max_joints; i++) {
        res += hal_pin_float_newf(HAL_IN, &(haldata->basex[i]), comp_id,
            "%s.base.%d.x", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->basey[i], comp_id,
            "%s.base.%d.y", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->basez[i], comp_id,
            "%s.base.%d.z", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformx[i], comp_id,
            "%s.platform.%d.x", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformy[i], comp_id,
            "%s.platform.%d.y", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformz[i], comp_id,
            "%s.platform.%d.z", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->basenx[i], comp_id,
            "%s.base-n.%d.x", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->baseny[i], comp_id,
            "%s.base-n.%d.y", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->basenz[i], comp_id,
            "%s.base-n.%d.z", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformnx[i], comp_id,
            "%s.platform-n.%d.x", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformny[i], comp_id,
            "%s.platform-n.%d.y", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_IN, &haldata->platformnz[i], comp_id,
            "%s.platform-n.%d.z", kp->halprefix, i);
        res += hal_pin_float_newf(HAL_OUT, &haldata->correction[i], comp_id,
            "%s.correction.%d", kp->halprefix, i);
        if (res) {goto error;}
        *haldata->correction[i] = 0.0;

    }

    res += hal_pin_u32_newf(HAL_OUT, &haldata->last_iter, comp_id,
        "genhexkins.last-iterations");
    *haldata->last_iter = 0;
    res += hal_pin_u32_newf(HAL_OUT, &haldata->max_iter, comp_id,
        "genhexkins.max-iterations");
    *haldata->max_iter = 0;
    res += hal_pin_float_newf(HAL_IN, &haldata->max_error, comp_id,
        "genhexkins.max-error");
    *haldata->max_error = 500.0;
    res += hal_pin_float_newf(HAL_IN, &haldata->conv_criterion, comp_id,
        "genhexkins.convergence-criterion");
    *haldata->conv_criterion = 1e-9;
    res += hal_pin_u32_newf(HAL_IN, &haldata->iter_limit, comp_id,
        "genhexkins.limit-iterations");
    *haldata->iter_limit = 120;
    res += hal_pin_float_newf(HAL_IN, &haldata->tool_offset, comp_id,
        "genhexkins.tool-offset");
    *haldata->tool_offset = 0.0;
    res += hal_pin_float_newf(HAL_IN, &haldata->spindle_offset, comp_id,
        "genhexkins.spindle-offset");
    *haldata->spindle_offset = 0.0;
    res += hal_pin_float_newf(HAL_IN, &haldata->screw_lead, comp_id,
        "genhexkins.screw-lead");
    *haldata->screw_lead = DEFAULT_SCREW_LEAD;

    if (res) {goto error;}

    *haldata->basex[0] = DEFAULT_BASE_0_X;
    *haldata->basey[0] = DEFAULT_BASE_0_Y;
    *haldata->basez[0] = DEFAULT_BASE_0_Z;
    *haldata->basex[1] = DEFAULT_BASE_1_X;
    *haldata->basey[1] = DEFAULT_BASE_1_Y;
    *haldata->basez[1] = DEFAULT_BASE_1_Z;
    *haldata->basex[2] = DEFAULT_BASE_2_X;
    *haldata->basey[2] = DEFAULT_BASE_2_Y;
    *haldata->basez[2] = DEFAULT_BASE_2_Z;
    *haldata->basex[3] = DEFAULT_BASE_3_X;
    *haldata->basey[3] = DEFAULT_BASE_3_Y;
    *haldata->basez[3] = DEFAULT_BASE_3_Z;
    *haldata->basex[4] = DEFAULT_BASE_4_X;
    *haldata->basey[4] = DEFAULT_BASE_4_Y;
    *haldata->basez[4] = DEFAULT_BASE_4_Z;
    *haldata->basex[5] = DEFAULT_BASE_5_X;
    *haldata->basey[5] = DEFAULT_BASE_5_Y;
    *haldata->basez[5] = DEFAULT_BASE_5_Z;

    *haldata->platformx[0] = DEFAULT_PLATFORM_0_X;
    *haldata->platformy[0] = DEFAULT_PLATFORM_0_Y;
    *haldata->platformz[0] = DEFAULT_PLATFORM_0_Z;
    *haldata->platformx[1] = DEFAULT_PLATFORM_1_X;
    *haldata->platformy[1] = DEFAULT_PLATFORM_1_Y;
    *haldata->platformz[1] = DEFAULT_PLATFORM_1_Z;
    *haldata->platformx[2] = DEFAULT_PLATFORM_2_X;
    *haldata->platformy[2] = DEFAULT_PLATFORM_2_Y;
    *haldata->platformz[2] = DEFAULT_PLATFORM_2_Z;
    *haldata->platformx[3] = DEFAULT_PLATFORM_3_X;
    *haldata->platformy[3] = DEFAULT_PLATFORM_3_Y;
    *haldata->platformz[3] = DEFAULT_PLATFORM_3_Z;
    *haldata->platformx[4] = DEFAULT_PLATFORM_4_X;
    *haldata->platformy[4] = DEFAULT_PLATFORM_4_Y;
    *haldata->platformz[4] = DEFAULT_PLATFORM_4_Z;
    *haldata->platformx[5] = DEFAULT_PLATFORM_5_X;
    *haldata->platformy[5] = DEFAULT_PLATFORM_5_Y;
    *haldata->platformz[5] = DEFAULT_PLATFORM_5_Z;

    *haldata->basenx[0] = DEFAULT_BASE_0_NX;
    *haldata->baseny[0] = DEFAULT_BASE_0_NY;
    *haldata->basenz[0] = DEFAULT_BASE_0_NZ;
    *haldata->basenx[1] = DEFAULT_BASE_1_NX;
    *haldata->baseny[1] = DEFAULT_BASE_1_NY;
    *haldata->basenz[1] = DEFAULT_BASE_1_NZ;
    *haldata->basenx[2] = DEFAULT_BASE_2_NX;
    *haldata->baseny[2] = DEFAULT_BASE_2_NY;
    *haldata->basenz[2] = DEFAULT_BASE_2_NZ;
    *haldata->basenx[3] = DEFAULT_BASE_3_NX;
    *haldata->baseny[3] = DEFAULT_BASE_3_NY;
    *haldata->basenz[3] = DEFAULT_BASE_3_NZ;
    *haldata->basenx[4] = DEFAULT_BASE_4_NX;
    *haldata->baseny[4] = DEFAULT_BASE_4_NY;
    *haldata->basenz[4] = DEFAULT_BASE_4_NZ;
    *haldata->basenx[5] = DEFAULT_BASE_5_NX;
    *haldata->baseny[5] = DEFAULT_BASE_5_NY;
    *haldata->basenz[5] = DEFAULT_BASE_5_NZ;

    *haldata->platformnx[0] = DEFAULT_PLATFORM_0_NX;
    *haldata->platformny[0] = DEFAULT_PLATFORM_0_NY;
    *haldata->platformnz[0] = DEFAULT_PLATFORM_0_NZ;
    *haldata->platformnx[1] = DEFAULT_PLATFORM_1_NX;
    *haldata->platformny[1] = DEFAULT_PLATFORM_1_NY;
    *haldata->platformnz[1] = DEFAULT_PLATFORM_1_NZ;
    *haldata->platformnx[2] = DEFAULT_PLATFORM_2_NX;
    *haldata->platformny[2] = DEFAULT_PLATFORM_2_NY;
    *haldata->platformnz[2] = DEFAULT_PLATFORM_2_NZ;
    *haldata->platformnx[3] = DEFAULT_PLATFORM_3_NX;
    *haldata->platformny[3] = DEFAULT_PLATFORM_3_NY;
    *haldata->platformnz[3] = DEFAULT_PLATFORM_3_NZ;
    *haldata->platformnx[4] = DEFAULT_PLATFORM_4_NX;
    *haldata->platformny[4] = DEFAULT_PLATFORM_4_NY;
    *haldata->platformnz[4] = DEFAULT_PLATFORM_4_NZ;
    *haldata->platformnx[5] = DEFAULT_PLATFORM_5_NX;
    *haldata->platformny[5] = DEFAULT_PLATFORM_5_NY;
    *haldata->platformnz[5] = DEFAULT_PLATFORM_5_NZ;

    //note: switchkins does not uses these as it provides gui.x, gui.y, etc.
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_x, comp_id, "genhexkins.x");
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_y, comp_id, "genhexkins.y");
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_z, comp_id, "genhexkins.z");
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_a, comp_id, "genhexkins.a");
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_b, comp_id, "genhexkins.b");
    res += hal_pin_float_newf(HAL_IN, &haldata->gui_c, comp_id, "genhexkins.c");

    res += hal_pin_bit_newf(HAL_OUT, &haldata->fwd_kins_fail, comp_id,
        "genhexkins.fwd-kins-fail");

    if (res) goto error;
    return 0;

error:
    return res;
} // genhexKinematicsSetup()

int switchkinsSetup(kparms* kp,
                    KS* kset0, KS* kset1, KS* kset2,
                    KF* kfwd0, KF* kfwd1, KF* kfwd2,
                    KI* kinv0, KI* kinv1, KI* kinv2
                   )
{
    kp->kinsname    = "genhexkins"; // !!! must agree with filename
    kp->halprefix   = "genhexkins"; // hal pin names
    kp->required_coordinates = "xyzabc";
    kp->max_joints  = strlen(kp->required_coordinates);
    kp->allow_duplicates  = 0;
    kp->fwd_iterates_mask = 0x1; //genhexkins switchkins_type==0
    kp->gui_kinstype      = 0;   //vismach gui for switchkins_type==0

    // switchkins_type==0 is startup default
    // kins with iterative forward algorithm should be switchkins_type==0
    *kset0 = genhexKinematicsSetup;
    *kfwd0 = genhexKinematicsForward;
    *kinv0 = genhexKinematicsInverse;

    *kset1 = identityKinematicsSetup;
    *kfwd1 = identityKinematicsForward;
    *kinv1 = identityKinematicsInverse;

    *kset2 = userkKinematicsSetup;
    *kfwd2 = userkKinematicsForward;
    *kinv2 = userkKinematicsInverse;

    return 0;
} //switchkinsSetup()
