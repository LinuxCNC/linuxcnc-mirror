/*****************************************************************
* Description: pumakins.c
*   Kinematics for puma typed robots
*   Set the params using HAL to fit your robot
*
*   Derived from a work by Fred Proctor
*
*   modified by rdp to add effect of D6 parameter (see pumagui)
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
*******************************************************************
*/
#include "pumakins.h"
#include "posemath.h"
#include "rtapi.h"
#include "rtapi_math.h"
#include "rtapi_string.h"
#include "hal.h"
#include "kinematics.h"
#include "switchkins.h"

struct haldata {
    hal_float_t *a2, *a3, *d3, *d4, *d6;
} *haldata = 0;

#define PUMA_A2 (*(haldata->a2))
#define PUMA_A3 (*(haldata->a3))
#define PUMA_D3 (*(haldata->d3))
#define PUMA_D4 (*(haldata->d4))
#define PUMA_D6 (*(haldata->d6))

static int pumaKinematicsForward(const double * joint,
                                 EmcPose * world,
                                 const KINEMATICS_FORWARD_FLAGS * fflags,
                                 KINEMATICS_INVERSE_FLAGS * iflags)
{
   (void)fflags;
   double s1, s2, s3, s4, s5, s6;
   double c1, c2, c3, c4, c5, c6;
   double s23;
   double c23;
   double t1, t2, t3, t4, t5;
   double sumSq, k;
   PmHomogeneous hom;
   PmPose worldPose;
   PmRpy rpy;

   /* Calculate sin of joints for future use */
   s1 = sin(joint[0]*PM_PI/180);
   s2 = sin(joint[1]*PM_PI/180);
   s3 = sin(joint[2]*PM_PI/180);
   s4 = sin(joint[3]*PM_PI/180);
   s5 = sin(joint[4]*PM_PI/180);
   s6 = sin(joint[5]*PM_PI/180);

   /* Calculate cos of joints for future use */
   c1 = cos(joint[0]*PM_PI/180);
   c2 = cos(joint[1]*PM_PI/180);
   c3 = cos(joint[2]*PM_PI/180);
   c4 = cos(joint[3]*PM_PI/180);
   c5 = cos(joint[4]*PM_PI/180);
   c6 = cos(joint[5]*PM_PI/180);

   s23 = c2 * s3 + s2 * c3;
   c23 = c2 * c3 - s2 * s3;

   /* Calculate terms to be used in definition of... */
   /* first column of rotation matrix.               */
   t1 = c4 * c5 * c6 - s4 * s6;
   t2 = s23 * s5 * c6;
   t3 = s4 * c5 * c6 + c4 * s6;
   t4 = c23 * t1 - t2;
   t5 = c23 * s5 * c6;

   /* Define first column of rotation matrix */
   hom.rot.x.x = c1 * t4 + s1 * t3;
   hom.rot.x.y = s1 * t4 - c1 * t3;
   hom.rot.x.z = -s23 * t1 - t5;

   /* Calculate terms to be used in definition of...  */
   /* second column of rotation matrix.               */
   t1 = -c4 * c5 * s6 - s4 * c6;
   t2 = s23 * s5 * s6;
   t3 = c4 * c6 - s4 * c5 * s6;
   t4 = c23 * t1 + t2;
   t5 = c23 * s5 * s6;

   /* Define second column of rotation matrix */
   hom.rot.y.x = c1 * t4 + s1 * t3;
   hom.rot.y.y = s1 * t4 - c1 * t3;
   hom.rot.y.z = -s23 * t1 + t5;

   /* Calculate term to be used in definition of... */
   /* third column of rotation matrix.              */
   t1 = c23 * c4 * s5 + s23 * c5;

   /* Define third column of rotation matrix */
   hom.rot.z.x = -c1 * t1 - s1 * s4 * s5;
   hom.rot.z.y = -s1 * t1 + c1 * s4 * s5;
   hom.rot.z.z = s23 * c4 * s5 - c23 * c5;

   /* Calculate term to be used in definition of...  */
   /* position vector.                               */
   t1 = PUMA_A2 * c2 + PUMA_A3 * c23 - PUMA_D4 * s23;

   /* Define position vector */
   hom.tran.x = c1 * t1 - PUMA_D3 * s1;
   hom.tran.y = s1 * t1 + PUMA_D3 * c1;
   hom.tran.z = -PUMA_A3 * s23 - PUMA_A2 * s2 - PUMA_D4 * c23;

   /* Calculate terms to be used to...   */
   /* determine flags.                   */
   sumSq = hom.tran.x * hom.tran.x + hom.tran.y * hom.tran.y -
           PUMA_D3 * PUMA_D3;
   k = (sumSq + hom.tran.z * hom.tran.z - PUMA_A2 * PUMA_A2 -
       PUMA_A3 * PUMA_A3 - PUMA_D4 * PUMA_D4) /
       (2.0 * PUMA_A2);

   /* reset flags */
   *iflags = 0;

   /* Set shoulder-up flag if necessary */
   if (fabs(joint[0]*PM_PI/180 - atan2(hom.tran.y, hom.tran.x) +
       atan2(PUMA_D3, -sqrt(sumSq))) < FLAG_FUZZ)
   {
     *iflags |= PUMA_SHOULDER_RIGHT;
   }

   /* Set elbow down flag if necessary */
   if (fabs(joint[2]*PM_PI/180 - atan2(PUMA_A3, PUMA_D4) +
       atan2(k, -sqrt(PUMA_A3 * PUMA_A3 +
       PUMA_D4 * PUMA_D4 - k * k))) < FLAG_FUZZ)
   {
      *iflags |= PUMA_ELBOW_DOWN;
   }

   /* set singular flag if necessary */
   t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
   t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 +
         hom.rot.z.z * s23;
   if (fabs(t1) < SINGULAR_FUZZ && fabs(t2) < SINGULAR_FUZZ)
   {
      *iflags |= PUMA_SINGULAR;
   }

   /* if not singular set wrist flip flag if necessary */
   else{
     if (! (fabs(joint[3]*PM_PI/180 - atan2(t1, t2)) < FLAG_FUZZ))
     {
       *iflags |= PUMA_WRIST_FLIP;
     }
   }
  /*  add effect of d6 parameter */
    hom.tran.x = hom.tran.x + hom.rot.z.x*PUMA_D6;
    hom.tran.y = hom.tran.y + hom.rot.z.y*PUMA_D6;
    hom.tran.z = hom.tran.z + hom.rot.z.z*PUMA_D6;

   /* convert hom.rot to world->quat */
   pmHomPoseConvert(&hom, &worldPose);
   pmQuatRpyConvert(&worldPose.rot,&rpy);
   world->tran = worldPose.tran;
   world->a = rpy.r * 180.0/PM_PI;
   world->b = rpy.p * 180.0/PM_PI;
   world->c = rpy.y * 180.0/PM_PI;


   /* return 0 and exit */
   return 0;
}

static int pumaKinematicsInverse(const EmcPose * world,
                                 double * joint,
                                 const KINEMATICS_INVERSE_FLAGS * iflags,
                                 KINEMATICS_FORWARD_FLAGS * fflags)
{
   PmHomogeneous hom;
   PmPose worldPose;
   PmRpy rpy;

   double t1, t2, t3;
   double k;
   double sumSq;

   double th1;
   double th3;
   double th23;
   double th2;
   double th4;
   double th5;
   double th6;

   double s1, c1;
   double s3, c3;
   double s23, c23;
   double s4, c4;
   double s5, c5;
   double s6, c6;
   double px, py, pz;

   /* reset flags */
   *fflags = 0;

   /* convert pose to hom */
   worldPose.tran = world->tran;
   rpy.r = world->a*PM_PI/180.0;
   rpy.p = world->b*PM_PI/180.0;
   rpy.y = world->c*PM_PI/180.0;
   pmRpyQuatConvert(&rpy,&worldPose.rot);
   pmPoseHomConvert(&worldPose, &hom);

  /* remove effect of d6 parameter */
   px = hom.tran.x - PUMA_D6*hom.rot.z.x;
   py = hom.tran.y - PUMA_D6*hom.rot.z.y;
   pz = hom.tran.z - PUMA_D6*hom.rot.z.z;

   /* Joint 1 (2 independent solutions) */

   /* save sum of squares for this and subsequent calcs */
   sumSq = px * px + py * py -
           PUMA_D3 * PUMA_D3;

   /* FIXME-- is use of + sqrt shoulder right or left? */
   if (*iflags & PUMA_SHOULDER_RIGHT){
     th1 = atan2(py, px) - atan2(PUMA_D3, -sqrt(sumSq));
   }
   else{
     th1 = atan2(py, px) - atan2(PUMA_D3, sqrt(sumSq));
   }

   /* save sin, cos for later calcs */
   s1 = sin(th1);
   c1 = cos(th1);

   /* Joint 3 (2 independent solutions) */

   k = (sumSq + pz * pz - PUMA_A2 * PUMA_A2 -
       PUMA_A3 * PUMA_A3 - PUMA_D4 * PUMA_D4) / (2.0 * PUMA_A2);

   /* FIXME-- is use of + sqrt elbow up or down? */
   if (*iflags & PUMA_ELBOW_DOWN){
     th3 = atan2(PUMA_A3, PUMA_D4) - atan2(k, -sqrt(PUMA_A3 * PUMA_A3 + PUMA_D4 * PUMA_D4 - k * k));
   }
   else{
     th3 = atan2(PUMA_A3, PUMA_D4) -
           atan2(k, sqrt(PUMA_A3 * PUMA_A3 + PUMA_D4 * PUMA_D4 - k * k));
   }

   /* compute sin, cos for later calcs */
   s3 = sin(th3);
   c3 = cos(th3);

   /* Joint 2 */

   t1 = (-PUMA_A3 - PUMA_A2 * c3) * pz +
        (c1 * px + s1 * py) * (PUMA_A2 * s3 - PUMA_D4);
   t2 = (PUMA_A2 * s3 - PUMA_D4) * pz +
        (PUMA_A3 + PUMA_A2 * c3) * (c1 * px + s1 * py);
   t3 = pz * pz + (c1 * px + s1 * py) *
        (c1 * px + s1 * py);

   th23 = atan2(t1, t2);
   th2 = th23 - th3;

   /* compute sin, cos for later calcs */
   s23 = t1 / t3;
   c23 = t2 / t3;

   /* Joint 4 */

   t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
   t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 + hom.rot.z.z * s23;
   if (fabs(t1) < SINGULAR_FUZZ && fabs(t2) < SINGULAR_FUZZ){
     *fflags |= PUMA_REACH;
     th4 = joint[3]*PM_PI/180;            /* use current value */
   }
   else{
     th4 = atan2(t1, t2);
   }

   /* compute sin, cos for later calcs */
   s4 = sin(th4);
   c4 = cos(th4);

   /* Joint 5 */

   s5 = hom.rot.z.z * (s23 * c4) -
        hom.rot.z.x * (c1 * c23 * c4 + s1 * s4) -
        hom.rot.z.y * (s1 * c23 * c4 - c1 * s4);
   c5 =-hom.rot.z.x * (c1 * s23) - hom.rot.z.y *
        (s1 * s23) - hom.rot.z.z * c23;
   th5 = atan2(s5, c5);

   /* Joint 6 */

   s6 = hom.rot.x.z * (s23 * s4) - hom.rot.x.x *
        (c1 * c23 * s4 - s1 * c4) - hom.rot.x.y *
        (s1 * c23 * s4 + c1 * c4);
   c6 = hom.rot.x.x * ((c1 * c23 * c4 + s1 * s4) *
        c5 - c1 * s23 * s5) + hom.rot.x.y *
        ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) -
        hom.rot.x.z * (s23 * c4 * c5 + c23 * s5);
   th6 = atan2(s6, c6);

   /* FIXME-- is wrist flip the normal or offset results? */
   if (*iflags & PUMA_WRIST_FLIP){
     th4 = th4 + PM_PI;
     th5 = -th5;
     th6 = th6 + PM_PI;
   }

   /* copy out */
   joint[0] = th1*180/PM_PI;
   joint[1] = th2*180/PM_PI;
   joint[2] = th3*180/PM_PI;
   joint[3] = th4*180/PM_PI;
   joint[4] = th5*180/PM_PI;
   joint[5] = th6*180/PM_PI;

   return 0;
}

int pumaKinematicsSetup(const  int   comp_id,
                        const  char* coordinates,
                        kparms*      kp)
{
    (void)coordinates;
    int res=0;

    haldata = hal_malloc(sizeof(*haldata));
    if (!haldata) goto error;


    res += hal_pin_float_newf(HAL_IN, &(haldata->a2), comp_id,"%s.A2",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->a3), comp_id,"%s.A3",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d3), comp_id,"%s.D3",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d4), comp_id,"%s.D4",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d6), comp_id,"%s.D6",kp->halprefix);
    if (res) { goto error; }

    PUMA_A2 = DEFAULT_PUMA560_A2;
    PUMA_A3 = DEFAULT_PUMA560_A3;
    PUMA_D3 = DEFAULT_PUMA560_D3;
    PUMA_D4 = DEFAULT_PUMA560_D4;
    PUMA_D6 = DEFAULT_PUMA560_D6;

    return 0;

error:
    return -1;
} // pumaKinematicsSetup()

int switchkinsSetup(kparms* kp,
                    KS* kset0, KS* kset1, KS* kset2,
                    KF* kfwd0, KF* kfwd1, KF* kfwd2,
                    KI* kinv0, KI* kinv1, KI* kinv2
                   )
{
    kp->kinsname    = "pumakins"; // !!! must agree with filename
    kp->halprefix   = "pumakins"; // hal pin names
    kp->required_coordinates = "xyzabc";
    kp->allow_duplicates     = 0;
    kp->max_joints = strlen(kp->required_coordinates);

    rtapi_print("\n!!! switchkins-type 0 is %s\n",kp->kinsname);
    *kset0 = pumaKinematicsSetup;
    *kfwd0 = pumaKinematicsForward;
    *kinv0 = pumaKinematicsInverse;

    *kset1 = identityKinematicsSetup;
    *kfwd1 = identityKinematicsForward;
    *kinv1 = identityKinematicsInverse;

    *kset2 = userkKinematicsSetup;
    *kfwd2 = userkKinematicsForward;
    *kinv2 = userkKinematicsInverse;

    return 0;
} // switchkinsSetup()
