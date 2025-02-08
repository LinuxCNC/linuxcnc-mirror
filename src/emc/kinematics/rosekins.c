/*
  Copyright 2016 Dewey Garrett <dgarrett@panix.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include "kinematics.h"
#include "posemath.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi_math.h"
#include "rtapi_app.h"

KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsInverse);
EXPORT_SYMBOL(kinematicsForward);
MODULE_LICENSE("GPL");

#ifndef hypot
#define hypot(a,b) (sqrt((a)*(a)+(b)*(b)))
#endif

struct haldata {
    hal_float_t *revolutions;
    hal_float_t *theta_degrees;
    hal_float_t *bigtheta_degrees;
} *haldata;

int kinematicsForward(const double *joints,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    (void)fflags;
    (void)iflags;
    double radius,z,theta;

    radius = joints[0];
    z      = joints[1];
    theta  = TO_RAD * joints[2];

    pos->tran.x = radius * cos(theta);
    pos->tran.y = radius * sin(theta);
    pos->tran.z = z;
    pos->a = 0;
    pos->b = 0;
    pos->c = 0;
    pos->u = 0;
    pos->v = 0;
    pos->w = 0;

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
                      double *joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)iflags;
    (void)fflags;
// There is a potential problem when accumulating bigtheta -- loss of
// precision based on size of mantissa -- but in practice, it is probably ok

    static int oldquad;
    static int revolutions;

    double     theta,bigtheta;
    int        nowquad = 0;
    double     x = pos->tran.x;
    double     y = pos->tran.y;
    double     z = pos->tran.z;

    if      (x >= 0 && y >= 0) nowquad = 1;
    else if (x <  0 && y >= 0) nowquad = 2;
    else if (x <  0 && y <  0) nowquad = 3;
    else if (x >= 0 && y <  0) nowquad = 4;

    if (oldquad == 2 && nowquad == 3) {revolutions += 1;}
    if (oldquad == 3 && nowquad == 2) {revolutions -= 1;}

    theta     = atan2(y,x);
    bigtheta  = theta + PM_2_PI * revolutions;

    *(haldata->revolutions) = revolutions;
    *(haldata->theta_degrees) = theta * TO_DEG;
    *(haldata->bigtheta_degrees) = bigtheta * TO_DEG;

    joints[0] = hypot(x,y);
    joints[1] = z;
    joints[2] = TO_DEG * bigtheta;
    joints[3] = 0;
    joints[4] = 0;
    joints[5] = 0;
    joints[6] = 0;
    joints[7] = 0;
    joints[8] = 0;

    oldquad = nowquad;
    return 0;
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

static int comp_id;

void rtapi_app_exit(void) { hal_exit(comp_id); }

int rtapi_app_main(void) {
    int ans;
    comp_id = hal_init("rosekins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));

    if((ans = hal_pin_float_new("rosekins.revolutions",
              HAL_OUT, &(haldata->revolutions), comp_id)) < 0) goto error;
    if((ans = hal_pin_float_new("rosekins.theta_degrees",
              HAL_OUT, &(haldata->theta_degrees), comp_id)) < 0) goto error;
    if((ans = hal_pin_float_new("rosekins.bigtheta_degrees",
              HAL_OUT, &(haldata->bigtheta_degrees), comp_id)) < 0) goto error;

    hal_ready(comp_id);
    return 0;

error:
    return ans;
}
