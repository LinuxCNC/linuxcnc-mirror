/********************************************************************
* Description: kinematics.h
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "emcpos.h" /* EmcPose */
#include "rtapi_bool.h"

/*
  The type of kinematics used.
  
  KINEMATICS_IDENTITY means that the joints and world coordinates are the
  same, as for slideway machines (XYZ milling machines). The EMC will allow
  changing from joint to world mode and vice versa. Also, the EMC will set
  the actual world position to be the actual joint positions (not commanded)
  by calling the forward kinematics each trajectory cycle.

  KINEMATICS_FORWARD_ONLY means that only the forward kinematics exist.
  Since the EMC requires at least the inverse kinematics, this should simply
  terminate the EMC.

  KINEMATICS_INVERSE_ONLY means that only the inverse kinematics exist.
  The forwards won't be called, and the EMC will only allow changing from
  joint to world mode at the home position.

  KINEMATICS_BOTH means that both the forward and inverse kins are defined.
  Like KINEMATICS_IDENTITY, the EMC will allow changing between world and
  joint modes. However, the kins are assumed to be somewhat expensive
  computationally, and the forwards won't be called at the trajectory rate
  to compute actual world coordinates from actual joint values.
*/

typedef enum {
    KINEMATICS_IDENTITY = 1,/* forward=inverse, both well-behaved */
    KINEMATICS_FORWARD_ONLY,/* forward but no inverse */
    KINEMATICS_INVERSE_ONLY,/* inverse but no forward */
    KINEMATICS_BOTH         /* forward and inverse both */
} KINEMATICS_TYPE;

/* the forward flags are passed to the forward kinematics so that they
   can resolve ambiguities in the world coordinates for a given joint set,
   e.g., for hexpods, this would be platform-below-base, platform-above-base.

   The flags are also passed to the inverse kinematics and are set by them,
   which is how they are changed from their initial value. For example, for
   hexapods you could do a coordinated move that brings the platform up from
   below the base to above the base. The forward flags would be set to
   indicate this. */
typedef unsigned long int KINEMATICS_FORWARD_FLAGS;

/* the inverse flags are passed to the inverse kinematics so that they
   can resolve ambiguities in the joint angles for a given world coordinate,
   e.g., for robots, this would be elbow-up, elbow-down, etc.

   The flags are also passed to the forward kinematics and are set by them,
   which is how they are changed from their initial value. For example, for
   robots you could do a joint move that brings the elbow from a down
   configuration to an up configuration. The inverse flags would be set to
   indicate this. */
typedef unsigned long int KINEMATICS_INVERSE_FLAGS;

/* the forward kinematics take joint values and determine world coordinates,
   given forward kinematics flags to resolve any ambiguities. The inverse
   flags are set to indicate their value appropriate to the joint values
   passed in. */
extern int kinematicsForward(const double *joint,
                             struct EmcPose * world,
                             const KINEMATICS_FORWARD_FLAGS * fflags,
                             KINEMATICS_INVERSE_FLAGS * iflags);

/* the inverse kinematics take world coordinates and determine joint values,
   given the inverse kinematics flags to resolve any ambiguities. The forward
   flags are set to indicate their value appropriate to the world coordinates
   passed in. */
extern int kinematicsInverse(const struct EmcPose * world,
                             double *joint,
                             const KINEMATICS_INVERSE_FLAGS * iflags,
                             KINEMATICS_FORWARD_FLAGS * fflags);

/* the home kinematics function sets all its arguments to their proper
   values at the known home position. When called, these should be set,
   when known, to initial values, e.g., from an INI file. If the home
   kinematics can accept arbitrary starting points, these initial values
   should be used.
*/
extern int kinematicsHome(struct EmcPose * world,
                          double *joint,
                          KINEMATICS_FORWARD_FLAGS * fflags,
                          KINEMATICS_INVERSE_FLAGS * iflags);

extern KINEMATICS_TYPE kinematicsType(void);

/* parameters for use with switchkins.c */
typedef struct kinematics_parms {
  char* sparm;     // module string parameter passed to kins
  char* kinsname;  // must agree with module(file) name
  char* halprefix; // for hal pin hames
  char* required_coordinates;
  int   max_joints;
  int   allow_duplicates;
  int   fwd_iterates_mask; // identify kins types that use iterative
                           // forward kinematics (typ: genhex)
                           // bitmask: 0x0 none
                           // bitmask: 0x1 bit0: switchkins_type==0
                           // bitmask: 0x2 bit1: switchkins_type==1
                           // bitmask: 0x4 bit2: switchkins_type==2
  int   gui_kinstype; // may be reqd for parallel kins with vismach
                      // to select switchkins_type for gui pins
} kparms;

/* map letters in a coordinates string to joint numbers
** sequentially.  Axis indices are 0:x,1:y,...,etc
** Example: coordinates=XYZYAC
** Result:  axis_idx_for_jno[0] = 0 ==> X
**          axis_idx_for_jno[1] = 1 ==> Y
**          axis_idx_for_jno[2] = 2 ==> Z
**          axis_idx_for_jno[3] = 1 ==> Y (duplicate allowed)
**          axis_idx_for_jno[4] = 1 ==> A
**          axis_idx_for_jno[5] = 1 ==> C
*/
extern int map_coordinates_to_jnumbers(const char *coordinates,
                                       const int  max_joints,
                                       const int  allow_duplicates,
                                             int  axis_idx_for_jno[]);

extern int mapped_joints_to_position(const int max_joints,
                                     const double* joints,
                                     EmcPose*  pose);

extern int position_to_mapped_joints(const int max_joints,
                                     const EmcPose* pos,
                                     double* joints);

extern int identityKinematicsSetup(const int   comp_id,
                                   const char* coordinates,
                                   kparms*     ksetup_parms);

extern int identityKinematicsForward(const double *joint,
                                     struct EmcPose * world,
                                     const KINEMATICS_FORWARD_FLAGS * fflags,
                                     KINEMATICS_INVERSE_FLAGS * iflags);

extern int identityKinematicsInverse(const struct EmcPose * world,
                                     double *joint,
                                     const KINEMATICS_INVERSE_FLAGS * iflags,
                                     KINEMATICS_FORWARD_FLAGS * fflags);

extern int kinematicsSwitchable(void);
extern int kinematicsSwitch(int switchkins_type);
//NOTE: switchable kinematics may require Interp::Synch
//      before/after invoking kinematicsSwitch()
//      A convenient command to synch is: M66 E0 L0

#define KINS_NOT_SWITCHABLE \
extern int kinematicsSwitchable() {return 0;} \
extern int kinematicsSwitch(int switchkins_type) { (void)switchkins_type; return 0;} \
EXPORT_SYMBOL(kinematicsSwitchable); \
EXPORT_SYMBOL(kinematicsSwitch);


// support for template for user-defined switchkins_type==2
extern int userkKinematicsSetup(const int   comp_id,
                                const char* coordinates,
                                kparms*     ksetup_parms);

extern int userkKinematicsForward(const double *joint,
                                  struct EmcPose * world,
                                  const KINEMATICS_FORWARD_FLAGS * fflags,
                                  KINEMATICS_INVERSE_FLAGS * iflags);

extern int userkKinematicsInverse(const struct EmcPose * world,
                                  double *joint,
                                  const KINEMATICS_INVERSE_FLAGS * iflags,
                                  KINEMATICS_FORWARD_FLAGS * fflags);
#endif
//*********************************************************************
// xyzac,xyzbc;
extern int trtKinematicsSetup(const int   comp_id,
                              const char* coordinates,
                              kparms*     ksetup_parms);

extern int xyzacKinematicsForward(const double *joints,
                                  EmcPose * pos,
                                  const KINEMATICS_FORWARD_FLAGS * fflags,
                                  KINEMATICS_INVERSE_FLAGS * iflags);

extern int xyzacKinematicsInverse(const EmcPose * pos,
                                  double *joints,
                                  const KINEMATICS_INVERSE_FLAGS * iflags,
                                  KINEMATICS_FORWARD_FLAGS * fflags);


extern int xyzbcKinematicsForward(const double *joints,
                                  EmcPose * pos,
                                  const KINEMATICS_FORWARD_FLAGS * fflags,
                                  KINEMATICS_INVERSE_FLAGS * iflags);

extern int xyzbcKinematicsInverse(const EmcPose * pos,
                                  double *joints,
                                  const KINEMATICS_INVERSE_FLAGS * iflags,
                                  KINEMATICS_FORWARD_FLAGS * fflags);

//*********************************************************************
