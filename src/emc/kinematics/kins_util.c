/* Utility routines for kinematics modules
** License GPL Version 2
**
** utilities for use with switchkins.c
**---------------------------------------------------------------------
** identityKinematicsSetup()
** identityKinematicsForward()
** identityKinematicsInverse()
**
** Routines for identity kinematics using mapping created by
** map_coordinates_to_jnumbers()
**
**---------------------------------------------------------------------
** map_coordinates_to_jnumbers()
**
** Map a string of coordinate letters to joint numbers sequentially.
** If allow_duplicates==1, a coordinate letter may be specified more
** than once to assign it to multiple joint numbers (the kinematics
** module must support such usage).
**
**   Default mapping if coordinates==NULL is:
**           X:0 Y:1 Z:2 A:3 B:4 C:5 U:6 V:7 W:8
**
**   Example coordinates-to-joints mappings:
**      coordinates=XYZ      X:0   Y:1   Z:2
**      coordinates=ZYX      Z:0   Y:1   X:2
**      coordinates=XYZZZZ   x:0   Y:1   Z:2,3,4,5
**      coordinates=XXYZ     X:0,1 Y:2   Z:3
**---------------------------------------------------------------------
**
** mapped_joints_to_position()
**
** Update position based mapping created by map_coordinates_to_jnumbers()
** (used for identity-based forward kinematics)
**---------------------------------------------------------------------
**
** position_to_mapped_joints()
**
** Update joints (including joints for duplicate letters)
** based on mapping created by map_coordinates_to_jnumbers()
** (used for identity-based inverse kinematics)
**
**---------------------------------------------------------------------
*/

#include "rtapi.h"
#include "motion.h"
#include "rtapi_string.h"

// principal joint numbers based on module 'coordinates' parameter
static int JX = -1;
static int JY = -1;
static int JZ = -1;
static int JA = -1;
static int JB = -1;
static int JC = -1;
static int JU = -1;
static int JV = -1;
static int JW = -1;

// bitmaps indicate joints used for each axis letter
static int X_joints_bitmap;
static int Y_joints_bitmap;
static int Z_joints_bitmap;
static int A_joints_bitmap;
static int B_joints_bitmap;
static int C_joints_bitmap;
static int U_joints_bitmap;
static int V_joints_bitmap;
static int W_joints_bitmap;

static int map_initialized = 0;
#define MAX_COORDINATES_CHARS 32
static char used_coordinates[MAX_COORDINATES_CHARS+1];

int map_coordinates_to_jnumbers(const char *coordinates,
                                const int  max_joints,
                                const int  allow_duplicates,
                                int   axis_idx_for_jno[] ) //result
{
    char* errtag="map_coordinates_to_jnumbers: ERROR:\n  ";
    int   jno=0;
    bool  found=0;
    int   dups[EMCMOT_MAX_AXIS];
    const char *coords = coordinates;
    char  coord_letter[] = {'X','Y','Z','A','B','C','U','V','W'};
    int   i;

    if (strlen(coordinates) > MAX_COORDINATES_CHARS) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "%s: map_coordinates_to_jnumbers too many chars:%s\n"
             ,__FILE__,coordinates);
        return -1;

    }
    // Note: may be called multiple times for different switchkins
    // types but coordinates must agree
    if (used_coordinates[0] == 0) {
        strcpy(used_coordinates,coordinates);
    } else {
        if (strcasecmp(coordinates,used_coordinates)) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                 "%s: map_coordinates_to_jnumbers altered:%s %s\n"
                 ,__FILE__,used_coordinates,coordinates);
            return -1;
        }
    }
    for (i=0; i<EMCMOT_MAX_AXIS; i++) {dups[i] = 0;}

    if ( (max_joints <= 0) || (max_joints > EMCMOT_MAX_JOINTS) ) {
        rtapi_print_msg(RTAPI_MSG_ERR,"%s bogus max_joints=%d\n",
          errtag,max_joints);
        return -1;
    }

    // init all axis_idx_for_jno[] (-1 means unspecified)
    for(jno=0; jno<EMCMOT_MAX_JOINTS; jno++) { axis_idx_for_jno[jno] = -1; }

    if (coords == NULL) { coords = "XYZABCUVW"; }
    jno = 0; // begin: assign joint numbers at 0th coords position
    while (*coords) {
        found = 0;
        switch(*coords) {
          case 'x': case 'X': axis_idx_for_jno[jno]= 0;dups[0]++;found=1;break;
          case 'y': case 'Y': axis_idx_for_jno[jno]= 1;dups[1]++;found=1;break;
          case 'z': case 'Z': axis_idx_for_jno[jno]= 2;dups[2]++;found=1;break;
          case 'a': case 'A': axis_idx_for_jno[jno]= 3;dups[3]++;found=1;break;
          case 'b': case 'B': axis_idx_for_jno[jno]= 4;dups[4]++;found=1;break;
          case 'c': case 'C': axis_idx_for_jno[jno]= 5;dups[5]++;found=1;break;
          case 'u': case 'U': axis_idx_for_jno[jno]= 6;dups[6]++;found=1;break;
          case 'v': case 'V': axis_idx_for_jno[jno]= 7;dups[7]++;found=1;break;
          case 'w': case 'W': axis_idx_for_jno[jno]= 8;dups[8]++;found=1;break;
          case ' ': case '\t': coords++;continue; //whitespace
        }
        if (found) {
            coords++; // next coordinates letter
            jno++;    // next joint number
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
              "%s Invalid character '%c' in coordinates '%s'\n",
              errtag,*coords,coordinates);
            return -1;
        }
        if (jno > max_joints) {
            rtapi_print_msg(RTAPI_MSG_ERR,
              "%s too many coordinates <%s> for max_joints=%d\n",
              errtag,coordinates,max_joints);
            return -1;
        }
    } // while

    if (!found) {
        rtapi_print_msg(RTAPI_MSG_ERR,"%s missing coordinates '%s'\n",
          errtag,coordinates);
        return -1;
    }
    if (!allow_duplicates) {
        int ano;
        for(ano=0; ano<EMCMOT_MAX_AXIS; ano++) {
            if (dups[ano] > 1) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                "%s duplicates not allowed in coordinates=%s, letter=%c\n",
                errtag,coordinates,coord_letter[ano]);
                return -1;
            }
        }
    }

    for (jno=0; jno < max_joints; jno++) {
      int bitnumber = 1<<jno;
      /* Assign principal joint (first joint listed for a coordinate letter
      ** (using the coordinates module parameter) and use for forward
      ** kinematics.
      ** Assign a bitmap for duplicate joints listed and use for inverse
      **  kinematics.
      **
      ** example: coordinates=xyzbcwy (duplicate y)
      **          JX=0 X_joints_bitmap=0x01 joints: 0
      **          JY=1 Y_joints_bitmap=0x42 joints: 1 and 6
      **          JZ=2 Z_joints_bitmap=0x04 joints: 2
      **          JB=3 C_joints_bitmap=0x10 joints: 3
      **          JC=4 C_joints_bitmap=0x10 joints: 4
      **          JW=5 C_joints_bitmap=0x10 joints: 5
      **
      ** xyzabcuvw letters
      ** 012345678 indices
      */
      if (axis_idx_for_jno[jno] == 0) {
         if (JX == -1)  JX=jno;
         X_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 1) {
         if (JY == -1)  JY=jno;
         Y_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 2) {
         if (JZ == -1)  JZ=jno;
         Z_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 3) {
         if (JA == -1)  JA=jno;
         A_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 4) {
         if (JB == -1)  JB=jno;
         B_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 5) {
         if (JC == -1)  JC=jno;
         C_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 6) {
         if (JU == -1)  JU=jno;
         U_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 7) {
         if (JV == -1)  JV=jno;
         V_joints_bitmap |= bitnumber;
      }
      if (axis_idx_for_jno[jno] == 8) {
         if (JW == -1)  JW=jno;
         W_joints_bitmap |= bitnumber;
      }
    }
    map_initialized = 1;
    return 0;
} //map_coordinates_to_jnumbers()

int mapped_joints_to_position(const int max_joints,
                              const double * joints,
                              EmcPose * pos)
{
    int jno;
    if (!map_initialized) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "mapped_joints_to_position() before map_initialized\n");
        return -1;
    }
    for (jno=0; jno < max_joints; jno++) {
        int bit = 1<<jno;
        if ( bit & X_joints_bitmap ) pos->tran.x = joints[JX];
        if ( bit & Y_joints_bitmap ) pos->tran.y = joints[JY];
        if ( bit & Z_joints_bitmap ) pos->tran.z = joints[JZ];
        if ( bit & A_joints_bitmap ) pos->a      = joints[JA];
        if ( bit & B_joints_bitmap ) pos->b      = joints[JB];
        if ( bit & C_joints_bitmap ) pos->c      = joints[JC];
        if ( bit & U_joints_bitmap ) pos->u      = joints[JU];
        if ( bit & V_joints_bitmap ) pos->v      = joints[JV];
        if ( bit & W_joints_bitmap ) pos->w      = joints[JW];
    }
    return 0;
} // mapped_joints_to_position()

int position_to_mapped_joints(const int max_joints,
                              const EmcPose * pos,
                              double* joints)
{
    int jno;
    if (!map_initialized) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "position_to_mapped_joints before map_initialized\n");
        return -1;
    }
    for (jno=0; jno < max_joints; jno++) {
        int bit = 1<<jno;
        if ( bit & X_joints_bitmap ) joints[jno] = pos->tran.x;
        if ( bit & Y_joints_bitmap ) joints[jno] = pos->tran.y;
        if ( bit & Z_joints_bitmap ) joints[jno] = pos->tran.z;
        if ( bit & A_joints_bitmap ) joints[jno] = pos->a;
        if ( bit & B_joints_bitmap ) joints[jno] = pos->b;
        if ( bit & C_joints_bitmap ) joints[jno] = pos->c;
        if ( bit & U_joints_bitmap ) joints[jno] = pos->u;
        if ( bit & V_joints_bitmap ) joints[jno] = pos->v;
        if ( bit & W_joints_bitmap ) joints[jno] = pos->w;
    }
    return 0;
} // position_to_mapped_joints()

static int identity_kinematics_initialized = 0;
static int identity_max_joints;

int identityKinematicsSetup(const int   comp_id,
                            const char* coordinates,
                            kparms*     kp)
{
    (void)comp_id;
    int axis_idx_for_jno[EMCMOT_MAX_JOINTS];
    int jno;
    int show=0;
    bool islathe;

    identity_max_joints = strlen(coordinates);

    if (map_coordinates_to_jnumbers(coordinates,
                                    kp->max_joints,
                                    kp->allow_duplicates,
                                    axis_idx_for_jno)) {
       return -1; //mapping failed
    }

    /* print message for unconventional ordering;
    **   a) duplicate coordinate letters
    **   b) letters not ordered by "XYZABCUVW" sequence
    **      (use kinstype=both works best for these)
    */
    for (jno=0; jno<identity_max_joints; jno++) {
        if (axis_idx_for_jno[jno] == -1) break; //fini
        if (axis_idx_for_jno[jno] != jno) { show++; } //not default order
    }
    islathe = !strcasecmp(coordinates,"xz"); // no show if simple lathe
    if (show && !islathe) {
        rtapi_print("\nidentityKinematicsSetup: coordinates:%s\n", coordinates);
        char *p="XYZABCUVW";
        for (jno=0; jno<identity_max_joints; jno++) {
            if (axis_idx_for_jno[jno] == -1) break; //fini
            rtapi_print("   Joint %d ==> Axis %c\n",
                       jno,*(p+axis_idx_for_jno[jno]));
        }
        if (kinematicsType() != KINEMATICS_BOTH) {
            rtapi_print("identityKinematicsSetup: Recommend: kinstype=both\n");
        }
        rtapi_print("\n");
    }

    identity_kinematics_initialized = 1;
    return 0;
} // identityKinematicsSetup()

int identityKinematicsForward(const double *joints,
                              EmcPose * pos,
                              const KINEMATICS_FORWARD_FLAGS * fflags,
                              KINEMATICS_INVERSE_FLAGS * iflags)
{
    (void)fflags;
    (void)iflags;
    if (!identity_kinematics_initialized) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "identityKinematicsForward: not initialized\n");
        return -1;
    }

    // support multiple-joint-per-coordinate-letter assignments:
    mapped_joints_to_position(identity_max_joints,joints,pos);
    return 0;
} // identityKinematicsForward()

int identityKinematicsInverse(const EmcPose * pos,
                              double *joints,
                              const KINEMATICS_INVERSE_FLAGS * iflags,
                              KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)iflags;
    (void)fflags;
    if (!identity_kinematics_initialized) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "identityKinematicsInverse: not initialized\n");
        return -1;
    }

    // support multiple-joint-per-coordinate-letter assignments:
    position_to_mapped_joints(identity_max_joints,pos,joints);

    return 0;
} // identityKinematicsInverse()
