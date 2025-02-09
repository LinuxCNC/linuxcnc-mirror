/********************************************************************
* Description: posemath.h
*   Declarations for pose math library data types and manipulation
*   functions.
*
*   Data types comprise various representations of translation and
*   rotation quantities, and a 'pose' for representing the location
*   and orientation of a frame in space relative to a base frame.
*   Translation representations include cartesian, spherical, and
*   cylindrical coordinates. All of these contain 3 elements. Rotation
*   representations include rotation vectors, quaternions, rotation
*   matrices, Euler angles, and roll-pitch-yaw. These contain at least
*   3 elements, and may contain more. Only 3 are necessary for the 3
*   degrees of freedom for either translation or rotation, but some
*   data representations use more for computational efficiency or
*   intuition at the expense of storage space.
*
*   Types are abbreviated in function naming with a few letters.
*   Functions exist for conversion between data types, checking for
*   consistency, normalization into consistency, extracting features
*   such as size, and arithmetic operations.
*
*   Names of data representations are in all capitals, prefixed with
*   'PM_'. Names of functions are in mixed case, prefixed with 'pm',
*   with case changes used to indicate new quantities instead of
*   underscores. Function syntax looks like
*    int UmQuatRotConvert(PM_QUATERNION, PM_ROTATION_VECTOR *);
*
*   The return value is an error code, 0 for success, or a non-zero
*   error code for failure, for example:
*
*    #define PM_ERR -1
*    #define PM_IMPL_ERR -2
*
*   The global variable 'pmErrno' is set to this return value.
*
*   C++ classes are used for data types so that operator overloading can
*   be used to reduce the programming labor. Using the overloaded operator
*   version of functions loses the integer error code. The global
*   variable 'pmErrno' can be queried after these operations. This is not
*   thread-safe or reentrant.
*
*   C++ names corresponding to the C structures use case mixing instead
*   of all caps. Thus, a quaternion in C++ is a PmQuaternion.
*
*   The MATH_DEBUG symbol can be defined to include error reporting via
*   printed errors.
*
*   Native efficient C functions exist for the PM_CARTESIAN, PM_QUATERNION,
*   and PM_POSE types. Constructors in all the classes have been defined
*   to convert to/from PM_CARTESIAN and any other translation type, and
*   to convert to/from PM_QUATERNION and any other rotation type. This means
*   that if no explicit C functions exist for another type, conversions
*   to the corresponding native type will occur automatically. If more
*   efficiency is desired for a particular type, C functions to handle the
*   operations should be coded and the overloaded C++ functions or operators
*   should be added.
*
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: LGPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/

#ifndef POSEMATH_H
#define POSEMATH_H

#ifdef __cplusplus

#define USE_CONST
#define USE_CCONST
#define USE_REF

#ifdef USE_CCONST
#define PM_CCONST const
#else
#define PM_CCONST
#endif

#ifdef USE_CONST
#define PM_CONST const
#else
#define PM_CONST
#endif

#ifdef USE_REF
#define PM_REF  &
#else
#define PM_REF
#endif

#if __cplusplus < 201103L
/*
 * Not required in C++11 and beyond. It will generate a warning:
 *    "implicitly-declared 'constexpr ...' is deprecated [-Wdeprecated-copy]"
 * If, somehow, somewhere, it is required, then you probably must implement
 * both copy-constructor, destructor and operator= override (rule of three).
 * Otherwise, if you do not have anything special in the members, then the
 * compiler will do a good job at doing the right thing for you.
 */
#define INCLUDE_POSEMATH_COPY_CONSTRUCTORS
#endif

/* forward declarations-- conversion ctors will need these */

/* translation types */
struct PM_CARTESIAN;		/* Cart */
struct PM_SPHERICAL;		/* Sph */
struct PM_CYLINDRICAL;		/* Cyl */

/* rotation types */
struct PM_ROTATION_VECTOR;	/* Rot */
struct PM_ROTATION_MATRIX;	/* Mat */
struct PM_QUATERNION;		/* Quat */
struct PM_EULER_ZYZ;		/* Zyz */
struct PM_EULER_ZYX;		/* Zyx */
struct PM_RPY;			/* Rpy */

/* pose types */
struct PM_POSE;			/* Pose */
struct PM_HOMOGENEOUS;		/* Hom */

/* PM_CARTESIAN */

struct PM_CARTESIAN {
    /* ctors/dtors */
    PM_CARTESIAN() {
    };
    PM_CARTESIAN(double _x, double _y, double _z);
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_CARTESIAN(PM_CCONST PM_CARTESIAN & cart);	// added 7-May-1997
    // by WPS
#endif

    PM_CARTESIAN(PM_CONST PM_CYLINDRICAL PM_REF c);	/* conversion */
    PM_CARTESIAN(PM_CONST PM_SPHERICAL PM_REF s);	/* conversion */

    /* operators */
    double &operator[] (int n);	/* this[n] */
    PM_CARTESIAN & operator += (const PM_CARTESIAN &o);
    PM_CARTESIAN & operator -= (const PM_CARTESIAN &o);

    // Scalar operations
    PM_CARTESIAN & operator *= (double o);
    PM_CARTESIAN & operator /= (double o);

    /* data */
    double x, y, z;		/* this.x, etc. */
};

/* PM_SPHERICAL */

struct PM_SPHERICAL {
    /* ctors/dtors */
    PM_SPHERICAL() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_SPHERICAL(PM_CCONST PM_SPHERICAL & s);
#endif
    PM_SPHERICAL(double _theta, double _phi, double _r);
    PM_SPHERICAL(PM_CONST PM_CYLINDRICAL PM_REF v);	/* conversion */
    PM_SPHERICAL(PM_CONST PM_CARTESIAN PM_REF v);	/* conversion */

    /* operators */
    double &operator[] (int n);	/* this[n] */

    /* data */
    double theta, phi, r;
};

/* PM_CYLINDRICAL */

struct PM_CYLINDRICAL {
    /* ctors/dtors */
    PM_CYLINDRICAL() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_CYLINDRICAL(PM_CCONST PM_CYLINDRICAL & c);
#endif
    PM_CYLINDRICAL(double _theta, double _r, double _z);
    PM_CYLINDRICAL(PM_CONST PM_CARTESIAN PM_REF v);	/* conversion */
    PM_CYLINDRICAL(PM_CONST PM_SPHERICAL PM_REF v);	/* conversion */

    /* operators */
    double &operator[] (int n);	/* this[n] */

    /* data */
    double theta, r, z;
};

/* PM_ROTATION_VECTOR */

struct PM_ROTATION_VECTOR {
    /* ctors/dtors */
    PM_ROTATION_VECTOR() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_ROTATION_VECTOR(PM_CCONST PM_ROTATION_VECTOR & r);
#endif
    PM_ROTATION_VECTOR(double _r, double _x, double _y, double _z);
    PM_ROTATION_VECTOR(PM_CONST PM_QUATERNION PM_REF q);	/* conversion 
								 */

    /* operators */
    double &operator[] (int n);	/* this[n] */

    /* data */
    double s, x, y, z;
};

/* PM_ROTATION_MATRIX */

struct PM_ROTATION_MATRIX {
    /* ctors/dtors */
    PM_ROTATION_MATRIX() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_ROTATION_MATRIX(PM_CCONST PM_ROTATION_MATRIX & mat);	/* added
								   7-May-1997 
								   by WPS */
#endif
    PM_ROTATION_MATRIX(double xx, double xy, double xz,
	double yx, double yy, double yz, double zx, double zy, double zz);
    PM_ROTATION_MATRIX(const PM_CARTESIAN& _x, const PM_CARTESIAN& _y, const PM_CARTESIAN& _z);
    PM_ROTATION_MATRIX(PM_CONST PM_ROTATION_VECTOR PM_REF v);	/* conversion 
								 */
    PM_ROTATION_MATRIX(PM_CONST PM_QUATERNION PM_REF q);	/* conversion 
								 */
    PM_ROTATION_MATRIX(PM_CONST PM_EULER_ZYZ PM_REF zyz);	/* conversion 
								 */
    PM_ROTATION_MATRIX(PM_CONST PM_EULER_ZYX PM_REF zyx);	/* conversion 
								 */
    PM_ROTATION_MATRIX(PM_CONST PM_RPY PM_REF rpy);	/* conversion */

    /* operators */
    PM_CARTESIAN & operator[](int n);	/* this[n] */

    /* data */
    PM_CARTESIAN x, y, z;
};

/* PM_QUATERNION */

enum PM_AXIS { PM_X, PM_Y, PM_Z };

struct PM_QUATERNION {
    /* ctors/dtors */
    PM_QUATERNION() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_QUATERNION(PM_CCONST PM_QUATERNION & quat);	/* added 7-May-1997
							   by WPS */
#endif
    PM_QUATERNION(double _s, double _x, double _y, double _z);
    PM_QUATERNION(PM_CONST PM_ROTATION_VECTOR PM_REF v);	/* conversion 
								 */
    PM_QUATERNION(PM_CONST PM_ROTATION_MATRIX PM_REF m);	/* conversion 
								 */
    PM_QUATERNION(PM_CONST PM_EULER_ZYZ PM_REF zyz);	/* conversion */
    PM_QUATERNION(PM_CONST PM_EULER_ZYX PM_REF zyx);	/* conversion */
    PM_QUATERNION(PM_CONST PM_RPY PM_REF rpy);	/* conversion */
    PM_QUATERNION(PM_AXIS axis, double angle);	/* conversion */

    /* operators */
    double &operator[] (int n);	/* this[n] */

    /* functions */
    void axisAngleMult(PM_AXIS axis, double angle);

    /* data */
    double s, x, y, z;		/* this.s, etc. */
};

/* PM_EULER_ZYZ */

struct PM_EULER_ZYZ {
    /* ctors/dtors */
    PM_EULER_ZYZ() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_EULER_ZYZ(PM_CCONST PM_EULER_ZYZ & zyz);
#endif
    PM_EULER_ZYZ(double _z, double _y, double _zp);
    PM_EULER_ZYZ(PM_CONST PM_QUATERNION PM_REF q);	/* conversion */
    PM_EULER_ZYZ(PM_CONST PM_ROTATION_MATRIX PM_REF m);	/* conversion */

    /* operators */
    double &operator[] (int n);

    /* data */
    double z, y, zp;
};

/* PM_EULER_ZYX */

struct PM_EULER_ZYX {
    /* ctors/dtors */
    PM_EULER_ZYX() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_EULER_ZYX(PM_CCONST PM_EULER_ZYX & zyx);
#endif
    PM_EULER_ZYX(double _z, double _y, double _x);
    PM_EULER_ZYX(PM_CONST PM_QUATERNION PM_REF q);	/* conversion */
    PM_EULER_ZYX(PM_CONST PM_ROTATION_MATRIX PM_REF m);	/* conversion */

    /* operators */
    double &operator[] (int n);

    /* data */
    double z, y, x;
};

/* PM_RPY */

struct PM_RPY {
    /* ctors/dtors */
    PM_RPY() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_RPY(PM_CCONST PM_RPY PM_REF rpy);	/* added 7-May-1997 by WPS */
#endif
    PM_RPY(double _r, double _p, double _y);
    PM_RPY(PM_CONST PM_QUATERNION PM_REF q);	/* conversion */
    PM_RPY(PM_CONST PM_ROTATION_MATRIX PM_REF m);	/* conversion */

    /* operators */
    double &operator[] (int n);

    /* data */
    double r, p, y;
};

/* PM_POSE */

struct PM_POSE {
    /* ctors/dtors */
    PM_POSE() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_POSE(PM_CCONST PM_POSE & p);
#endif
    PM_POSE(const PM_CARTESIAN& v, const PM_QUATERNION& q);
    PM_POSE(double x, double y, double z,
	double s, double sx, double sy, double sz);
    PM_POSE(PM_CONST PM_HOMOGENEOUS PM_REF h);	/* conversion */

    /* operators */
    double &operator[] (int n);	/* this[n] */

    /* data */
    PM_CARTESIAN tran;
    PM_QUATERNION rot;
};

/* PM_HOMOGENEOUS */

struct PM_HOMOGENEOUS {
    /* ctors/dtors */
    PM_HOMOGENEOUS() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_HOMOGENEOUS(PM_CCONST PM_HOMOGENEOUS & h);
#endif
    PM_HOMOGENEOUS(const PM_CARTESIAN& v, const PM_ROTATION_MATRIX& m);
    PM_HOMOGENEOUS(PM_CONST PM_POSE PM_REF p);	/* conversion */

    /* operators */
    PM_CARTESIAN & operator[](int n);	/* column vector */

    /* data ( [ 0 0 0 1 ] element is manually returned by [] if needed ) */
    PM_CARTESIAN tran;
    PM_ROTATION_MATRIX rot;
};

/* PM_LINE */

struct PM_LINE {
    /* ctors/dtors */
    PM_LINE() {
    };
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_LINE(PM_CCONST PM_LINE &);
#endif

    /* functions */
    int init(const PM_POSE& start, const PM_POSE& end);
    int point(double len, PM_POSE * point);

    /* data */
    PM_POSE start;		/* where motion was started */
    PM_POSE end;		/* where motion is going */
    PM_CARTESIAN uVec;		/* unit vector from start to end */
};

/* PM_CIRCLE */

struct PM_CIRCLE {
    /* ctors/dtors */
    PM_CIRCLE()
      : radius(0.0),
        angle(0.0),
        spiral(0.0)
    {};
#ifdef INCLUDE_POSEMATH_COPY_CONSTRUCTORS
    PM_CIRCLE(PM_CCONST PM_CIRCLE &);
#endif

    /* functions */
    int init(const PM_POSE& start, const PM_POSE& end,
	const PM_CARTESIAN& center, const PM_CARTESIAN& normal, int turn);
    int point(double angle, PM_POSE * point);

    /* data */
    PM_CARTESIAN center;
    PM_CARTESIAN normal;
    PM_CARTESIAN rTan;
    PM_CARTESIAN rPerp;
    PM_CARTESIAN rHelix;
    double radius;
    double angle;
    double spiral;
};

/* overloaded external functions */

/* dot */
extern double dot(const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);

/* cross */
extern PM_CARTESIAN cross(const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);

#if 0
/* norm */
extern PM_CARTESIAN norm(PM_CARTESIAN v);
extern PM_QUATERNION norm(PM_QUATERNION q);
extern PM_ROTATION_VECTOR norm(PM_ROTATION_VECTOR r);
extern PM_ROTATION_MATRIX norm(PM_ROTATION_MATRIX m);
#endif

/* unit */
extern PM_CARTESIAN unit(const PM_CARTESIAN &v);
extern PM_QUATERNION unit(const PM_QUATERNION &q);
extern PM_ROTATION_VECTOR unit(const PM_ROTATION_VECTOR &r);
extern PM_ROTATION_MATRIX unit(const PM_ROTATION_MATRIX &m);

/* isNorm */
extern int isNorm(const PM_CARTESIAN &v);
extern int isNorm(const PM_QUATERNION &q);
extern int isNorm(const PM_ROTATION_VECTOR &r);
extern int isNorm(const PM_ROTATION_MATRIX &m);

/* mag */
extern double mag(const PM_CARTESIAN &v);

/* disp */
extern double disp(const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);

/* inv */
extern PM_CARTESIAN inv(const PM_CARTESIAN &v);
extern PM_ROTATION_MATRIX inv(const PM_ROTATION_MATRIX &m);
extern PM_QUATERNION inv(const PM_QUATERNION &q);
extern PM_POSE inv(const PM_POSE &p);
extern PM_HOMOGENEOUS inv(const PM_HOMOGENEOUS &h);

/* project */
extern PM_CARTESIAN proj(const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);

/* overloaded arithmetic functions */

/* unary +, - for translation, rotation, pose */
extern PM_CARTESIAN operator + (const PM_CARTESIAN &v);
extern PM_CARTESIAN operator - (const PM_CARTESIAN &v);
extern PM_QUATERNION operator + (const PM_QUATERNION &q);
extern PM_QUATERNION operator - (const PM_QUATERNION &q);
extern PM_POSE operator + (const PM_POSE &p);
extern PM_POSE operator - (const PM_POSE &p);

/* compare operators */
extern int operator == (const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);
extern int operator == (const PM_QUATERNION &q1, const PM_QUATERNION &q2);
extern int operator == (const PM_POSE &p1, const PM_POSE &p2);
extern int operator != (const PM_CARTESIAN &v1, const PM_CARTESIAN &v2);
extern int operator != (const PM_QUATERNION &q1, const PM_QUATERNION &q2);
extern int operator != (const PM_POSE &p1, const PM_POSE &p2);

/* translation +, -, scalar *, - */

/* v + v */
extern PM_CARTESIAN operator + (PM_CARTESIAN v1, const PM_CARTESIAN &v2);
/* v - v */
extern PM_CARTESIAN operator - (PM_CARTESIAN v1, const PM_CARTESIAN &v2);
/* v * s */
extern PM_CARTESIAN operator *(PM_CARTESIAN v, double s);
/* s * v */
extern PM_CARTESIAN operator *(double s, PM_CARTESIAN v);
/* v / s */
extern PM_CARTESIAN operator / (const PM_CARTESIAN &v, double s);

/* rotation * by scalar, translation, and rotation */

/* s * q */
extern PM_QUATERNION operator *(double s, const PM_QUATERNION &q);
/* q * s */
extern PM_QUATERNION operator *(const PM_QUATERNION &q, double s);
/* q / s */
extern PM_QUATERNION operator / (const PM_QUATERNION &q, double s);
/* q * v */
extern PM_CARTESIAN operator *(const PM_QUATERNION &q, const PM_CARTESIAN &v);
/* q * q */
extern PM_QUATERNION operator *(const PM_QUATERNION &q1, const PM_QUATERNION &q2);
/* m * m */
extern PM_ROTATION_MATRIX operator *(const PM_ROTATION_MATRIX &m1,
    const PM_ROTATION_MATRIX &m2);

/* pose operators */

/* q * p */
extern PM_POSE operator *(const PM_QUATERNION &q, const PM_POSE &p);
/* p * p */
extern PM_POSE operator *(const PM_POSE &p1, const PM_POSE &p2);
/* p * v */
extern PM_CARTESIAN operator *(const PM_POSE &p, const PM_CARTESIAN &v);

#endif /* __cplusplus */

/* now comes the C stuff */

#ifdef __cplusplus
extern "C" {
#endif

/* PmCartesian */

    typedef struct {
	double x, y, z;		/* this.x, etc. */

    } PmCartesian;

/* PmSpherical */

    typedef struct {
	double theta, phi, r;

    } PmSpherical;

/* PmCylindrical */

    typedef struct {
	double theta, r, z;

    } PmCylindrical;

/* PmAxis */
#ifdef __cplusplus
    typedef PM_AXIS PmAxis;
#else
    typedef enum { PM_X, PM_Y, PM_Z } PmAxis;
#endif

/* PmRotationVector */

    typedef struct {
	double s, x, y, z;

    } PmRotationVector;

/* PmRotationMatrix */

    typedef struct {
	PmCartesian x, y, z;

    } PmRotationMatrix;

/* PmQuaternion */

    typedef struct {
	double s, x, y, z;	/* this.s, etc. */

    } PmQuaternion;

/* PmEulerZyz */

    typedef struct {
	double z, y, zp;

    } PmEulerZyz;

/* PmEulerZyx */

    typedef struct {
	double z, y, x;

    } PmEulerZyx;

/* PmRpy */

    typedef struct {
	double r, p, y;

    } PmRpy;

/* PmPose */

    typedef struct {
	PmCartesian tran;
	PmQuaternion rot;

    } PmPose;

/* PmCartLine */
    typedef struct {
        PmCartesian start;
        PmCartesian end;
        PmCartesian uVec;
        double tmag;
        int tmag_zero;
    } PmCartLine;

/* Homogeneous transform PmHomogeneous */

    typedef struct {
	PmCartesian tran;
	PmRotationMatrix rot;

    } PmHomogeneous;

/* line structure */

    typedef struct {
	PmPose start;		/* where motion was started */
	PmPose end;		/* where motion is going */
	PmCartesian uVec;	/* unit vector from start to end */
	PmQuaternion qVec;	/* unit of rotation */
	double tmag;
	double rmag;
	int tmag_zero;
	int rmag_zero;

    } PmLine;

/* Generalized circle structure */

    typedef struct {
	PmCartesian center;
	PmCartesian normal;
	PmCartesian rTan;
	PmCartesian rPerp;
	PmCartesian rHelix;
	double radius;
	double angle;
	double spiral;

    } PmCircle;

/*
   shorthand types for normal use-- don't define PM_LOOSE_NAMESPACE if these
   names are used by other headers you need to include and you don't want
   these shorthand versions
*/

/* some nice constants */

#define PM_PI      3.14159265358979323846
#define PM_PI_2    1.57079632679489661923
#define PM_PI_4    0.78539816339744830962
#define PM_2_PI    6.28318530717958647692

#ifdef PM_LOOSE_NAMESPACE

    typedef PmCartesian VECTOR;
    typedef PmSpherical SPHERICAL;
    typedef PmCylindrical CYLINDRICAL;
    typedef PmQuaternion QUATERNION;
    typedef PmRotationMatrix MATRIX;
    typedef PmEulerZyz ZYZ;
    typedef PmEulerZyx ZYX;
    typedef PmRpy RPY;
    typedef PmPose POSE;
    typedef PmHomogeneous HX;
    typedef PmCircle CIRCLE;
    typedef PmLine LINE;

#define PI                PM_PI
#define PI_2              PM_PI_2
#define PI_4              PM_PI_4
#define TWO_PI            PM_2_PI	/* 2_PI invalid macro name */
#endif

/* quicky macros */

//#define pmClose(a, b, eps) ((fabs((a) - (b)) < (eps)) ? 1 : 0)
//#define pmSq(x) ((x)*(x))

int pmClose(double a, double b, double eps); 
__attribute__((always_inline)) static inline double pmSq(double x) { return x*x; }

#ifdef TO_DEG
#undef TO_DEG
#endif
#define TO_DEG (180./PM_PI)

#ifdef TO_RAD
#undef TO_RAD
#endif
#define TO_RAD (PM_PI/180.)

/*! \todo FIXME-- fix these */

/* DOUBLE_FUZZ is the smallest double, d, such that (1+d != 1) w/o FPC.
   DOUBLECP_FUZZ is the same only with the Floating Point CoProcessor */

#define DOUBLE_FUZZ 2.2204460492503131e-16
#define DOUBLECP_FUZZ 1.0842021724855044e-19


/**
 * FIXME sloppily defined constants here.
 * These constants are quite large compared to the DOUBLE_FUZZ limitation. They
 * seem like an ugly band-aid for floating point problems.
 */

// FIXME setting this to be an order of magnitude smaller than canon's shortest
// allowed segment. This is still larger than TP's smallest position, so it may
// be silently causing trouble.
// andypugh 5/2/22 This seems to be interpreted to be in config units.
#define CART_FUZZ (1.0e-8)
/* how close a cartesian vector's magnitude must be for it to be considered
   a zero vector */

#define Q_FUZZ (1.0e-06)
/* how close elements of a Q must be to be equal */

#define QS_FUZZ (1.0e-6)
/* how close q.s is to 0 to be 180 deg rotation */

#define RS_FUZZ (1.0e-6)
/* how close r.s is for a rotation vector to be considered 0 */

#define QSIN_FUZZ (1.0e-6)
/* how close sin(a/2) is to 0 to be zero rotation */

#define V_FUZZ (1.0e-8)
/* how close elements of a V must be to be equal */

#define SQRT_FUZZ (-1.0e-6)
/* how close to 0 before math_sqrt() is error */

#define UNIT_VEC_FUZZ (1.0e-6)
/* how close mag of vec must be to 1.00 */

#define UNIT_QUAT_FUZZ (1.0e-6)
/* how close mag of quat must be to 1.00 */

#define UNIT_SC_FUZZ (1.0e-6)
/* how close mag of sin, cos must be to 1.00 */

#define E_EPSILON (1.0e-6)
/* how close second ZYZ euler angle must be to 0/PI for degeneration */

#define SINGULAR_EPSILON (1.0e-6)
/* how close to zero the determinate of a matrix must be for singularity */

#define RPY_P_FUZZ (1.0e-6)
/* how close pitch is to zero for RPY to degenerate */

#define ZYZ_Y_FUZZ (1.0e-6)
/* how close Y is to zero for ZYZ Euler to degenerate */

#define ZYX_Y_FUZZ (1.0e-6)
/* how close Y is to zero for ZYX Euler to degenerate */

#define CIRCLE_FUZZ (1.0e-6)
/* Bug fix for the missing circles problem */

/* debug output printing */
    extern void pmPrintError(const char *fmt, ...) __attribute__((format(printf,1,2)));

/* global error number and errors */
    extern int pmErrno;
    extern void pmPerror(const char *fmt);
#define PM_OK               0	/* no error */
#define PM_ERR             -1	/* unspecified error */
#define PM_IMPL_ERR        -2	/* not implemented */
#define PM_NORM_ERR        -3	/* arg should have been norm */
#define PM_DIV_ERR         -4	/* divide by zero error */

/* Scalar functions */

    extern double pmSqrt(double x);

/* Translation rep conversion functions */

    extern int pmCartSphConvert(PmCartesian const * const, PmSpherical * const);
    extern int pmCartCylConvert(PmCartesian const * const, PmCylindrical * const);
    extern int pmSphCartConvert(PmSpherical const * const, PmCartesian * const);
    extern int pmSphCylConvert(PmSpherical const * const, PmCylindrical * const);
    extern int pmCylCartConvert(PmCylindrical const * const, PmCartesian * const);
    extern int pmCylSphConvert(PmCylindrical const * const, PmSpherical * const);

/* Rotation rep conversion functions */

    extern int pmAxisAngleQuatConvert(PmAxis, double, PmQuaternion * const);

    extern int pmRotQuatConvert(PmRotationVector const * const, PmQuaternion * const);
    extern int pmRotMatConvert(PmRotationVector const * const, PmRotationMatrix * const);
    extern int pmRotZyzConvert(PmRotationVector const * const, PmEulerZyz * const);
    extern int pmRotZyxConvert(PmRotationVector const * const, PmEulerZyx * const);
    extern int pmRotRpyConvert(PmRotationVector const * const, PmRpy * const);

    extern int pmQuatRotConvert(PmQuaternion const * const, PmRotationVector * const);
    extern int pmQuatMatConvert(PmQuaternion const * const, PmRotationMatrix * const);
    extern int pmQuatZyzConvert(PmQuaternion const * const, PmEulerZyz * const);
    extern int pmQuatZyxConvert(PmQuaternion const * const, PmEulerZyx * const);
    extern int pmQuatRpyConvert(PmQuaternion const * const, PmRpy * const);

    extern int pmMatRotConvert(PmRotationMatrix const * const, PmRotationVector * const);
    extern int pmMatQuatConvert(PmRotationMatrix const * const, PmQuaternion * const);
    extern int pmMatZyzConvert(PmRotationMatrix const * const, PmEulerZyz * const);
    extern int pmMatZyxConvert(PmRotationMatrix const * const, PmEulerZyx * const);
    extern int pmMatRpyConvert(PmRotationMatrix const * const, PmRpy * const);

    extern int pmZyzRotConvert(PmEulerZyz const * const, PmRotationVector * const);
    extern int pmZyzQuatConvert(PmEulerZyz const * const, PmQuaternion * const);
    extern int pmZyzMatConvert(PmEulerZyz const * const, PmRotationMatrix * const);
    extern int pmZyzZyxConvert(PmEulerZyz const * const, PmEulerZyx * const);
    extern int pmZyzRpyConvert(PmEulerZyz const * const, PmRpy * const);

    extern int pmZyxRotConvert(PmEulerZyx const * const, PmRotationVector * const);
    extern int pmZyxQuatConvert(PmEulerZyx const * const, PmQuaternion * const);
    extern int pmZyxMatConvert(PmEulerZyx const * const, PmRotationMatrix * const);
    extern int pmZyxZyzConvert(PmEulerZyx const * const, PmEulerZyz * const);
    extern int pmZyxRpyConvert(PmEulerZyx const * const, PmRpy * const);

    extern int pmRpyRotConvert(PmRpy const * const, PmRotationVector * const);
    extern int pmRpyQuatConvert(PmRpy const * const, PmQuaternion * const);
    extern int pmRpyMatConvert(PmRpy const * const, PmRotationMatrix * const);
    extern int pmRpyZyzConvert(PmRpy const * const, PmEulerZyz * const);
    extern int pmRpyZyxConvert(PmRpy const * const, PmEulerZyx * const);

/* Combined rep conversion functions */

    extern int pmPoseHomConvert(PmPose const * const, PmHomogeneous* const);

    extern int pmHomPoseConvert(PmHomogeneous const * const, PmPose * const);

/* Arithmetic functions

   Note: currently, only functions for PmCartesian, PmQuaternion, and
   PmPose are supported directly. The type conversion functions
   will be used implicitly when applying arithmetic function
   to other types. This will be slower and less accurate. Explicit
   functions can be added incrementally.
*/

/* translation functions */

/* NOTE:  only Cartesian type supported in C now */

    extern int pmCartCartCompare(PmCartesian const * const, PmCartesian const * const);
    extern int pmCartCartDot(PmCartesian const * const, PmCartesian const * const, double * const);
    extern int pmCartCartCross(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartCartMult(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartCartDiv(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartInfNorm(PmCartesian const * v, double * out);
    extern int pmCartMag(PmCartesian const * const, double * const);
    extern int pmCartMagSq(PmCartesian const * const, double * const);
    extern int pmCartCartDisp(PmCartesian const * const v1, PmCartesian const * const v2, double *d);
    extern int pmCartCartAdd(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartCartSub(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartScalMult(PmCartesian const * const, double, PmCartesian * const);
    extern int pmCartScalDiv(PmCartesian const * const, double, PmCartesian * const);
    extern int pmCartNeg(PmCartesian const * const, PmCartesian * const);
    extern int pmCartUnit(PmCartesian const * const, PmCartesian * const);
    extern int pmCartAbs(PmCartesian const * const, PmCartesian * const);
    // Equivalent of compound operators like +=, -=, etc. Basically, these functions work directly on the first PmCartesian
    extern int pmCartCartAddEq(PmCartesian * const, PmCartesian const * const);
    extern int pmCartCartSubEq(PmCartesian * const, PmCartesian const * const);
    extern int pmCartScalMultEq(PmCartesian * const, double);
    extern int pmCartScalDivEq(PmCartesian * const, double);
    extern int pmCartUnitEq(PmCartesian * const);
    extern int pmCartNegEq(PmCartesian * const);
/*! \todo Another #if 0 */
#if 0
    extern int pmCartNorm(PmCartesian const * const v, PmCartesian * const vout);
#else
// Hopefully guaranteed to cause a compile error when used.
#define pmCartNorm(a,b,c,d,e)  bad{a.b.c.d.e}
#endif

    extern int pmCartIsNorm(PmCartesian const * const v);
    extern int pmCartInv(PmCartesian const * const, PmCartesian * const);
    extern int pmCartInvEq(PmCartesian * const);
    extern int pmCartCartProj(PmCartesian const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmCartPlaneProj(PmCartesian const * const v, PmCartesian const * const normal,
	PmCartesian * vout);

/* rotation functions */

/* quaternion functions */

    extern int pmQuatQuatCompare(PmQuaternion const * const, PmQuaternion const * const);
    extern int pmQuatMag(PmQuaternion const * const q, double *d);
    extern int pmQuatNorm(PmQuaternion const * const, PmQuaternion * const);
    extern int pmQuatInv(PmQuaternion const * const, PmQuaternion * const);
    extern int pmQuatIsNorm(PmQuaternion const * const);
    extern int pmQuatScalMult(PmQuaternion const * const q, double s, PmQuaternion * const qout);
    extern int pmQuatScalDiv(PmQuaternion const * const q, double s, PmQuaternion * const qout);
    extern int pmQuatQuatMult(PmQuaternion const * const, PmQuaternion const * const, PmQuaternion * const);
    extern int pmQuatCartMult(PmQuaternion const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmQuatAxisAngleMult(PmQuaternion const * const, PmAxis, double,
	PmQuaternion *);

/* rotation vector functions */

    extern int pmRotScalMult(PmRotationVector const * const, double, PmRotationVector * const);
    extern int pmRotScalDiv(PmRotationVector const * const, double, PmRotationVector * const);
    extern int pmRotIsNorm(PmRotationVector const * const);
    extern int pmRotNorm(PmRotationVector const * const, PmRotationVector * const);

/* rotation matrix functions */

/*        |  m.x.x   m.y.x   m.z.x  |   */
/*   M =  |  m.x.y   m.y.y   m.z.y  |   */
/*        |  m.x.z   m.y.z   m.z.z  |   */

    extern int pmMatNorm(PmRotationMatrix const * const m, PmRotationMatrix * const mout);
    extern int pmMatIsNorm(PmRotationMatrix const * const  m);
    extern int pmMatInv(PmRotationMatrix const * const  m, PmRotationMatrix * const mout);
    extern int pmMatCartMult(PmRotationMatrix const * const  m, PmCartesian const * const  v,
	PmCartesian * const vout);
    extern int pmMatMatMult(PmRotationMatrix const * const  m1, PmRotationMatrix const * const  m2,
	PmRotationMatrix * const mout);

/* pose functions*/

    extern int pmPosePoseCompare(PmPose const * const, PmPose const * const);
    extern int pmPoseInv(PmPose const * const p, PmPose * const);
    extern int pmPoseCartMult(PmPose const * const, PmCartesian const * const, PmCartesian * const);
    extern int pmPosePoseMult(PmPose const * const, PmPose const * const, PmPose * const);

/* homogeneous functions */
    extern int pmHomInv(PmHomogeneous const * const, PmHomogeneous * const);

/* line functions */

    extern int pmLineInit(PmLine * const line, PmPose const * const start, PmPose const * const end);
    extern int pmLinePoint(PmLine const * const line, double len, PmPose * const point);

/* pure cartesian line functions */
    extern int pmCartLineInit(PmCartLine * const line, PmCartesian const * const start, PmCartesian const * const end);
    extern int pmCartLinePoint(PmCartLine const * const line, double len, PmCartesian * const point);
    extern int pmCartLineStretch(PmCartLine * const line, double new_len, int from_end);

/* circle functions */

    extern int pmCircleInit(PmCircle * const circle,
            PmCartesian const * const start, PmCartesian const * const end,
            PmCartesian const * const center, PmCartesian const * const normal, int turn);

    extern int pmCirclePoint(PmCircle const * const circle, double angle, PmCartesian * const point);
    extern int pmCircleStretch(PmCircle * const circ, double new_angle, int from_end);

/* slicky macros for item-by-item copying between C and C++ structs */

#ifdef __cplusplus
}				/* matches extern "C" for C++ */

template <class A, class B>
void toCart(const A& src, B* dst) {(dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

template <class A, class B>
void toCyl(const A& src, B* dst) {(dst)->theta = (src).theta; (dst)->r = (src).r; (dst)->z = (src).z;}

template <class A, class B>
void toSph(const A& src, B* dst) {(dst)->theta = (src).theta; (dst)->phi = (src).phi; (dst)->r = (src).r;}

template <class A, class B>
void toQuat(const A& src, B* dst) {(dst)->s = (src).s; (dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

template <class A, class B>
void toRot(const A& src, B* dst) {(dst)->s = (src).s; (dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z;}

template <class A, class B>
void toMat(const A& src, B* dst) {toCart((src).x, &((dst)->x)); toCart((src).y, &((dst)->y)); toCart((src).z, &((dst)->z));}

template <class A, class B>
void toEulerZyz(const A& src, B* dst) {(dst)->z = (src).z; (dst)->y = (src).y; (dst)->zp = (src).zp;}

template <class A, class B>
void toEulerZyx(const A& src, B* dst) {(dst)->z = (src).z; (dst)->y = (src).y; (dst)->x = (src).x;}

template <class A, class B>
void toRpy(const A& src, B* dst) {(dst)->r = (src).r; (dst)->p = (src).p; (dst)->y = (src).y;}

template <class A, class B>
void toPose(const A& src, B* dst) {toCart((src).tran, &((dst)->tran)); toQuat((src).rot, &((dst)->rot));}

template <class A, class B>
void toHom(const A& src, B* dst) {toCart((src).tran, &((dst)->tran)); toMat((src).rot, &((dst)->rot));}

template <class A, class B>
void toLine(const A& src, B* dst) {toPose((src).start, &((dst)->start)); toPose((src).end, &((dst)->end)); toCart((src).uVec, &((dst)->uVec));}

template <class A, class B>
void toCircle(const A& src, B* dst) {toCart((src).center, &((dst)->center)); toCart((src).normal, &((dst)->normal)); toCart((src).rTan, &((dst)->rTan)); toCart((src).rPerp, &((dst)->rPerp)); toCart((src).rHelix, &((dst)->rHelix)); (dst)->radius = (src).radius; (dst)->angle = (src).angle; (dst)->spiral = (src).spiral;}

#endif
#endif				/* #ifndef POSEMATH_H */
