/* -------------------------------------------------------------------------
* SvzAlgebra.h -- Basic algebra header file
* --------------------------------------------------------------------------
* Copyright (c) 1995 - 2011 soVoz Inc. 
* -------------------------------------------------------------------------- 
* Tiantian Liu, Liming Zhao, Joey Schnurr, Stephen Lane 
*/

//
// References and Sources:
//
// C++ Vector and Matrix Algebra routines						
// Author: Jean-Francois DOUE									
// Version 3.1 --- October 1993									
//
//
//	"Graphics Gems IV",  Edited by Paul S. Heckbert
//	Academic Press, 1994, ISBN 0-12-336156-9
//	"You are free to use and modify this code in any way 
//	you like." (p. xv)
//
//	Modifications by J. Nagle, March 1997
//  Modifications by Liming Zhao and Stephen Lane, 2006 - 2011

#pragma once

#ifndef ALGEBRA_H
#define ALGEBRA_H

#define ALGEBRAIOSTREAMS

#pragma warning(disable: 4244)
#include <iostream>
#include <assert.h>
#include <cmath>

using namespace std;

#ifndef Real
#define Real float
#endif

#ifndef SOVOZMATH
#ifndef M_PI
const float M_PI = 3.14159265358979323846f;		// per CRC handbook, 14th. ed.
#endif
#ifndef M_PI_2
const float M_PI_2 = (M_PI/2.0f);				// PI/2
#endif
const float M2_PI = (M_PI*2.0f);				// PI*2
const float Rad2Deg = (180.0f / M_PI);			// Rad to Degree
const float Deg2Rad = (M_PI / 180.0f);	        // Degree to Rad

enum {PA, PB, PC, PD};		    // planes
enum {RED, GREEN, BLUE};	    // colors
enum {KA, KD, KS, ES};		    // phong coefficients
enum {VX, VY, VZ, VW};		    // axes
// SOVOZMATH defined at end of file
#endif

#ifndef EPSILON
#define EPSILON 0.00001
#endif

enum {vx, vy, vz, vw};		    // axes


static unsigned int SVZ_JOINT_DOF_X = 0x0004;
static unsigned int SVZ_JOINT_DOF_Y = 0x0002;
static unsigned int SVZ_JOINT_DOF_Z = 0x0001;

// this line defines a new type: pointer to a function which returns a
// Real and takes as argument a Real
typedef Real (*Svz_V_FCT_PTR)(Real);

// min-max macros
#define MIN(A,B) ((A) < (B) ? (A) : (B))
#define MAX(A,B) ((A) > (B) ? (A) : (B))

#undef min					// allow as function names
#undef max

// error handling macro
#define ALGEBRA_ERROR(E) { assert(false); }

class vec2;
class vec3;
class vec4;
class mat3;
class mat4;
class quat;

//#define ALGEBRAIOSTREAMS

/****************************************************************
*																*
*			    2D Vector										*
*																*
****************************************************************/

class vec2
{
protected:

	Real n[2];

public:

	// Constructors
	vec2();
	vec2(const Real x, const Real y);
	vec2(const Real d);
	vec2(const vec2& v);				// copy constructor
	vec2(const vec3& v);				// cast v3 to v2
	vec2(const vec3& v, int dropAxis);	// cast v3 to v2

	// Assignment operators
	vec2& operator	= ( const vec2& v );	// assignment of a vec2
	vec2& operator += ( const vec2& v );	// incrementation by a vec2
	vec2& operator -= ( const vec2& v );	// decrementation by a vec2
	vec2& operator *= ( const Real d );	// multiplication by a constant
	vec2& operator /= ( const Real d );	// division by a constant
	Real& operator [] ( int i);			// indexing
	Real operator [] ( int i) const;// read-only indexing

	// special functions
	Real Length() const;			// length of a vec2
	Real Length2() const;			// squared length of a vec2
	vec2& Normalize() ;				// normalize a vec2 in place
	vec2& apply(Svz_V_FCT_PTR fct);		// apply a func. to each component
	int dim() const;								// returns dimension of vector

	// friends
	friend vec2 operator- (const vec2& v);						// -v1
	friend vec2 operator+ (const vec2& a, const vec2& b);	    // v1 + v2
	friend vec2 operator- (const vec2& a, const vec2& b);	    // v1 - v2
	friend vec2 operator* (const vec2& a, const Real d);	    // v1 * 3.0
	friend vec2 operator* (const Real d, const vec2& a);	    // 3.0 * v1
	friend vec2 operator* (const mat3& a, const vec2& v);	    // M . v
	friend vec2 operator* (const vec2& v, const mat3& a);		// v . M
	friend Real operator* (const vec2& a, const vec2& b);    // dot product
	friend vec2 operator/ (const vec2& a, const Real d);	    // v1 / 3.0
	friend vec3 operator^ (const vec2& a, const vec2& b);	    // cross product
	friend int operator== (const vec2& a, const vec2& b);	    // v1 == v2 ?
	friend int operator!= (const vec2& a, const vec2& b);	    // v1 != v2 ?

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const vec2& v);	// output to stream
	//	friend istream& operator >> (istream& s, vec2& v);	    // input from strm.
#endif ALGEBRAIOSTREAMS

	friend void swap(vec2& a, vec2& b);						// swap v1 & v2
	friend vec2 min(const vec2& a, const vec2& b);		    // min(v1, v2)
	friend vec2 max(const vec2& a, const vec2& b);		    // max(v1, v2)
	friend vec2 prod(const vec2& a, const vec2& b);		    // term by term *

	// necessary friend declarations
	friend class vec3;
};

/****************************************************************
*																*
*			    3D Vector										*
*																*
****************************************************************/

class vec3
{
protected:

	Real n[3];

public:

	// Constructors
	vec3();
	vec3(const Real x, const Real y, const Real z);
	vec3(const Real d);
	vec3(const vec3& v);						// copy constructor
	vec3(const vec2& v);						// cast v2 to v3
	vec3(const vec2& v, Real d);		    // cast v2 to v3
	vec3(const vec4& v);						// cast v4 to v3
	vec3(const vec4& v, int dropAxis);	    // cast v4 to v3
	vec3(const quat& q);						// cast quat to  v3 *** SHL added 11/25/10

	// Assignment operators
	vec3& operator	= ( const vec3& v );	    // assignment of a vec3
	vec3& operator += ( const vec3& v );	    // incrementation by a vec3
	vec3& operator -= ( const vec3& v );	    // decrementation by a vec3
	vec3& operator *= ( const Real d );	    // multiplication by a constant
	vec3& operator /= ( const Real d );	    // division by a constant
	Real& operator [] ( int i);					// indexing
	Real operator[] (int i) const;				// read-only indexing

	// special functions
	Real Length() const;						// length of a vec3
	Real Length2() const;						// squared length of a vec3
	vec3& Normalize();							// normalize a vec3 in place
	vec3& apply(Svz_V_FCT_PTR fct);				// apply a func. to each component
	vec3 Cross(vec3 &v) const;				// cross product
	int dim() const;								// returns dimension of vector
	Real* getn();

	// friends
	friend vec3 operator - (const vec3& v);						// -v1
	friend vec3 operator + (const vec3& a, const vec3& b);	    // v1 + v2
	friend vec3 operator - (const vec3& a, const vec3& b);	    // v1 - v2
	friend vec3 operator * (const vec3& a, const Real d);	    // v1 * 3.0
	friend vec3 operator * (const Real d, const vec3& a);	    // 3.0 * v1
	friend vec3 operator * (const mat4& a, const vec3& v);	    // M . v
	friend vec3 operator * (const vec3& v, const mat4& a);		// v . M
	friend Real operator * (const vec3& a, const vec3& b);    // dot product
	friend vec3 operator / (const vec3& a, const Real d);	    // v1 / 3.0
	friend vec3 operator ^ (const vec3& a, const vec3& b);	    // cross product
	friend int operator == (const vec3& a, const vec3& b);	    // v1 == v2 ?
	friend int operator != (const vec3& a, const vec3& b);	    // v1 != v2 ?

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const vec3& v);	   // output to stream
	friend istream& operator >> (istream& s, vec3& v);	    // input from strm.
#endif // ALGEBRAIOSTREAMS

	friend void swap(vec3& a, vec3& b);						// swap v1 & v2
	friend vec3 min(const vec3& a, const vec3& b);		    // min(v1, v2)
	friend vec3 max(const vec3& a, const vec3& b);		    // max(v1, v2)
	friend vec3 prod(const vec3& a, const vec3& b);		    // term by term *

	// necessary friend declarations
	friend class vec2;
	friend class vec4;
	friend class quat;
	friend class mat3;
	friend vec2 operator * (const mat3& a, const vec2& v);	// linear transform
	friend vec3 operator * (const mat3& a, const vec3& v);
	friend mat3 operator * (const mat3& a, const mat3& b);	// matrix 3 product
};

#ifndef SOVOZMATH
const vec3 Xaxis(1.0f, 0.0f, 0.0f);
const vec3 Yaxis(0.0f, 1.0f, 0.0f);
const vec3 Zaxis(0.0f, 0.0f, 1.0f);
#endif

/****************************************************************
*																*
*			    4D Vector										*
*																*
****************************************************************/

class vec4
{
public:

	Real n[4];

public:

	// Constructors
	vec4();
	vec4(const Real x, const Real y, const Real z, const Real w);
	vec4(const Real d);
	vec4(const vec4& v);			    // copy constructor
	vec4(const vec3& v);			    // cast vec3 to vec4
	vec4(const vec3& v, const Real d);	    // cast vec3 to vec4

	// Assignment operators
	vec4& operator	= ( const vec4& v );	    // assignment of a vec4
	vec4& operator += ( const vec4& v );	    // incrementation by a vec4
	vec4& operator -= ( const vec4& v );	    // decrementation by a vec4
	vec4& operator *= ( const Real d );	    // multiplication by a constant
	vec4& operator /= ( const Real d );	    // division by a constant
	Real& operator [] ( int i);				// indexing
	Real operator[] (int i) const;			// read-only indexing

	// special functions
	Real Length() const;			// length of a vec4
	Real Length2() const;			// squared length of a vec4
	vec4& Normalize();			    // normalize a vec4 in place
	vec4& apply(Svz_V_FCT_PTR fct);		// apply a func. to each component
	int dim() const;								// returns dimension of vector

	// friends
	friend vec4 operator - (const vec4& v);						// -v1
	friend vec4 operator + (const vec4& a, const vec4& b);	    // v1 + v2
	friend vec4 operator - (const vec4& a, const vec4& b);	    // v1 - v2
	friend vec4 operator * (const vec4& a, const Real d);	    // v1 * 3.0
	friend vec4 operator * (const Real d, const vec4& a);	    // 3.0 * v1
	friend vec4 operator * (const mat4& a, const vec4& v);	    // M . v
	friend vec4 operator * (const vec4& v, const mat4& a);	    // v . M
	friend Real operator * (const vec4& a, const vec4& b);    // dot product
	friend vec4 operator / (const vec4& a, const Real d);	    // v1 / 3.0
	friend int operator == (const vec4& a, const vec4& b);	    // v1 == v2 ?
	friend int operator != (const vec4& a, const vec4& b);	    // v1 != v2 ?

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const vec4& v);	// output to stream
	// friend istream& operator >> (istream& s, vec4& v);	    // input from strm.
#endif //  ALGEBRAIOSTREAMS

	friend void swap(vec4& a, vec4& b);						// swap v1 & v2
	friend vec4 min(const vec4& a, const vec4& b);		    // min(v1, v2)
	friend vec4 max(const vec4& a, const vec4& b);		    // max(v1, v2)
	friend vec4 prod(const vec4& a, const vec4& b);		    // term by term *

	// necessary friend declarations
	friend class vec3;
	friend class quat;
	friend class mat4;
	friend vec3 operator * (const mat4& a, const vec3& v);	// linear transform
	friend mat4 operator * (const mat4& a, const mat4& b);	// matrix 4 product
};

/****************************************************************
*																*
*			   3x3 Matrix										*
*																*
****************************************************************/

class mat3
{
protected:

	vec3 v[3];

public:

	// Constructors
	mat3();
	mat3(const vec3& v0, const vec3& v1, const vec3& v2);
	mat3(const Real d);
	mat3(const mat3& m);
	mat3(const mat4& m);

	// Static functions
	static mat3 Identity();
	static mat3 Translation2D(const vec2& v);
	static mat3 Rotation2DDeg(const vec2& center, const Real angleDeg);
	static mat3 Rotation2DRad(const vec2& center, const Real angleRad);
	static mat3 Scaling2D(const vec2& scaleVector);
	static mat3 Rotation3DDeg(const vec3& axis, const Real angleDeg);
	static mat3 Rotation3DRad(const vec3& axis, const Real angleRad);
	static mat3 Rotation3DDeg(const int Axis, const Real angleDeg);
	static mat3 Rotation3DRad(const int Axis, const Real angleRad);
	static mat3 Slerp(const mat3& rot0, const mat3& rot1, const Real& fPerc);
	static mat3 Lerp(const mat3& rot0, const mat3& rot1, const Real& fPerc);

	// Rotation operations, matrix must be orthonomal
	bool ToEulerAnglesXYZ(vec3& anglesRad) const;
	bool ToEulerAnglesXZY(vec3& anglesRad) const;
	bool ToEulerAnglesYXZ(vec3& anglesRad) const;
	bool ToEulerAnglesYZX(vec3& anglesRad) const;
	bool ToEulerAnglesZXY(vec3& anglesRad) const;
	bool ToEulerAnglesZYX(vec3& anglesRad) const;
	mat3 FromEulerAnglesXYZ(const vec3& anglesRad);
	mat3 FromEulerAnglesXZY(const vec3& anglesRad);
	mat3 FromEulerAnglesYXZ(const vec3& anglesRad);
	mat3 FromEulerAnglesYZX(const vec3& anglesRad);
	mat3 FromEulerAnglesZXY(const vec3& anglesRad);
	mat3 FromEulerAnglesZYX(const vec3& anglesRad);

	// Conversion with quat
	quat ToQuaternion() const;
	void FromQuaternion(const quat& q);
	void ToAxisAngle(vec3& axis, Real& angleRad) const;
	void ToAxisAngle2(vec3& axis, Real& angleRad) const;
	void FromAxisAngle(const vec3& axis, const Real& angleRad);

	// Assignment operators
	mat3& operator	= ( const mat3& m );	    // assignment of a mat3
	mat3& operator += ( const mat3& m );	    // incrementation by a mat3
	mat3& operator -= ( const mat3& m );	    // decrementation by a mat3
	mat3& operator *= ( const Real d );	    // multiplication by a constant
	mat3& operator /= ( const Real d );	    // division by a constant
	vec3& operator [] ( int i);					// indexing
	const vec3& operator [] ( int i) const;		// read-only indexing

	// special functions
	mat3 transpose() const;								// transpose
	mat3 inverse() const;								// inverse
	mat3& apply(Svz_V_FCT_PTR fct);							// apply a func. to each element
	void getData(Real* d);							// turn into 4x4 opengl matrix
	bool reorthogonalize();								// Gram-Schmidt orthogonalization
	//void getRow(unsigned int axis, vec3& rowVec) const;	// get a particular row
	//void getCol(unsigned int axis, vec3& colVec) const;	// get a particular col
	vec3 getRow(unsigned int axis) const;	// get a particular row
	vec3 getCol(unsigned int axis) const;	// get a particular col
	void setRow(unsigned int axis, const vec3& rowVec);	// set a particular row
	void setCol(unsigned int axis, const vec3& colVec);	// set a particular col
	mat3 Normalize(int axis);
	vec3 GetYawPitchRoll(unsigned int leftAxis, unsigned int upAixs, unsigned int frontAxis) const;

	// OpenGL related functions - see Note at bottom of file
	void ToGLMatrix(Real* pData);
	void WriteToGLMatrix(Real* m);							// turn rotational data into 4x4 opengl matrix with zero translation
	void ReadFromGLMatrix(Real* m);	


	// friends
	friend mat3 operator - (const mat3& a);						// -m1
	friend mat3 operator + (const mat3& a, const mat3& b);	    // m1 + m2
	friend mat3 operator - (const mat3& a, const mat3& b);	    // m1 - m2
	friend mat3 operator * (const mat3& a, const mat3& b);		// m1 * m2
	friend mat3 operator * (const mat3& a, const Real d);	    // m1 * 3.0
	friend mat3 operator * (const Real d, const mat3& a);	    // 3.0 * m1
	friend mat3 operator / (const mat3& a, const Real d);	    // m1 / 3.0
	friend int operator == (const mat3& a, const mat3& b);	    // m1 == m2 ?
	friend int operator != (const mat3& a, const mat3& b);	    // m1 != m2 ?

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const mat3& m);	// output to stream
	friend istream& operator >> (istream& s, mat3& m);	    // input from strm.
#endif //  ALGEBRAIOSTREAMS

	friend void swap(mat3& a, mat3& b);			    // swap m1 & m2

	// necessary friend declarations
	friend vec3 operator * (const mat3& a, const vec3& v);	    // linear transform
	friend vec2 operator * (const mat3& a, const vec2& v);	    // linear transform
};

/****************************************************************
*																*
*			   4x4 Matrix										*
*																*
****************************************************************/

class mat4
{
protected:

	vec4 v[4];

public:

	// Constructors
	mat4();
	mat4(const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3);
	mat4(const Real d);
	mat4(const mat4& m);
	mat4(const mat3& m);
	mat4(const mat3& m, const vec3& t);
	mat4(const Real* d);

	// Static functions
	static mat4 identity();
	static mat4 translation3D(const vec3& v);
	static mat4 rotation3DDeg(const vec3& axis, const Real angleDeg);
	static mat4 rotation3DRad(const vec3& axis, const Real angleRad);
	static mat4 scaling3D(const vec3& scaleVector);
	static mat4 perspective3D(const Real d);

	// Assignment operators
	mat4& operator	= ( const mat4& m );	    // assignment of a mat4
	mat4& operator += ( const mat4& m );	    // incrementation by a mat4
	mat4& operator -= ( const mat4& m );	    // decrementation by a mat4
	mat4& operator *= ( const Real d );	    // multiplication by a constant
	mat4& operator /= ( const Real d );	    // division by a constant
	vec4& operator [] ( int i);					// indexing
	const vec4& operator [] ( int i) const;		// read-only indexing

	// special functions
	mat4 transpose() const;						// transpose
	mat4 inverse() const;						// inverse
	mat4& apply(Svz_V_FCT_PTR fct);					// apply a func. to each element
	void getData(Real* d);

	// friends
	friend mat4 operator - (const mat4& a);						// -m1
	friend mat4 operator + (const mat4& a, const mat4& b);	    // m1 + m2
	friend mat4 operator - (const mat4& a, const mat4& b);	    // m1 - m2
	friend mat4 operator * (const mat4& a, const mat4& b);		// m1 * m2
	friend mat4 operator * (const mat4& a, const Real d);	    // m1 * 4.0
	friend mat4 operator * (const Real d, const mat4& a);	    // 4.0 * m1
	friend mat4 operator / (const mat4& a, const Real d);	    // m1 / 3.0
	friend int operator == (const mat4& a, const mat4& b);	    // m1 == m2 ?
	friend int operator != (const mat4& a, const mat4& b);	    // m1 != m2 ?

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const mat4& m);	// output to stream
	//	friend istream& operator >> (istream& s, mat4& m);			// input from strm.
#endif //  ALGEBRAIOSTREAMS

	friend void swap(mat4& a, mat4& b);							// swap m1 & m2

	// necessary friend declarations
	friend vec4 operator * (const mat4& a, const vec4& v);	    // linear transform
	friend vec3 operator * (const mat4& a, const vec3& v);	    // linear transform
};

/****************************************************************
*																*
*		    quat          								*
*																*
****************************************************************/


class quat
{
protected:

	Real n[4];

	// Used by Slerp
	static Real CounterWarp(Real t, Real fCos);
	static Real ISqrt_approx_in_neighborhood(Real s);

public:

	// Constructors
	quat();
	quat(Real q[4]);	// Format is W,X,Y,Z
	quat(const Real w, const Real x, const Real y, const Real z);
	quat(const quat& q);
	quat(const vec4& v);
	quat(const vec3& v);  //*** SHL added 11/25/10
	quat(const Real d);  //*** SHL added 11/25/10

	// Static functions
	static Real Dot(const quat& q0, const quat& q1);
	static quat Exp(const quat& q);
	static quat Log(const quat& q);
	static quat UnitInverse(const quat& q);
	static quat Slerp(Real t, const quat& q0, const quat& q1);
	static quat Intermediate (const quat& q0, const quat& q1, const quat& q2);
	static quat Squad(Real t, const quat& q0, const quat& a, const quat& b, const quat& q1);
	static quat ProjectToAxis(const quat& q, vec3 axis);
	static quat ProjectToAxis2(const quat& q, vec3 axis);

	// Conversion functions
	void ToAxisAngle (vec3& axis, Real& angleRad) const;
	void FromAxisAngle (const vec3& axis, Real angleRad);

	void FromAxisXAngle(Real angleRad);
	void FromAxisYAngle(Real angleRad);
	void FromAxisZAngle(Real angleRad);

	mat3 ToRotation () const;
	void FromRotation (const mat3& rot);

	// Assignment operators
	quat& operator = (const quat& q);	// assignment of a quaternion
	quat& operator += (const quat& q);	// summation with a quaternion
	quat& operator -= (const quat& q);	// subtraction with a quaternion
	quat& operator *= (const quat& q);	// multiplication by a quaternion
	quat& operator *= (const Real d);		// multiplication by a scalar
	quat& operator /= (const Real d);		// division by a scalar

	// Indexing
	Real& operator[](int i); //*** SHL change 11/25/10 
	Real operator[](int i) const;

	Real& W();
	Real W() const;
	Real& X();
	Real X() const;
	Real& Y();
	Real Y() const;
	Real& Z();
	Real Z() const;

	// Friends
	friend quat operator - (const quat& q);							// -q
	friend quat operator + (const quat& q0, const quat& q1);	    // q0 + q1
	friend quat operator - (const quat& q0, const quat& q1);	// q0 - q1
	friend quat operator * (const quat& q, const Real d);			// q * 3.0
	friend quat operator * (const Real d, const quat& q);			// 3.0 * v
	friend quat operator * (const quat& q0, const quat& q1);  // q0 * q1
	friend quat operator / (const quat& q, const Real d);			// q / 3.0
	friend bool operator == (const quat& q0, const quat& q1);		// q0 == q1 ?
	friend bool operator != (const quat& q0, const quat& q1);		// q0 != q1 ?

	// Special functions
	Real Length() const;
	Real Length2() const;
	quat& Normalize();
	quat& fastNormalize();
	quat inverse() const;
	void Zero();
	int dim() const;								// returns dimension of vector

#ifdef ALGEBRAIOSTREAMS
	friend ostream& operator << (ostream& s, const quat& q);	// output to stream
#endif //  ALGEBRAIOSTREAMS

	//necessary friend declarations
	friend class vec3;
	friend class mat4;
	friend mat3;
};

// Some functions for handling computation on angles
inline void clampAngle(Real& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

inline Real AngleDiff(Real angle0, Real angle1)
{
	Real value = angle0 - angle1;
	clampAngle(value);
	return value;
}

inline Real Lerp(Real v0, Real v1, Real fPerc)
{
	return v0 + fPerc * (v1 - v0);
}

#ifndef SOVOZMATH
#define SOVOZMATH
#endif

#endif


//OpenGL transformation matrix
//	Rx =
//	|1       0        0    Tx|
//	|0  cos(a)  -sin(a)    Ty|
//	|0  sin(a)   cos(a)    Tz|
//	|0       0        0    1 |
//
//	Ry =
//	| cos(a)  0  sin(a)    Tx|
//	|      0  1       0    Ty|
//	|-sin(a)  0  cos(a)    Tz|
//	|      0  0       0    1 |
//
//	Rz = 
//	|cos(a)  -sin(a)  0   Tx|
//	|sin(a)   cos(a)  0   Ty|
//	|     0        0  1   Tz|
//	|     0        0  0   1 |
//
// However, when they are stored in OpenGL matrix, they are stored column major
// OpenGL convention
// m[0] = R[0][0]; m[4] = R[0][1]; m[8]  = R[0][2]; m[12] = Tx;
// m[1] = R[1][0]; m[5] = R[1][1]; m[9]  = R[1][2]; m[13] = Ty;
// m[2] = R[2][0]; m[6] = R[2][1]; m[10] = R[2][2]; m[14] = Tz;
// m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
