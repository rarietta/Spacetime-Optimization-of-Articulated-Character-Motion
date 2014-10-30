/* -------------------------------------------------------------------------
* SvzAlgebra.h -- Basic algebra source file
* --------------------------------------------------------------------------
* Copyright (c) 1995 - 2011 Sovoz Inc.  All Rights Reserved
* -------------------------------------------------------------------------- 
* Liming Zhao, Stephen Lane
*/

#include "SvzAlgebra.h"

//
//	Implementation
//

/****************************************************************
*																*
*		    vec2 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec2::vec2()
{
}

vec2::vec2(const Real x, const Real y)
{
	n[vx] = x; n[vy] = y; 
}

vec2::vec2(const Real d)
{ 
	n[vx] = n[vy] = d; 
}

vec2::vec2(const vec2& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; 
}

vec2::vec2(const vec3& v) // it is up to caller to avoid divide-by-zero
{ 
	n[vx] = v.n[vx]/v.n[vz]; n[vy] = v.n[vy]/v.n[vz]; 
};

vec2::vec2(const vec3& v, int dropAxis) {
	switch (dropAxis) {
	case vx: n[vx] = v.n[vy]; n[vy] = v.n[vz]; break;
	case vy: n[vx] = v.n[vx]; n[vy] = v.n[vz]; break;
	default: n[vx] = v.n[vx]; n[vy] = v.n[vy]; break;
	}
}


// ASSIGNMENT OPERATORS

vec2& vec2::operator = (const vec2& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; return *this; 
}

vec2& vec2::operator += ( const vec2& v )
{ 
	n[vx] += v.n[vx]; n[vy] += v.n[vy]; return *this; 
}

vec2& vec2::operator -= ( const vec2& v )
{ 
	n[vx] -= v.n[vx]; n[vy] -= v.n[vy]; return *this; 
}

vec2& vec2::operator *= ( const Real d )
{ 
	n[vx] *= d; n[vy] *= d; return *this; 
}

vec2& vec2::operator /= ( const Real d )
{ 
	Real d_inv = 1./d; n[vx] *= d_inv; n[vy] *= d_inv; return *this; 
}

Real& vec2::operator [] ( int i)
{
	assert(!(i < vx || i > vy));		// subscript check
	return n[i];
}

Real vec2::operator [] ( int i) const
{
	assert(!(i < vx || i > vy));
	return n[i];
}


// SPECIAL FUNCTIONS

Real vec2::Length() const
{ 
	return sqrt(Length2()); 
}

Real vec2::Length2() const
{ 
	return n[vx]*n[vx] + n[vy]*n[vy]; 
}

vec2& vec2::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	//*this /= Length(); return *this; 

	Real norm = Length();
	if (norm < EPSILON)
		return *this = vec2(1.0f, 0.0f);
	else return *this /= norm;
}

vec2& vec2::apply(Svz_V_FCT_PTR fct)
{ 
	n[vx] = (*fct)(n[vx]); n[vy] = (*fct)(n[vy]); return *this; 
}

int vec2::dim() const								// SHL added - returns dimension of vector
{
	return (sizeof(n)/sizeof(Real));
}

// FRIENDS

vec2 operator - (const vec2& a)
{ 
	return vec2(-a.n[vx],-a.n[vy]);
}

vec2 operator + (const vec2& a, const vec2& b)
{ 
	return vec2(a.n[vx]+ b.n[vx], a.n[vy] + b.n[vy]);
}

vec2 operator - (const vec2& a, const vec2& b)
{ 
	return vec2(a.n[vx]-b.n[vx], a.n[vy]-b.n[vy]);
}

vec2 operator * (const vec2& a, const Real d)
{ 
	return vec2(d*a.n[vx], d*a.n[vy]);
}

vec2 operator * (const Real d, const vec2& a)
{ 
	return a*d; 
}

vec2 operator * (const mat3& a, const vec2& v)
{
	vec3 av;
	av.n[vx] = a.v[0].n[vx]*v.n[vx] + a.v[0].n[vy]*v.n[vy] + a.v[0].n[vz];
	av.n[vy] = a.v[1].n[vx]*v.n[vx] + a.v[1].n[vy]*v.n[vy] + a.v[1].n[vz];
	av.n[vz] = a.v[2].n[vx]*v.n[vx] + a.v[2].n[vy]*v.n[vy] + a.v[2].n[vz];
	return av;
}

vec2 operator * (const vec2& v, const mat3& a)
{ 
	return a.transpose() * v; 
}

Real operator * (const vec2& a, const vec2& b)
{ 
	return (a.n[vx]*b.n[vx] + a.n[vy]*b.n[vy]); 
}

vec2 operator / (const vec2& a, const Real d)
{ 
	Real d_inv = 1./d; return vec2(a.n[vx]*d_inv, a.n[vy]*d_inv);
}

vec3 operator ^ (const vec2& a, const vec2& b)
{ 
	return vec3(0.0, 0.0, a.n[vx] * b.n[vy] - b.n[vx] * a.n[vy]);
}

int operator == (const vec2& a, const vec2& b)
{ 
	return (a.n[vx] == b.n[vx]) && (a.n[vy] == b.n[vy]); 
}

int operator != (const vec2& a, const vec2& b)
{ 
	return !(a == b); 
}

#ifdef ALGEBRAIOSTREAMS
ostream& operator << (ostream& s, const vec2& v)
{ 
	return s << "[ " << v.n[vx] << ' ' << v.n[vy] << " ]"; 
}
// istream& operator >> (istream& s, vec2& v)
//{
//	vec2	v_tmp;
//	char	c = ' ';
//
//	while (isspace(c))
//		s >> c;
//	// The vectors can be formatted either as x y or | x y |
//	if (c == '[') {
//		s >> v_tmp[vx] >> v_tmp[vy];
//		while (s >> c && isspace(c)) ;
//		if (c != ']')
//			s.set(_bad);
//	}
//	else {
//		s.putback(c);
//		s >> v_tmp[vx] >> v_tmp[vy];
//	}
//	if (s)
//		v = v_tmp;
//	return s;
//}
#endif // ALGEBRAIOSTREAMS

void swap(vec2& a, vec2& b)
{ 
	vec2 tmp(a); a = b; b = tmp;
}

vec2 min(const vec2& a, const vec2& b)
{ 
	return vec2(MIN(a.n[vx], b.n[vx]), MIN(a.n[vy], b.n[vy]));
}

vec2 max(const vec2& a, const vec2& b)
{ 
	return vec2(MAX(a.n[vx], b.n[vx]), MAX(a.n[vy], b.n[vy]));
}

vec2 prod(const vec2& a, const vec2& b)
{ 
	return vec2(a.n[vx] * b.n[vx], a.n[vy] * b.n[vy]);
}




/****************************************************************
*																*
*		    vec3 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec3::vec3()
{
}

vec3::vec3(const Real x, const Real y, const Real z)
{ 
	n[vx] = x; n[vy] = y; n[vz] = z; 
}

vec3::vec3(const Real d)
{ 
	n[vx] = n[vy] = n[vz] = d; 
}

vec3::vec3(const vec3& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; 
}

vec3::vec3(const vec2& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = 1.0; 
}

vec3::vec3(const vec2& v, Real d)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = d; 
}

vec3::vec3(const vec4& v) // it is up to caller to avoid divide-by-zero
{ 
	n[vx] = v.n[vx] / v.n[vw]; 
	n[vy] = v.n[vy] / v.n[vw];
	n[vz] = v.n[vz] / v.n[vw]; 
}

vec3::vec3(const quat& q)	// cast quat to  v3 //*** SHL added 11/25/10
{
}

vec3::vec3(const vec4& v, int dropAxis)
{
	switch (dropAxis) {
	case vx: n[vx] = v.n[vy]; n[vy] = v.n[vz]; n[vz] = v.n[vw]; break;
	case vy: n[vx] = v.n[vx]; n[vy] = v.n[vz]; n[vz] = v.n[vw]; break;
	case vz: n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vw]; break;
	default: n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; break;
	}
}


// ASSIGNMENT OPERATORS

vec3& vec3::operator = (const vec3& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; return *this; 
}

vec3& vec3::operator += ( const vec3& v )
{ 
	n[vx] += v.n[vx]; n[vy] += v.n[vy]; n[vz] += v.n[vz]; return *this; 
}

vec3& vec3::operator -= ( const vec3& v )
{ 
	n[vx] -= v.n[vx]; n[vy] -= v.n[vy]; n[vz] -= v.n[vz]; return *this; 
}

vec3& vec3::operator *= ( const Real d )
{ 
	n[vx] *= d; n[vy] *= d; n[vz] *= d; return *this; 
}

vec3& vec3::operator /= ( const Real d )
{ 
	Real d_inv = 1./d; n[vx] *= d_inv; n[vy] *= d_inv; n[vz] *= d_inv;
	return *this; 
}

Real& vec3::operator [] ( int i) {
	assert(! (i < vx || i > vz));
	return n[i];
}

Real vec3::operator [] ( int i) const {
	assert(! (i < vx || i > vz));
	return n[i];
}


// SPECIAL FUNCTIONS

Real vec3::Length() const
{  
	return sqrt(Length2()); 
}

Real vec3::Length2() const
{  
	return n[vx]*n[vx] + n[vy]*n[vy] + n[vz]*n[vz]; 
}

vec3& vec3::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	Real norm = Length();
	if (norm == 0.0)
		return *this = vec3(1.0f, 0.0f, 0.0f);
	else return *this /= (double) norm;
}

vec3& vec3::apply(Svz_V_FCT_PTR fct)
{ 
	n[vx] = (*fct)(n[vx]); n[vy] = (*fct)(n[vy]); n[vz] = (*fct)(n[vz]);
	return *this; 
}

vec3 vec3::Cross(vec3 &v) const
{
	vec3 tmp;
	tmp[0] = n[1] * v.n[2] - n[2] * v.n[1];
	tmp[1] = n[2] * v.n[0] - n[0] * v.n[2];
	tmp[2] = n[0] * v.n[1] - n[1] * v.n[0];
	return tmp;
}

int vec3::dim() const								// SHL added - returns dimension of vector
{
	return (sizeof(n)/sizeof(Real));
}

Real* vec3::getn()
{
	return n;
}

// FRIENDS

vec3 operator - (const vec3& a)
{  
	return vec3(-a.n[vx],-a.n[vy],-a.n[vz]);
}

vec3 operator + (const vec3& a, const vec3& b)
{ 
	return vec3(a.n[vx]+ b.n[vx], a.n[vy] + b.n[vy], a.n[vz] + b.n[vz]);
}

vec3 operator - (const vec3& a, const vec3& b)
{ 
	return vec3(a.n[vx]-b.n[vx], a.n[vy]-b.n[vy], a.n[vz]-b.n[vz]);
}

vec3 operator * (const vec3& a, const Real d)
{ 
	return vec3(d*a.n[vx], d*a.n[vy], d*a.n[vz]);
}

vec3 operator * (const Real d, const vec3& a)
{ 
	return a*d; 
}

vec3 operator * (const mat3& a, const vec3& v)
{
#define ROWCOL(i) a.v[i].n[0]*v.n[vx] + a.v[i].n[1]*v.n[vy] \
	+ a.v[i].n[2]*v.n[vz]
	return vec3(ROWCOL(0), ROWCOL(1), ROWCOL(2));
#undef ROWCOL // (i)
}

vec3 operator * (const mat4& a, const vec3& v)
{ 
	return a * vec4(v);
}

vec3 operator * (const vec3& v, const mat4& a)
{ 
	return a.transpose() * v; 
}

Real operator * (const vec3& a, const vec3& b)
{ 
	return (a.n[vx]*b.n[vx] + a.n[vy]*b.n[vy] + a.n[vz]*b.n[vz]); 
}

vec3 operator / (const vec3& a, const Real d)
{ 
	Real d_inv = 1./d; 
	return vec3(a.n[vx]*d_inv, a.n[vy]*d_inv, a.n[vz]*d_inv);
}

vec3 operator ^ (const vec3& a, const vec3& b)
{
	return vec3(a.n[vy]*b.n[vz] - a.n[vz]*b.n[vy],
		a.n[vz]*b.n[vx] - a.n[vx]*b.n[vz],
		a.n[vx]*b.n[vy] - a.n[vy]*b.n[vx]);
}

int operator == (const vec3& a, const vec3& b)
{ 
	return (a.n[vx] == b.n[vx]) && (a.n[vy] == b.n[vy]) && (a.n[vz] == b.n[vz]);
}

int operator != (const vec3& a, const vec3& b)
{ 
	return !(a == b); }

#ifdef ALGEBRAIOSTREAMS
//ostream& operator << (ostream& s, const vec3& v)
//{ 
//	return s << "[ " << v.n[vx] << ' ' << v.n[vy] << ' ' << v.n[vz] << " ]"; 
//}
ostream& operator << (ostream& s, const vec3& v)
{ 
	return s << v[0] << "\t" << v[1] << "\t" << v[2]; 
}
istream& operator >> (istream& s, vec3& v)
{
	vec3	v_tmp;

	s >> v_tmp[0] >> v_tmp[1] >> v_tmp[2]; 

	if (s)
		v = v_tmp;

	return s;
}


// istream& operator >> (istream& s, vec3& v)
//{
//    vec3	v_tmp;
//    char	c = ' ';
//
//    while (isspace(c))
//	s >> c;
//    // The vectors can be formatted either as x y z or | x y z |
//    if (c == '[') {
//	s >> v_tmp[vx] >> v_tmp[vy] >> v_tmp[vz];
//	while (s >> c && isspace(c)) ;
//	if (c != ']')
//	    s.set(_bad);
//	}
//    else {
//	s.putback(c);
//	s >> v_tmp[vx] >> v_tmp[vy] >> v_tmp[vz];
//	}
//    if (s)
//	v = v_tmp;
//    return s;
//}
#endif // ALGEBRAIOSTREAMS

void swap(vec3& a, vec3& b)
{ 
	vec3 tmp(a); a = b; b = tmp;
}

vec3 min(const vec3& a, const vec3& b)
{ 
	return vec3(MIN(a.n[vx], b.n[vx]), MIN(a.n[vy], b.n[vy]), MIN(a.n[vz], b.n[vz]));
}

vec3 max(const vec3& a, const vec3& b)
{ 
	return vec3(MAX(a.n[vx], b.n[vx]), MAX(a.n[vy], b.n[vy]), MAX(a.n[vz], b.n[vz]));
}

vec3 prod(const vec3& a, const vec3& b)
{ 
	return vec3(a.n[vx] * b.n[vx], a.n[vy] * b.n[vy], a.n[vz] * b.n[vz]);
}


/****************************************************************
*																*
*		    vec4 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec4::vec4()
{
}

vec4::vec4(const Real x, const Real y, const Real z, const Real w)
{ 
	n[vx] = x; n[vy] = y; n[vz] = z; n[vw] = w; 
}

vec4::vec4(const Real d)
{  
	n[vx] = n[vy] = n[vz] = n[vw] = d; 
}

vec4::vec4(const vec4& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; n[vw] = v.n[vw]; 
}

vec4::vec4(const vec3& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; n[vw] = 1.0; 
}

vec4::vec4(const vec3& v, const Real d)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz];  n[vw] = d; 
}


// ASSIGNMENT OPERATORS

vec4& vec4::operator = (const vec4& v)
{ 
	n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz]; n[vw] = v.n[vw];
	return *this; 
}

vec4& vec4::operator += ( const vec4& v )
{ 
	n[vx] += v.n[vx]; n[vy] += v.n[vy]; n[vz] += v.n[vz]; n[vw] += v.n[vw];
	return *this; 
}

vec4& vec4::operator -= ( const vec4& v )
{ 
	n[vx] -= v.n[vx]; n[vy] -= v.n[vy]; n[vz] -= v.n[vz]; n[vw] -= v.n[vw];
	return *this; 
}

vec4& vec4::operator *= ( const Real d )
{ 
	n[vx] *= d; n[vy] *= d; n[vz] *= d; n[vw] *= d; 
	return *this; 
}

vec4& vec4::operator /= ( const Real d )
{ 
	Real d_inv = 1./d; 
	n[vx] *= d_inv; n[vy] *= d_inv; n[vz] *= d_inv; n[vw] *= d_inv; 
	return *this; 
}

Real& vec4::operator [] ( int i)
{
	assert(! (i < vx || i > vw));
	return n[i];
}

Real vec4::operator [] ( int i) const
{
	assert(! (i < vx || i > vw));
	return n[i];
}

// SPECIAL FUNCTIONS

Real vec4::Length() const
{ 
	return sqrt(Length2()); 
}

Real vec4::Length2() const
{ 
	return n[vx]*n[vx] + n[vy]*n[vy] + n[vz]*n[vz] + n[vw]*n[vw]; 
}

vec4& vec4::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	*this /= Length(); return *this; 
}

vec4& vec4::apply(Svz_V_FCT_PTR fct)
{ 
	n[vx] = (*fct)(n[vx]); 
	n[vy] = (*fct)(n[vy]); 
	n[vz] = (*fct)(n[vz]);
	n[vw] = (*fct)(n[vw]); 
	return *this; 
}

int vec4::dim() const								// SHL added - returns dimension of vector
{
	return (sizeof(n)/sizeof(Real));
}

// FRIENDS

vec4 operator - (const vec4& a)
{ 
	return vec4(-a.n[vx],-a.n[vy],-a.n[vz],-a.n[vw]);
}

vec4 operator + (const vec4& a, const vec4& b)
{ 
	return vec4(a.n[vx] + b.n[vx], a.n[vy] + b.n[vy], a.n[vz] + b.n[vz], a.n[vw] + b.n[vw]);
}

vec4 operator - (const vec4& a, const vec4& b)
{  
	return vec4(a.n[vx] - b.n[vx], a.n[vy] - b.n[vy], a.n[vz] - b.n[vz], a.n[vw] - b.n[vw]);
}

vec4 operator * (const vec4& a, const Real d)
{ 
	return vec4(d*a.n[vx], d*a.n[vy], d*a.n[vz], d*a.n[vw] );
}

vec4 operator * (const Real d, const vec4& a)
{ 
	return a*d; 
}

vec4 operator * (const mat4& a, const vec4& v)
{
#define ROWCOL(i) a.v[i].n[0]*v.n[vx] + a.v[i].n[1]*v.n[vy] \
	+ a.v[i].n[2]*v.n[vz] + a.v[i].n[3]*v.n[vw]
	return vec4(ROWCOL(0), ROWCOL(1), ROWCOL(2), ROWCOL(3));
#undef ROWCOL // (i)
}

vec4 operator * (const vec4& v, const mat4& a)
{ 
	return a.transpose() * v; 
}

Real operator * (const vec4& a, const vec4& b)
{ 
	return (a.n[vx]*b.n[vx] + a.n[vy]*b.n[vy] + a.n[vz]*b.n[vz] + a.n[vw]*b.n[vw]); 
}

vec4 operator / (const vec4& a, const Real d)
{ 
	Real d_inv = 1./d; 
	return vec4(a.n[vx]*d_inv, a.n[vy]*d_inv, a.n[vz]*d_inv, a.n[vw]*d_inv);
}

int operator == (const vec4& a, const vec4& b)
{ 
	return (a.n[vx] == b.n[vx]) && (a.n[vy] == b.n[vy]) && (a.n[vz] == b.n[vz]) && (a.n[vw] == b.n[vw]); 
}

int operator != (const vec4& a, const vec4& b)
{ 
	return !(a == b); 
}

#ifdef ALGEBRAIOSTREAMS
ostream& operator << (ostream& s, const vec4& v)
{ 
	return s << "[ " << v.n[vx] << ' ' << v.n[vy] << ' ' << v.n[vz] << ' ' << v.n[vw] << " ]"; 
}

// stream& operator >> (istream& s, vec4& v)
//{
//    vec4	v_tmp;
//    char	c = ' ';
//
//    while (isspace(c))
//	s >> c;
//    // The vectors can be formatted either as x y z w or | x y z w |
//    if (c == '[') {
//	s >> v_tmp[vx] >> v_tmp[vy] >> v_tmp[vz] >> v_tmp[vw];
//	while (s >> c && isspace(c)) ;
//	if (c != ']')
//	    s.set(_bad);
//	}
//    else {
//	s.putback(c);
//	s >> v_tmp[vx] >> v_tmp[vy] >> v_tmp[vz] >> v_tmp[vw];
//	}
//    if (s)
//	v = v_tmp;
//    return s;
//}
#endif // ALGEBRAIOSTREAMS

void swap(vec4& a, vec4& b)
{ 
	vec4 tmp(a); a = b; b = tmp;
}

vec4 min(const vec4& a, const vec4& b)
{ 
	return vec4(MIN(a.n[vx], b.n[vx]), MIN(a.n[vy], b.n[vy]), MIN(a.n[vz], b.n[vz]), MIN(a.n[vw], b.n[vw]));
}

vec4 max(const vec4& a, const vec4& b)
{ 
	return vec4(MAX(a.n[vx], b.n[vx]), MAX(a.n[vy], b.n[vy]), MAX(a.n[vz], b.n[vz]), MAX(a.n[vw], b.n[vw]));
}

vec4 prod(const vec4& a, const vec4& b)
{ 
	return vec4(a.n[vx] * b.n[vx], a.n[vy] * b.n[vy], a.n[vz] * b.n[vz], a.n[vw] * b.n[vw]);
}


/****************************************************************
*																*
*		    mat3 member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

mat3::mat3()
{
	v[0] = vec3(0.0,0.0,0.0);
	v[1] = v[2] = v[0];
}

mat3::mat3(const vec3& v0, const vec3& v1, const vec3& v2)
{ 
	v[0] = v0; v[1] = v1; v[2] = v2; 
}

mat3::mat3(const Real d)
{ 
	v[0] = v[1] = v[2] = vec3(d);
}

mat3::mat3(const mat3& m)
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
}

mat3::mat3(const mat4& m){
	v[0] = vec3(m[0][0], m[0][1], m[0][2]);
	v[1] = vec3(m[1][0], m[1][1], m[1][2]);
	v[2] = vec3(m[2][0], m[2][1], m[2][2]);
}

// Static functions

mat3 mat3::Identity()
{
	return mat3(vec3(1.0, 0.0, 0.0),
		vec3(0.0, 1.0, 0.0),
		vec3(0.0, 0.0, 1.0));
}

mat3 mat3::Translation2D(const vec2& v)
{
	return mat3(vec3(1.0, 0.0, v[vx]),
		vec3(0.0, 1.0, v[vy]),
		vec3(0.0, 0.0, 1.0));
}

mat3 mat3::Rotation2DDeg(const vec2& center, const Real angleDeg)
{
	Real  angleRad = angleDeg * Deg2Rad;
	return Rotation2DRad(center, angleRad);
}

mat3 mat3::Rotation2DRad(const vec2& center, const Real angleRad)
{
	Real  c = cos(angleRad), s = sin(angleRad);

	return mat3(vec3(c, -s, center[vx] * (1.0-c) + center[vy] * s),
		vec3(s, c, center[vy] * (1.0-c) - center[vx] * s),
		vec3(0.0, 0.0, 1.0));
}

mat3 mat3::Scaling2D(const vec2& scaleVector)
{
	return mat3(vec3(scaleVector[vx], 0.0, 0.0),
		vec3(0.0, scaleVector[vy], 0.0),
		vec3(0.0, 0.0, 1.0));
}

mat3 mat3::Rotation3DDeg(const vec3& axis, const Real angleDeg)
{
	Real  angleRad = angleDeg * Deg2Rad;
	return Rotation3DRad(axis, angleRad);
}

mat3 mat3::Rotation3DRad(const vec3& axis, const Real angleRad)
{
	Real c = cos(angleRad), s = sin(angleRad), t = 1.0 - c;
	vec3 Axis = axis;
	Axis.Normalize();
	return mat3(vec3(t * Axis[vx] * Axis[vx] + c,
		t * Axis[vx] * Axis[vy] - s * Axis[vz],
		t * Axis[vx] * Axis[vz] + s * Axis[vy]),
		vec3(t * Axis[vx] * Axis[vy] + s * Axis[vz],
		t * Axis[vy] * Axis[vy] + c,
		t * Axis[vy] * Axis[vz] - s * Axis[vx]),
		vec3(t * Axis[vx] * Axis[vz] - s * Axis[vy],
		t * Axis[vy] * Axis[vz] + s * Axis[vx],
		t * Axis[vz] * Axis[vz] + c)
		);
}

mat3 mat3::Rotation3DDeg(const int Axis, const Real angleDeg)
{
	Real  angleRad = angleDeg * Deg2Rad;
	return Rotation3DRad(Axis, angleRad);
}

mat3 mat3::Rotation3DRad(const int Axis, const Real angleRad)
{
	vec3 axis;
	switch(Axis)
	{
	case vx: axis = vec3(1.0, 0.0, 0.0);
		break;
	case vy: axis = vec3(0.0, 1.0, 0.0);
		break;
	case vz: axis = vec3(0.0, 0.0, 1.0);
		break;
	}
	return Rotation3DRad(axis, angleRad);
}

mat3 mat3::Slerp(const mat3& rot0, const mat3& rot1, const Real& fPerc)
{
	return mat3(0.0f);
}

mat3 mat3::Lerp(const mat3& rot0, const mat3& rot1, const Real& fPerc)
{
	return mat3(0.0f);
}

// Rotation operations, matrix must be orthonomal
bool mat3::ToEulerAnglesXYZ(vec3& angleRad) const
{
	return true;
}

bool mat3::ToEulerAnglesXZY(vec3& angleRad) const
{
	return true;
}

bool mat3::ToEulerAnglesYXZ(vec3& angleRad) const
{
	return true;
}

bool mat3::ToEulerAnglesYZX(vec3& angleRad) const
{
	return true;
}

bool mat3::ToEulerAnglesZXY(vec3& angleRad) const
{
	return true;
}

bool mat3::ToEulerAnglesZYX(vec3& angleRad) const
{
	return true;
}

mat3 mat3::FromEulerAnglesXYZ(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

mat3 mat3::FromEulerAnglesXZY(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

mat3 mat3::FromEulerAnglesYXZ(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

mat3 mat3::FromEulerAnglesYZX(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

mat3 mat3::FromEulerAnglesZXY(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

mat3 mat3::FromEulerAnglesZYX(const vec3& angleRad)
{
	*this = mat3(0.0f);
	return *this;
}

bool mat3::reorthogonalize()
{
	// Factor M = QR where Q is orthogonal and R is upper triangular.
	// Algorithm uses Gram-Schmidt orthogonalization (the QR algorithm).
	//
	// If M = [ m0 | m1 | m2 ] and Q = [ q0 | q1 | q2 ], then
	//
	//   q0 = m0/|m0|
	//   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
	//   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
	//
	// where |V| indicates length of vector V and A*B indicates dot
	// product of vectors A and B.  The matrix R has entries
	//
	//   r00 = q0*m0  r01 = q0*m1  r02 = q0*m2
	//   r10 = 0      r11 = q1*m1  r12 = q1*m2
	//   r20 = 0      r21 = 0      r22 = q2*m2
	//
	// The reorthogonalization replaces current matrix by computed Q.

	const Real fEpsilon = 1e-05f;

	// unitize column 0
	Real fLength = sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
	if ( fLength < fEpsilon )
		return false;
	Real fInvLength = 1.0f / fLength;
	v[0][0] *= fInvLength;
	v[1][0] *= fInvLength;
	v[2][0] *= fInvLength;

	// project out column 0 from column 1
	Real fDot = v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1];
	v[0][1] -= fDot * v[0][0];
	v[1][1] -= fDot * v[1][0];
	v[2][1] -= fDot * v[2][0];

	// unitize column 1
	fLength = sqrt(v[0][1] * v[0][1] + v[1][1] * v[1][1] + v[2][1] * v[2][1]);
	if ( fLength < fEpsilon )
		return false;
	fInvLength = 1.0f/fLength;
	v[0][1] *= fInvLength;
	v[1][1] *= fInvLength;
	v[2][1] *= fInvLength;

	// project out column 0 from column 2
	fDot = v[0][0] * v[0][2] + v[1][0] * v[1][2] + v[2][0] * v[2][2];
	v[0][2] -= fDot * v[0][0];
	v[1][2] -= fDot * v[1][0];
	v[2][2] -= fDot * v[2][0];

	// project out column 1 from column 2
	fDot = v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2];
	v[0][2] -= fDot * v[0][1];
	v[1][2] -= fDot * v[1][1];
	v[2][2] -= fDot * v[2][1];

	// unitize column 2
	fLength = sqrt(v[0][2] * v[0][2] + v[1][2] * v[1][2] + v[2][2] * v[2][2]);
	if ( fLength < fEpsilon )
		return false;
	fInvLength = 1.0f / fLength;
	v[0][2] *= fInvLength;
	v[1][2] *= fInvLength;
	v[2][2] *= fInvLength;

	return true;
}

// Conversion with quat
quat mat3::ToQuaternion() const
{
	return quat(0.0f);
}


void mat3::FromQuaternion(const quat& q)
{
}

void mat3::ToAxisAngle(vec3& axis, Real& angleRad) const
{
}

void mat3::ToAxisAngle2(vec3& axis, Real& angleRad) const
{
	quat q = this->ToQuaternion();
	//q.Normalize();
	q.ToAxisAngle(axis, angleRad);
}

mat3 mat3::inverse() const    // Gauss-Jordan elimination with partial pivoting
{
	mat3 a(*this),	    // As a evolves from original mat into identity
		b(mat3::Identity());   // b evolves from identity into inverse(a)
	int	 i, j, i1;

	// Loop over cols of a from left to right, eliminating above and below diag
	for (j=0; j<3; j++) {   // Find largest pivot in column j among rows j..2
		i1 = j;		    // Row with largest pivot candidate
		for (i=j+1; i<3; i++)
			if (fabs(a.v[i].n[j]) > fabs(a.v[i1].n[j]))
				i1 = i;

		// Swap rows i1 and j in a and b to put pivot on diagonal
		swap(a.v[i1], a.v[j]);
		swap(b.v[i1], b.v[j]);

		// Scale row j to have a unit diagonal
		if (a.v[j].n[j]==0.)
			ALGEBRA_ERROR("mat3::inverse: singular matrix; can't invert\n");
		b.v[j] /= a.v[j].n[j];
		a.v[j] /= a.v[j].n[j];

		// Eliminate off-diagonal elems in col j of a, doing identical ops to b
		for (i=0; i<3; i++)
			if (i!=j) 
			{
				b.v[i] -= a.v[i].n[j]*b.v[j];
				a.v[i] -= a.v[i].n[j]*a.v[j];
			}
	}
	return b;
}

void mat3::FromAxisAngle(const vec3& axis, const Real& angleRad)
{
	*this = Rotation3DRad(axis, angleRad);
}


// ASSIGNMENT OPERATORS

mat3& mat3::operator = ( const mat3& m )
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
	return *this; 
}

mat3& mat3::operator += ( const mat3& m )
{ 
	v[0] += m.v[0]; v[1] += m.v[1]; v[2] += m.v[2]; 
	return *this; 
}

mat3& mat3::operator -= ( const mat3& m )
{ 
	v[0] -= m.v[0]; v[1] -= m.v[1]; v[2] -= m.v[2]; 
	return *this; 
}

mat3& mat3::operator *= ( const Real d )
{ 
	v[0] *= d; v[1] *= d; v[2] *= d; 
	return *this; 
}

mat3& mat3::operator /= ( const Real d )
{ 
	v[0] /= d; v[1] /= d; v[2] /= d; 
	return *this; 
}

vec3& mat3::operator [] ( int i)
{
	assert(! (i < vx || i > vz));
	return v[i];
}

const vec3& mat3::operator [] ( int i) const
{
	assert(!(i < vx || i > vz));
	return v[i];
}

// SPECIAL FUNCTIONS

mat3 mat3::transpose() const
{
	return mat3(vec3(v[0][0], v[1][0], v[2][0]),
		vec3(v[0][1], v[1][1], v[2][1]),
		vec3(v[0][2], v[1][2], v[2][2]));
}

mat3& mat3::apply(Svz_V_FCT_PTR fct)
{
	v[vx].apply(fct);
	v[vy].apply(fct);
	v[vz].apply(fct);
	return *this;
}

void mat3::getData(Real* d)
{
	d[0] = v[0][0]; d[4] = v[0][1]; d[8] = v[0][2]; d[12] = 0;
	d[1] = v[1][0]; d[5] = v[1][1]; d[9] = v[1][2]; d[13] = 0;
	d[2] = v[2][0]; d[6] = v[2][1]; d[10] = v[2][2]; d[14] = 0;
	d[3] = 0; d[7] = 0; d[11] = 0; d[15] = 1;
}

vec3 mat3::getRow(unsigned int axis) const
{
	vec3 rowVec = v[axis];
	return rowVec;
}

vec3 mat3::getCol(unsigned int axis) const
{
	vec3 colVec;
	colVec[0] = v[0][axis]; colVec[1] = v[1][axis]; colVec[2] = v[2][axis];
	return colVec;
}

void mat3::setRow(unsigned int axis, const vec3& rowVec)
{
	v[axis] = rowVec;
}

void mat3::setCol(unsigned int axis, const vec3& colVec)
{
	v[0][axis] = colVec[0]; v[1][axis] = colVec[1]; v[2][axis] = colVec[2];
}

mat3 mat3::Normalize(int axis)
{
	vec3 v0 = vec3(v[0][0],v[1][0],v[2][0]).Normalize();
	vec3 v1 = vec3(v[0][1],v[1][1],v[2][1]).Normalize();
	vec3 v2 = vec3(v[0][2],v[1][2],v[2][2]).Normalize();

	if (axis == 0)
		v0 = v1.Cross(v2);
	else if (axis == 1)
		v1 = v2.Cross(v0);
	else if (axis == 2)
		v2 = v0.Cross(v1);
	else
		assert(3==4);

	return mat3(v0,v1,v2).transpose();
}

vec3 mat3::GetYawPitchRoll(unsigned int leftAxis, unsigned int upAxis, unsigned int frontAxis) const
{
	// Assume world coordinates: Y up, X left, Z front.

	vec3 leftVect, upVect, frontVect, dVect, angles, frontVect2, leftVect2;
	Real t, value, x, y;
	leftVect = getCol(leftAxis);
	upVect = getCol(upAxis);
	frontVect = getCol(frontAxis);

	// Compute yaw angle
	if (frontVect[vy] >= 0.0f && upVect[vy] >= 0.0f)
	{
		frontVect2 = frontVect;
		dVect = -upVect - frontVect2;
	}else if (frontVect[vy] < 0.0f && upVect[vy] < 0.0f)
	{
		frontVect2 = -frontVect;
		dVect = upVect - frontVect2;
	}else if (frontVect[vy] >= 0.0f && upVect[vy] < 0.0f)
	{
		frontVect2 = -frontVect;
		dVect = -upVect - frontVect2;

	}else if (frontVect[vy] < 0.0f && upVect[vy] >= 0.0f)
	{
		frontVect2 = frontVect;
		dVect = upVect - frontVect2;
	}
	t = -frontVect2[vy] / dVect[vy];
	x = frontVect2[vz] + t * dVect[vz];
	y = frontVect2[vx] + t * dVect[vx];
	angles[0] = atan2(y, x);
	frontVect2 = vec3(y, 0.0f, x);
	frontVect2.Normalize();
	leftVect2 = vec3(0.0f, 1.0f, 0.0f);
	leftVect2 = leftVect2.Cross(frontVect2);

	// Compute pitch angle
	Real dotProd = frontVect * frontVect2;
	if(fabs(dotProd) > 1.0)
		dotProd = (dotProd>0)?1.0:-1.0;
	Real v = acos(dotProd);
	if (frontVect[vy] >= 0.0f)
	{
		value = -v;
	}else
	{
		value = v;
	}
	angles[1] = value;

	// Compute roll angle
	dotProd = leftVect * leftVect2;
	if(fabs(dotProd) > 1.0)
		dotProd = (dotProd>0)?1.0:-1.0;
	v = acos(dotProd);
	if (leftVect[vy] >= 0.0f)
	{
		value = -v;
	}else
	{
		value = v;
	}
	angles[2] = value;

	return angles;
}

//OpenGL Functions

void mat3::ToGLMatrix( Real* pData )
{
	pData[0] = v[0][0]; pData[4] = v[0][1]; pData[8]  = v[0][2]; pData[12] = 0.0f;
	pData[1] = v[1][0]; pData[5] = v[1][1]; pData[9]  = v[1][2]; pData[13] = 0.0f;
	pData[2] = v[2][0]; pData[6] = v[2][1]; pData[10] = v[2][2]; pData[14] = 0.0f;
	pData[3] = 0.0f;    pData[7] = 0.0f;    pData[11] = 0.0f;    pData[15] = 1.0f;
}

void mat3::WriteToGLMatrix(Real* m)
{
	m[0] = v[0][0]; m[4] = v[0][1]; m[8] = v[0][2];  m[12] = 0.0f;
	m[1] = v[1][0]; m[5] = v[1][1]; m[9] = v[1][2];  m[13] = 0.0f;
	m[2] = v[2][0]; m[6] = v[2][1]; m[10] = v[2][2]; m[14] = 0.0f;
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void mat3::ReadFromGLMatrix(Real* m)
{
	v[0][0] = m[0]; v[0][1] = m[4]; v[0][2] = m[8];
	v[1][0] = m[1]; v[1][1] = m[5]; v[1][2] = m[9];
	v[2][0] = m[2]; v[2][1] = m[6]; v[2][2] = m[10];
}



// FRIENDS

mat3 operator - (const mat3& a)
{ 
	return mat3(-a.v[0], -a.v[1], -a.v[2]);
}

mat3 operator + (const mat3& a, const mat3& b)
{ 
	return mat3(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]);
}

mat3 operator - (const mat3& a, const mat3& b)
{ 
	return mat3(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]);
}

mat3 operator * (const mat3& a, const mat3& b)
{
#define ROWCOL(i, j) \
	a.v[i].n[0]*b.v[0][j] + a.v[i].n[1]*b.v[1][j] + a.v[i].n[2]*b.v[2][j]
	return mat3(vec3(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2)),
		vec3(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2)),
		vec3(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2)));
#undef ROWCOL // (i, j)
}

mat3 operator * (const mat3& a, const Real d)
{ 
	return mat3(a.v[0] * d, a.v[1] * d, a.v[2] * d);
}

mat3 operator * (const Real d, const mat3& a)
{ 
	return a*d; 
}

mat3 operator / (const mat3& a, const Real d)
{ 
	return mat3(a.v[0] / d, a.v[1] / d, a.v[2] / d);
}

int operator == (const mat3& a, const mat3& b)
{ 
	return (a.v[0] == b.v[0]) && (a.v[1] == b.v[1]) && (a.v[2] == b.v[2]); 
}

int operator != (const mat3& a, const mat3& b)
{ 
	return !(a == b); 
}

#ifdef ALGEBRAIOSTREAMS
ostream& operator << (ostream& s, const mat3& m)
{ 
	//return s << m.v[vx] << '\t' << m.v[vy] << '\t' << m.v[vz]; 
	return s	<< m[0][0] << "\t" << m[0][1] << "\t" << m[0][2] << "\t" 
		<< m[1][0] << "\t" << m[1][1] << "\t" << m[1][2] << "\t" 
		<< m[2][0] << "\t" << m[2][1] << "\t" << m[2][2];
}

// stream& operator >> (istream& s, mat3& m)
//{
//    mat3 m_tmp;
//    s >> m_tmp[vx] >> m_tmp[vy] >> m_tmp[vz];
//    if (s)
//	m = m_tmp;
//    return s;
//}

istream& operator >> (istream& s, mat3& m)
{
	mat3 m_tmp;
	s   >> m_tmp[0][0] >> m_tmp[0][1] >> m_tmp[0][2]
	>> m_tmp[1][0] >> m_tmp[1][1] >> m_tmp[1][2]
	>> m_tmp[2][0] >> m_tmp[2][1] >> m_tmp[2][2];

	if (s)
		m = m_tmp;
	return s;
}
#endif // ALGEBRAIOSTREAMS

void swap(mat3& a, mat3& b)
{ 
	mat3 tmp(a); a = b; b = tmp;
}


/****************************************************************
*																*
*		    mat4 member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

mat4::mat4()
{
}

mat4::mat4(const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3)
{ 
	v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3; 
}

mat4::mat4(const Real d)
{ 
	v[0] = v[1] = v[2] = v[3] = vec4(d);
}

mat4::mat4(const mat4& m)
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; v[3] = m.v[3]; 
}

mat4::mat4(const Real* d)
{
	v[0] = vec4(d[0], d[4], d[8], d[12]);
	v[1] = vec4(d[1], d[5], d[9], d[13]);
	v[2] = vec4(d[2], d[6], d[10], d[14]);
	v[3] = vec4(d[3], d[7], d[11], d[15]);
}

mat4::mat4(const mat3& m)
{
	v[0] = vec4(m[0], 0);
	v[1] = vec4(m[1], 0);
	v[2] = vec4(m[2], 0);
	v[3] = vec4(0, 0, 0, 1);
}

mat4::mat4(const mat3& m, const vec3& t)
{
	v[0] = vec4(m[0], t[0]);
	v[1] = vec4(m[1], t[1]);
	v[2] = vec4(m[2], t[2]);
	v[3] = vec4(0, 0, 0, 1);
}

// Static functions

mat4 mat4::identity()
{
	return mat4(vec4(1.0, 0.0, 0.0, 0.0),
		vec4(0.0, 1.0, 0.0, 0.0),
		vec4(0.0, 0.0, 1.0, 0.0),
		vec4(0.0, 0.0, 0.0, 1.0));
}

mat4 mat4::translation3D(const vec3& v)
{
	return mat4(vec4(1.0, 0.0, 0.0, v[vx]),
		vec4(0.0, 1.0, 0.0, v[vy]),
		vec4(0.0, 0.0, 1.0, v[vz]),
		vec4(0.0, 0.0, 0.0, 1.0));
}

mat4 mat4::rotation3DDeg(const vec3& axis, const Real angleDeg)
{
	Real angleRad = angleDeg * Deg2Rad;
	return rotation3DRad(axis, angleRad);
}

mat4 mat4::rotation3DRad(const vec3& axis, const Real angleRad)
{
	Real  c = cos(angleRad),
		s = sin(angleRad),
		t = 1.0 - c;
	vec3 Axis = axis;
	Axis.Normalize();
	return mat4(vec4(t * Axis[vx] * Axis[vx] + c,
		t * Axis[vx] * Axis[vy] - s * Axis[vz],
		t * Axis[vx] * Axis[vz] + s * Axis[vy],
		0.0),
		vec4(t * Axis[vx] * Axis[vy] + s * Axis[vz],
		t * Axis[vy] * Axis[vy] + c,
		t * Axis[vy] * Axis[vz] - s * Axis[vx],
		0.0),
		vec4(t * Axis[vx] * Axis[vz] - s * Axis[vy],
		t * Axis[vy] * Axis[vz] + s * Axis[vx],
		t * Axis[vz] * Axis[vz] + c,
		0.0),
		vec4(0.0, 0.0, 0.0, 1.0));
}

mat4 mat4::scaling3D(const vec3& scaleVector)
{
	return mat4(vec4(scaleVector[vx], 0.0, 0.0, 0.0),
		vec4(0.0, scaleVector[vy], 0.0, 0.0),
		vec4(0.0, 0.0, scaleVector[vz], 0.0),
		vec4(0.0, 0.0, 0.0, 1.0));
}

mat4 mat4::perspective3D(const Real d)
{
	return mat4(vec4(1.0, 0.0, 0.0, 0.0),
		vec4(0.0, 1.0, 0.0, 0.0),
		vec4(0.0, 0.0, 1.0, 0.0),
		vec4(0.0, 0.0, 1.0/d, 0.0));
}

// ASSIGNMENT OPERATORS

mat4& mat4::operator = ( const mat4& m )
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; v[3] = m.v[3];
	return *this; 
}

mat4& mat4::operator += ( const mat4& m )
{ 
	v[0] += m.v[0]; v[1] += m.v[1]; v[2] += m.v[2]; v[3] += m.v[3];
	return *this; 
}

mat4& mat4::operator -= ( const mat4& m )
{ 
	v[0] -= m.v[0]; v[1] -= m.v[1]; v[2] -= m.v[2]; v[3] -= m.v[3];
	return *this; 
}

mat4& mat4::operator *= ( const Real d )
{ 
	v[0] *= d; v[1] *= d; v[2] *= d; v[3] *= d; 
	return *this; 
}

mat4& mat4::operator /= ( const Real d )
{ 
	v[0] /= d; v[1] /= d; v[2] /= d; v[3] /= d; 
	return *this; 
}

vec4& mat4::operator [] ( int i)
{
	assert(! (i < vx || i > vw));
	return v[i];
}

const vec4& mat4::operator [] ( int i) const
{
	assert(! (i < vx || i > vw));
	return v[i];
}

// SPECIAL FUNCTIONS;

mat4 mat4::transpose() const
{
	return mat4(vec4(v[0][0], v[1][0], v[2][0], v[3][0]),
		vec4(v[0][1], v[1][1], v[2][1], v[3][1]),
		vec4(v[0][2], v[1][2], v[2][2], v[3][2]),
		vec4(v[0][3], v[1][3], v[2][3], v[3][3]));
}

mat4& mat4::apply(Svz_V_FCT_PTR fct)
{ 
	v[vx].apply(fct); v[vy].apply(fct); v[vz].apply(fct); v[vw].apply(fct);
	return *this; 
}

void mat4::getData(Real* d)
{
	d[0] = v[0][0]; d[1] = v[1][0]; d[2] = v[2][0]; d[3] = v[3][0];
	d[4] = v[0][1]; d[5] = v[1][1]; d[6] = v[2][1]; d[7] = v[3][1];
	d[8] = v[0][2]; d[9] = v[1][2]; d[10] = v[2][2]; d[11] = v[3][2];
	d[12] = v[0][3]; d[13] = v[1][3]; d[14] = v[2][3]; d[15] = v[3][3];
}

// FRIENDS

mat4 operator - (const mat4& a)
{ 
	return mat4(-a.v[0], -a.v[1], -a.v[2], -a.v[3]);
}

mat4 operator + (const mat4& a, const mat4& b)
{ 
	return mat4(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]);
}

mat4 operator - (const mat4& a, const mat4& b)
{ 
	return mat4(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2], a.v[3] - b.v[3]);
}

mat4 operator * (const mat4& a, const mat4& b)
{
#define ROWCOL(i, j) a.v[i].n[0]*b.v[0][j] + a.v[i].n[1]*b.v[1][j] + \
	a.v[i].n[2]*b.v[2][j] + a.v[i].n[3]*b.v[3][j]
	return mat4(
		vec4(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2), ROWCOL(0,3)),
		vec4(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2), ROWCOL(1,3)),
		vec4(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2), ROWCOL(2,3)),
		vec4(ROWCOL(3,0), ROWCOL(3,1), ROWCOL(3,2), ROWCOL(3,3))
		);
#undef ROWCOL
}

mat4 operator * (const mat4& a, const Real d)
{ 
	return mat4(a.v[0] * d, a.v[1] * d, a.v[2] * d, a.v[3] * d);
}

mat4 operator * (const Real d, const mat4& a)
{ 
	return a*d; 
}

mat4 operator / (const mat4& a, const Real d)
{ 
	return mat4(a.v[0] / d, a.v[1] / d, a.v[2] / d, a.v[3] / d);
}

int operator == (const mat4& a, const mat4& b)
{ 
	return ((a.v[0] == b.v[0]) && (a.v[1] == b.v[1]) && (a.v[2] == b.v[2]) && (a.v[3] == b.v[3])); 
}

int operator != (const mat4& a, const mat4& b)
{ 
	return !(a == b); 
}

#ifdef ALGEBRAIOSTREAMS
ostream& operator << (ostream& s, const mat4& m)
{ 
	return s << m.v[vx] << '\n' << m.v[vy] << '\n' << m.v[vz] << '\n' << m.v[vw]; 
}

// istream& operator >> (istream& s, mat4& m)
//{
//	mat4 m_tmp;
//	s >> m_tmp[vx] >> m_tmp[vy] >> m_tmp[vz] >> m_tmp[vw];
//	if (s)
//		m = m_tmp;
//	return s;
//}
#endif // ALGEBRAIOSTREAMS

void swap(mat4& a, mat4& b)
{ 
	mat4 tmp(a); a = b; b = tmp;
}

mat4 mat4::inverse()	const    // Gauss-Jordan elimination with partial pivoting
{
	mat4 a(*this),	    // As a evolves from original mat into identity
		b(mat4::identity());   // b evolves from identity into inverse(a)
	int i, j, i1;

	// Loop over cols of a from left to right, eliminating above and below diag
	for (j=0; j<4; j++) {   // Find largest pivot in column j among rows j..3
		i1 = j;		    // Row with largest pivot candidate
		for (i=j+1; i<4; i++)
			if (fabs(a.v[i].n[j]) > fabs(a.v[i1].n[j]))
				i1 = i;

		// Swap rows i1 and j in a and b to put pivot on diagonal
		swap(a.v[i1], a.v[j]);
		swap(b.v[i1], b.v[j]);

		// Scale row j to have a unit diagonal
		if (a.v[j].n[j]==0.)
			ALGEBRA_ERROR("mat4::inverse: singular matrix; can't invert\n");
		b.v[j] /= a.v[j].n[j];
		a.v[j] /= a.v[j].n[j];

		// Eliminate off-diagonal elems in col j of a, doing identical ops to b
		for (i=0; i<4; i++)
			if (i!=j) {
				b.v[i] -= a.v[i].n[j]*b.v[j];
				a.v[i] -= a.v[i].n[j]*a.v[j];
			}
	}
	return b;
}



/****************************************************************
*																*
*		    quat member functions							*
*																*
****************************************************************/

// CONSTRUCTORS

quat::quat()
{
}

quat::quat(const Real d)  //*** SHL added 11/25/10
{ 
	if (d==0.0) {
		n[vw] = 1.0; n[vx] = n[vy] = n[vz] = 0.0; 
	}
	else n[vw] = d;

}

quat::quat(Real q[4])
{
	n[vw] = q[0]; n[vx] = q[1]; n[vy] = q[2]; n[vz] = q[3];
}

quat::quat(const Real w, const Real x, const Real y, const Real z)
{
	n[vw] = w; n[vx] = x; n[vy] = y; n[vz] = z;
}

quat::quat(const quat& q)
{
	n[vw] = q.n[vw]; n[vx] = q.n[vx]; n[vy] = q.n[vy]; n[vz] = q.n[vz];
}

quat::quat(const vec4& v)
{
	n[vw] = v.n[vw]; n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz];
}

quat::quat(const vec3& v)  //*** SHL added 11/25/10
{ 
	n[vw] = 0.0; n[vx] = v.n[vx]; n[vy] = v.n[vy]; n[vz] = v.n[vz];
}

// Static functions

Real quat::Dot(const quat& q0, const quat& q1)
{
	return q0.n[vw] * q1.n[vw] + q0.n[vx] * q1.n[vx] + q0.n[vy] * q1.n[vy] + q0.n[vz] * q1.n[vz];
}

quat quat::UnitInverse(const quat& q)
{
	return quat(q.n[vw], -q.n[vx], -q.n[vy], -q.n[vz]);
}

Real quat::CounterWarp(Real t, Real fCos)
{
	const Real ATTENUATION = 0.82279687f;
	const Real WORST_CASE_SLOPE = 0.58549219f;

	Real fFactor = 1.0 - ATTENUATION * fCos;
	fFactor *= fFactor;
	Real fK = WORST_CASE_SLOPE * fFactor;

	return t * (fK * t * (2.0 * t - 3.0) + 1.0 + fK);
}

static const Real ISQRT_NEIGHBORHOOD = 0.959066f;
static const Real ISQRT_SCALE = 1.000311f;
static const Real ISQRT_ADDITIVE_CONSTANT = ISQRT_SCALE / (Real)sqrt(ISQRT_NEIGHBORHOOD);
static const Real ISQRT_FACTOR = ISQRT_SCALE * (-0.5f / (ISQRT_NEIGHBORHOOD * (Real)sqrt(ISQRT_NEIGHBORHOOD)));
Real quat::ISqrt_approx_in_neighborhood(Real s)
{
	return ISQRT_ADDITIVE_CONSTANT + (s - ISQRT_NEIGHBORHOOD) * ISQRT_FACTOR;	
}

// Assignment operators

quat& quat::operator = (const quat& q)
{
	n[vw] = q.n[vw]; n[vx] = q.n[vx]; n[vy] = q.n[vy]; n[vz] = q.n[vz];
	return *this;
}

quat& quat::operator += (const quat& q)
{
	n[vw] += q.n[vw]; n[vx] += q.n[vx]; n[vy] += q.n[vy]; n[vz] += q.n[vz];
	return *this;
}

quat& quat::operator -= (const quat& q)
{
	n[vw] -= q.n[vw]; n[vx] -= q.n[vx]; n[vy] -= q.n[vy]; n[vz] -= q.n[vz];
	return *this;
}

quat& quat::operator *= (const quat& q)
{
	*this = quat(n[vw] * q.n[vw] - n[vx] * q.n[vx] - n[vy] * q.n[vy] - n[vz] * q.n[vz],
		n[vw] * q.n[vx] + n[vx] * q.n[vw] + n[vy] * q.n[vz] - n[vz] * q.n[vy],
		n[vw] * q.n[vy] + n[vy] * q.n[vw] + n[vz] * q.n[vx] - n[vx] * q.n[vz],
		n[vw] * q.n[vz] + n[vz] * q.n[vw] + n[vx] * q.n[vy] - n[vy] * q.n[vx]);
	return *this;
}

quat& quat::operator *= (const Real d)
{
	n[vw] *= d; n[vx] *= d;	n[vy] *= d; n[vz] *= d;
	return *this;
}

quat& quat::operator /= (const Real d)
{
	n[vw] /= d; n[vx] /= d;	n[vy] /= d; n[vz] /= d;
	return *this;
}

// Indexing
Real& quat::operator [](int i)
{
	return n[i];
}

Real quat::operator [](int i) const
{
	return n[i];
}

Real& quat::W()
{
	return n[vw];
}

Real quat::W() const
{
	return n[vw];
}

Real& quat::X()
{
	return n[vx];
}

Real quat::X() const
{
	return n[vx];
}

Real& quat::Y()
{
	return n[vy];
}

Real quat::Y() const
{
	return n[vy];
}

Real& quat::Z()
{
	return n[vz];
}

Real quat::Z() const
{
	return n[vz];
}

// Friends

quat operator - (const quat& q)
{
	return quat(-q.n[vw], -q.n[vx], -q.n[vy], -q.n[vz]);
}

quat operator + (const quat& q0, const quat& q1)
{
	return quat(q0.n[vw] + q1.n[vw], q0.n[vx] + q1.n[vx], q0.n[vy] + q1.n[vy], q0.n[vz] + q1.n[vz]);
}

quat operator - (const quat& q0, const quat& q1)
{
	return quat(q0.n[vw] - q1.n[vw], q0.n[vx] - q1.n[vx], q0.n[vy] - q1.n[vy], q0.n[vz] - q1.n[vz]);
}

quat operator * (const quat& q, const Real d)
{
	return quat(q.n[vw] * d, q.n[vx] * d, q.n[vy] * d, q.n[vz] * d);
}

quat operator * (const Real d, const quat& q)
{
	return quat(q.n[vw] * d, q.n[vx] * d, q.n[vy] * d, q.n[vz] * d);
}

quat operator * (const quat& q0, const quat& q1)
{
	// modified
	return quat(0.0f);
}

quat operator / (const quat& q, const Real d)
{
	return quat(q.n[vw] / d, q.n[vx] / d, q.n[vy] / d, q.n[vz] / d);
}

bool operator == (const quat& q0, const quat& q1)
{
	return (q0.n[vw] == q1.n[vw]) && (q0.n[vx] == q1.n[vx]) && (q0.n[vy] == q1.n[vy]) && (q0.n[vz] == q1.n[vz]);
}

bool operator != (const quat& q0, const quat& q1)
{
	return !(q0 == q1); 
}

// special functions

Real quat::Length2() const
{
	return n[vw] * n[vw] + n[vx] * n[vx] + n[vy] * n[vy] + n[vz] * n[vz];
}

Real quat::Length() const
{
	return sqrt(Length2());
}

quat& quat::Normalize()
{
	Real l = Length();
	if (l < EPSILON || abs(l) > 1e6)
	{
		FromAxisAngle(vec3(0.0f, 1.0f, 0.0f), 0.0f);
	}else
	{
		*this /= l;
	}

	return *this; 
}

quat& quat::fastNormalize()
{
	Real s = n[vw] * n[vw] + n[vx] * n[vx] + n[vy] * n[vy] + n[vz] * n[vz]; // length^2
	Real k = ISqrt_approx_in_neighborhood(s);

	if (s <= 0.91521198) {
		k *= ISqrt_approx_in_neighborhood(k * k * s);

		if (s <= 0.65211970) {
			k *= ISqrt_approx_in_neighborhood(k * k * s);
		}
	}

	n[vw] *= k;
	n[vx] *= k;
	n[vy] *= k;
	n[vz] *= k;

	return * this;
}

quat quat::inverse() const
{
	return quat(n[vw], -n[vx], -n[vy], -n[vz]);
}

quat quat::Exp(const quat& q)
{
	// q = A*(x*i+y*j+z*k) where (x,y,z) is unit length
	// exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k)
	Real angle = sqrt(q.n[vx] * q.n[vx] + q.n[vy] * q.n[vy] + q.n[vz] * q.n[vz]);
	Real sn, cs;
	sn = sin(angle);
	cs = cos(angle);

	// When A is near zero, sin(A)/A is approximately 1.  Use
	// exp(q) = cos(A)+A*(x*i+y*j+z*k)
	Real coeff = ( abs(sn) < EPSILON ? 1.0 : sn/angle );

	quat result(cs, coeff * q.n[vx], coeff * q.n[vy], coeff * q.n[vz]);

	return result;
}

quat quat::Log(const quat& q)
{
	// q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x,y,z) is unit length
	// log(q) = A*(x*i+y*j+z*k)

	Real angle = acos(q.n[vw]);
	Real sn = sin(angle);

	// When A is near zero, A/sin(A) is approximately 1.  Use
	// log(q) = sin(A)*(x*i+y*j+z*k)
	Real coeff = ( abs(sn) < EPSILON ? 1.0 : angle/sn );

	return quat(0.0f, coeff * q.n[vx], coeff * q.n[vy], coeff * q.n[vz]);
}

void quat::Zero()
{
	n[vw] = n[vx] = n[vy] = n[vz] = 0.0f;
}

int quat::dim() const								// SHL added - returns dimension of vector
{
	return (sizeof(n)/sizeof(Real));
}

quat quat::Slerp(Real t, const quat& q0, const quat& q1)
{
	return quat(0.0f);
}

quat quat::Intermediate (const quat& q0, const quat& q1, const quat& q2)
{
	return quat(0.0f);
}

quat quat::Squad(Real t, const quat& q0, const quat& a, const quat& b, const quat& q1)
{
	return Slerp(2.0 * t * (1.0 - t), Slerp(t, q0, q1), Slerp(t, a, b));
}

quat quat::ProjectToAxis(const quat& q, vec3 axis)
{
	axis.Normalize();
	vec3 qv = vec3(q[vx], q[vy], q[vz]);
	Real angle = acos(q.n[vw]);
	Real sn = sin(angle);
	vec3 qaxis = qv / sn;
	qaxis.Normalize();
	angle = qaxis * axis;
	Real halfTheta;
	if (angle < EPSILON)
	{
		halfTheta = 0.0f;
	}else
	{
		Real s = axis * qv;
		Real c = q[vw];
		halfTheta = atan2(s, c);
	}	
	Real cn = cos(halfTheta);
	sn = sin(halfTheta);
	return quat(cn, sn * axis[vx], sn * axis[vy], sn * axis[vz]);
}

quat quat::ProjectToAxis2(const quat& q, vec3 axis)
{
	axis.Normalize();
	vec3 qv = vec3(q[vx], q[vy], q[vz]);
	Real angle = acos(q.n[vw]);
	Real sn = sin(angle);
	vec3 qaxis = qv / sn;
	qaxis.Normalize();
	angle = qaxis * axis;
	Real halfTheta;
	if (fabs(angle) < EPSILON)
	{
		halfTheta = 0.0f;
	}else
	{
		sn = axis * qv;
		//Real c = q[vw];
		halfTheta = asin(sn);
	}	
	Real cn = cos(halfTheta);
	return quat(cn, sn * axis[vx], sn * axis[vy], sn * axis[vz]);
}

// Conversion functions
void quat::ToAxisAngle (vec3& axis, Real& angleRad) const
{
	Real length2 = n[vx]*n[vx] + n[vy]*n[vy] + n[vz]*n[vz];
	if (length2 < EPSILON) {
		angleRad = 0.0;
		axis[vx] = 0.0;
		axis[vy] = 1.0;
		axis[vz] = 0.0;
	}
	else {
		Real length = sqrt(length2);
		angleRad = 2.0 * acos(n[vw]);
		axis[vx] = n[vx]/length;
		axis[vy] = n[vy]/length;
		axis[vz] = n[vz]/length;
		if (angleRad > M_PI) { // always use the smaller angle
			angleRad = M2_PI - angleRad;
			axis *= -1.0;
		}
	}
}

void quat::FromAxisAngle (const vec3& axis, Real angleRad)
{
	Real fHalfAngle = angleRad * 0.5;
	Real sn = sin(fHalfAngle);
	n[vw] = cos(fHalfAngle);
	n[vx] = axis[vx] * sn;
	n[vy] = axis[vy] * sn;
	n[vz] = axis[vz] * sn;
}

void quat::FromAxisXAngle(Real angleRad)
{
	Real fHalfAngle = angleRad * 0.5;
	n[vw] = cos(fHalfAngle);
	n[vx] = sin(fHalfAngle);
	n[vy] = n[vz] = 0.0f;
}

void quat::FromAxisYAngle(Real angleRad)
{
	Real fHalfAngle = angleRad * 0.5;
	n[vw] = cos(fHalfAngle);
	n[vy] = sin(fHalfAngle);
	n[vx] = n[vz] = 0.0f;
}

void quat::FromAxisZAngle(Real angleRad)
{
	Real fHalfAngle = angleRad * 0.5;
	n[vw] = cos(fHalfAngle);
	n[vz] = sin(fHalfAngle);
	n[vx] = n[vy] = 0.0f;
}

mat3 quat::ToRotation () const
{
	// operations (*,+,-) = 24
	Real tx  = 2.0 * n[vx];
	Real ty  = 2.0 * n[vy];
	Real tz  = 2.0 * n[vz];
	Real twx = tx * n[vw];
	Real twy = ty * n[vw];
	Real twz = tz * n[vw];
	Real txx = tx * n[vx];
	Real txy = ty * n[vx];
	Real txz = tz * n[vx];
	Real tyy = ty * n[vy];
	Real tyz = tz * n[vy];
	Real tzz = tz * n[vz];

	mat3 m;
	m[0][0] = 1.0 - tyy - tzz;
	m[0][1] = txy - twz;
	m[0][2] = txz + twy;
	m[1][0] = txy + twz;
	m[1][1] = 1.0 - txx - tzz;
	m[1][2] = tyz - twx;
	m[2][0] = txz - twy;
	m[2][1] = tyz + twx;
	m[2][2] = 1.0 - txx - tyy;
	return m;
}
void quat::FromRotation (const mat3& rot)
{
}

#ifdef ALGEBRAIOSTREAMS
ostream& operator << (ostream& s, const quat& q)
{ 
	return s << "[ " << q.n[vw] << ' ' << q.n[vx] << ' ' << q.n[vy] << ' ' << q.n[vz] << " ]"; 
}
#endif // ALGEBRAIOSTREAMS

