/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef _VEC3D_H
#define _VEC3D_H

//Possible Linux portability issues: min, max

#include <math.h>
#include <float.h>

//indices for each direction
#define vec3_X 0
#define vec3_Y 1
#define vec3_Z 2

//! A generic 3D vector template
/*!
The template parameter is assumed to be either float or double depending on the desired numerical resolution.
*/
template <typename T = double>
class Vec3D {
public:
	T x; //!< The current X value.
	T y; //!< The current Y value.
	T z; //!< The current Z value.

	//Constructors
	Vec3D() :x(0), y(0), z(0) {} //!< Constructor. Initialzes x, y, z to zero.
	Vec3D(const T dx, const T dy, const T dz) {x = dx; y = dy; z = dz;} //!< Constructor with specified individual values.
	Vec3D(const Vec3D& s) {x = s.x; y = s.y; z = s.z;} //!< Copy constructor.

#ifdef WIN32
	bool IsValid() const {return !_isnan((double)x) && _finite((double)x) && !_isnan((double)y) && _finite((double)y) && !_isnan((double)z) && _finite((double)z);} //!< Returns true if all values are valid numbers.
#else
	bool IsValid() const {return !__isnand(x) && finite(x) && !__isnand(y) && finite(y) && !__isnand(z) && finite(z);} //!< Returns true if all values are valid numbers.
#endif

	//Stuff to make code with mixed template parameters work...
	template <typename U> Vec3D<T>(const Vec3D<U>& s)						{x = (T)s.x; y = (T)s.y; z = (T)s.z;} //!< Copy constructor from another template type
	template <typename U> operator Vec3D<U>() const							{return Vec3D<U>(x, y, z);} //!< overload conversion operator for different template types
	template <typename U> Vec3D<T> operator=(const Vec3D<U>& s)				{x = s.x; y = s.y; z = s.z; return *this; } //!< equals operator for different template types
	template <typename U> const Vec3D<T> operator+(const Vec3D<U>& s)		{return Vec3D<T>(x+s.x, y+s.y, z+s.z);} //!< addition operator for different template types
	template <typename U> const Vec3D<T> operator-(const Vec3D<U>& s)		{return Vec3D<T>(x-s.x, y-s.y, z-s.z);} //!< subtraction operator for different template types
	template <typename U> const Vec3D<T> operator*(const U& f) const		{return Vec3D<T>(f*x, f*y, f*z);} //!< multiplication operator for different template types
	template <typename U> const friend Vec3D<T> operator*(const U f, const Vec3D<T>& v) {return v*f;} //!< multiplication operator for different template types with number first. Therefore must be a friend because scalar value comes first.
	template <typename U> const Vec3D<T>& operator+=(const Vec3D<U>& s)		{x += s.x; y += s.y; z += s.z; return *this;} //!< add and set for different template types
	template <typename U> const Vec3D<T>& operator-=(const Vec3D<U>& s)		{x -= s.x; y -= s.y; z -= s.z; return *this;} //!< subract and set for different template types

	//Operators
	inline Vec3D& operator=(const Vec3D& s)			{x = s.x; y = s.y; z = s.z; return *this; } //!< overload equals.
	const Vec3D operator+(const Vec3D &v) const		{return Vec3D(x+v.x, y+v.y, z+v.z);} //!< overload addition.
	const Vec3D operator-(const Vec3D &v) const		{return Vec3D(x-v.x, y-v.y, z-v.z);} //!< overload subtraction.
	const Vec3D operator-() const					{return Vec3D(-x, -y, -z);} //!< overload negation (unary).
	const Vec3D operator*(const T &f) const			{return Vec3D(f*x, f*y, f*z);} //!< overload multiplication.
	const friend Vec3D operator*(const T f, const Vec3D& v) {return v*f;} //!< overload multiplication with number first.
	const Vec3D operator/(const T &f) const			{T Inv = (T)1.0/f; return Vec3D(Inv*x, Inv*y, Inv*z);} //!< overload division.
	bool operator==(const Vec3D& v)					{return (x==v.x && y==v.y && z==v.z);} //!< overload is equal.
	bool operator!=(const Vec3D& v)					{return !(*this==v);} //!< overload is not equal.
	const Vec3D& operator+=(const Vec3D& s)			{x += s.x; y += s.y; z += s.z; return *this;} //!< overload add and set
	const Vec3D& operator-=(const Vec3D& s)			{x -= s.x; y -= s.y; z -= s.z; return *this;} //!< overload subract and set
	const Vec3D& operator*=(const T f)				{x *= f; y *= f; z *= f; return *this;} //!< overload multiply and set
	const Vec3D& operator/=(const T f)				{T Inv = (T)1.0/f; x *= Inv; y *= Inv; z *= Inv; return *this;} //!< overload divide and set
	const T& operator[](int index) const			{switch (index%3){case vec3_X: return x; case vec3_Y: return y; /*case vec3_Z:*/ default: return z;}} //!< overload index operator. 0 ("vec3_X") is x, 1 ("vec3_Y") is y and 2 ("vec3_Z") is z.
	T& operator[](int index)						{switch (index%3){case vec3_X: return x; case vec3_Y: return y; /*case vec3_Z:*/ default: return z;}}  //!< overload  index operator. 0 ("vec3_X") is x, 1 ("vec3_Y") is y and 2 ("vec3_Z") is z.

	//Attributes
	T		getX(void) const	{return x;} //!< returns the x value
	T		getY(void) const	{return y;} //!< returns the y value
	T		getZ(void) const	{return z;} //!< returns the z value
	void	setX(const T XIn)	{x = XIn;} //!< sets the x value
	void	setY(const T YIn)	{y = YIn;} //!< sets the y value
	void	setZ(const T ZIn)	{z = ZIn;} //!< sets the z value

	//Vector operations (change this object)
	T		Normalize()			{T l = sqrt(x*x+y*y+z*z); if (l > 0) {x /= l;y /= l;z /= l;} return l;} //!< Normalizes this vector. Returns the previous magnitude of this vector before normalization. Note: function changes this vector.
	void	NormalizeFast()		{T l = sqrt(x*x+y*y+z*z); if (l>0) {T li = 1.0/l;	x*=li; y*=li; z*=li;}} //!< Normalizes this vector slightly faster than Normalize() by not returning a value. Note: function changes this vector.
	Vec3D	Rot(const Vec3D u, const T a) {T ca = cos(a); T sa = sin(a); T t = 1-ca; return Vec3D((u.x*u.x*t + ca) * x + (u.x*u.y*t - u.z*sa) * y + (u.z*u.x*t + u.y*sa) * z, (u.x*u.y*t + u.z*sa) * x + (u.y*u.y*t+ca) * y + (u.y*u.z*t - u.x*sa) * z, (u.z*u.x*t - u.y*sa) * x + (u.y*u.z*t + u.x*sa) * y + (u.z*u.z*t + ca) * z);} //!< Rotates this vector about an axis definied by "u" an angle "a". (http://www.cprogramming.com/tutorial/3d/rotation.html) Note: function changes this vector. @param[in] u axis to rotate this vector about. Must be normalized. @param[in] a The amount to rotate in radians.
	void	RotZ(const T a)		{T xt =  x*cos(a) - y*sin(a); T yt = x*sin(a) + y*cos(a); x = xt; y = yt;} //!< Rotates this vector about the Z axis "a" radians. Note: function changes this vector. @param[in] a Radians to rotate by.
	void	RotY(const T a)		{T xt =  x*cos(a) + z*sin(a); T zt = -x*sin(a) + z*cos(a); x = xt; z = zt;} //!< Rotates this vector about the Y axis "a" radians. Note: function changes this vector. @param[in] a Radians to rotate by.
	void	RotX(const T a)		{T yt =  y*cos(a) + z*sin(a); T zt = -y*sin(a) + z*cos(a); y = yt; z = zt;} //!< Rotates this vector about the X axis "a" radians. Note: function changes this vector. @param[in] a Radians to rotate by.

	//Vector operations (don't change this object!)
	Vec3D	Cross(const Vec3D& v) const		{return Vec3D(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x);} //!< Returns the cross product of this vector crossed by the provided vector "v". This vector is not modified. @param[in] v Vector to cross by.
	T		Dot(const Vec3D& v) const		{return (x * v.x + y * v.y + z * v.z);} //!< Returns the dot product of this vector dotted with the provided vector "v". This vector is not modified. @param[in] v Vector to dot with.
	Vec3D	Abs() const						{return Vec3D(x>=0?x:-x, y>=0?y:-y, z>=0?z:-z);} //!< Returns the absolute value of this vector. This vector is not modified.
	Vec3D	Normalized() const				{T l = sqrt(x*x+y*y+z*z); return (l>0?(*this)/l:(*this));} //!< Returns a normalized version of this vector. Unlike Normalize() or NormalizeFast(), this vector is not modified.
	bool	IsNear(const Vec3D& s, const T thresh) {return Dist2(s)<thresh*thresh;} //!< Returns true if this vector is within "thresh" distance of the specified vector "s". Neither vector is modified. @param[in] s vector to compare this position to. @param[in] thresh Euclidian distance to determine if the other vector is within.
	T		Length() const					{return sqrt(x*x+y*y+z*z);} //!< Returns the length (magnitude) of this vector. This vector is not modified.
	T		Length2() const					{return (x*x+y*y+z*z);} //!< Returns the length (magnitude) squared of this vector. This vector is not modified.
	Vec3D	Min(const Vec3D& s) const		{return Vec3D(x<s.x ? x:s.x, y<s.y ? y:s.y, z<s.z ? z:s.z);} //!< Returns a vector populated by the minimum x, y, and z value of this vector and the specified vector "s". This vector is not modified. @param[in] s Second vector to consider.
	Vec3D	Max(const Vec3D& s) const		{return Vec3D(x>s.x ? x:s.x, y>s.y ? y:s.y, z>s.z ? z:s.z);} //!< Returns a vector populated by the maximum x, y, and z value of this vector and the specified vector "s". This vector is not modified. @param[in] s Second vector to consider.
	T		Min() const						{T Min1 = (x<y ? x:y); return (z<Min1 ? z:Min1);} //!< Returns the smallest of x, y, or z of this vector. This vector is not modified.
	T		Max() const						{T Max1 = (x>y ? x:y); return (z>Max1 ? z:Max1);} //!< Returns the largest of x, y, or z of this vector. This vector is not modified.
	Vec3D	Scale(const Vec3D& v) const		{return Vec3D(x*v.x, y*v.y, z*v.z);} //!< Returns a vector where each value of this vector is scaled by its respective value in vector "v". This vector is not modified. @param[in] v Vector with scaling values.
	Vec3D	ScaleInv(const Vec3D& v) const	{return Vec3D(x/v.x, y/v.y, z/v.z);} //!< Returns a vector where each value of this vector is inversely scaled by its respective value in vector "v". This vector is not modified. @param[in] v Vector with scaling values.
	T		Dist(const Vec3D& v) const		{return sqrt(Dist2(v));} //!< Returns the euclidian distance between this vector and the specified vector "v". This vector is not modified. @param[in] v Vector to compare with.
	T		Dist2(const Vec3D& v) const		{return (v.x-x)*(v.x-x)+(v.y-y)*(v.y-y)+(v.z-z)*(v.z-z);} //!< Returns the euclidian distance squared between this vector and the specified vector "v". This vector is not modified. @param[in] v Vector to compare with.
	T		AlignWith(const Vec3D target, Vec3D& rotax) const {Vec3D thisvec = Normalized(); Vec3D targvec = target.Normalized(); Vec3D rotaxis = thisvec.Cross(targvec); if (rotaxis.Length2() == 0) {rotaxis=target.ArbitraryNormal();} rotax = rotaxis.Normalized(); return acos(thisvec.Dot(targvec));} //!< Returns a rotation amount in radians and a unit vector (returned via the rotax argument) that will align this vector with target vector "target'. This vector is not modified. @param[in] target target vector. @param[out] rotax Unit vector of rotation axis.
	Vec3D	ArbitraryNormal() const			{Vec3D n = Normalized(); if (fabs(n.x) <= fabs(n.y) && fabs(n.x) <= fabs(n.z)){n.x = 1;} else if (fabs(n.y) <= fabs(n.x) && fabs(n.y) <= fabs(n.z)){n.y = 1;}	else {n.z = 1;}	return Cross(n).Normalized();} //!< Generates and returns an arbitrary vector that is normal to this one. This vector is not modified. 
};

#endif //_VEC3D_H
