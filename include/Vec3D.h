/*******************************************************************************
Copyright (c) 2012, Jonathan Hiller

This file is part of the AMF Tools suite. http://amf.wikispaces.com/
AMF Tools is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
AMF Tools is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
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

template <typename T = double> //<typename T=vfloat> ?
class Vec3D {
public:
	//Constructors
	Vec3D() :x(0), y(0), z(0) {}
	Vec3D(const Vec3D& s) {x = s.x; y = s.y; z = s.z;} //copy constructor
	Vec3D(const T dx, const T dy, const T dz) {x = dx; y = dy; z = dz;}

	//Stuff to make code with mixed template parameters work...
	template <typename U> Vec3D<T>(const Vec3D<U>& s) {x = (T)s.x; y = (T)s.y; z = (T)s.z;} //copy constructor
	template <typename U> operator Vec3D<U>() const {return Vec3D<U>(x, y, z);} //overload conversion operator for different template types?
	template <typename U> Vec3D<T> operator=(const Vec3D<U>& s) {x = s.x; y = s.y; z = s.z; return *this; }; //overload equals
	template <typename U> const Vec3D<T> operator+(const Vec3D<U>& s){return Vec3D<T>(x+s.x, y+s.y, z+s.z);}
	template <typename U> const Vec3D<T> operator-(const Vec3D<U>& s){return Vec3D<T>(x-s.x, y-s.y, z-s.z);}
	template <typename U> const Vec3D<T> operator*(const U& f) const {return Vec3D<T>(f*x, f*y, f*z);}
	template <typename U> const friend Vec3D<T> operator*(const U f, const Vec3D<T>& v) {return v*f;} //must be friend because float operator is first...
	template <typename U> const Vec3D<T>& operator+=(const Vec3D<U>& s) {x += s.x; y += s.y; z += s.z; return *this;} //add and set
	template <typename U> const Vec3D<T>& operator-=(const Vec3D<U>& s) {x -= s.x; y -= s.y; z -= s.z; return *this;} //subract and set

	//Operators
	inline Vec3D& operator=(const Vec3D& s) {x = s.x; y = s.y; z = s.z; return *this; }; //overload equals
	const Vec3D operator+(const Vec3D &v) const {return Vec3D(x+v.x, y+v.y, z+v.z);}
	const Vec3D operator-(const Vec3D &v) const {return Vec3D(x-v.x, y-v.y, z-v.z);}
	const Vec3D operator-() const {return Vec3D(-x, -y, -z);} //Negation (unary)
	const Vec3D operator*(const T &f) const {return Vec3D(f*x, f*y, f*z);}
	const friend Vec3D operator*(const T f, const Vec3D& v) {return v*f;} //must be friend because float operator is first...
	const Vec3D operator/(const T &f) const {T Inv = (T)1.0/f; return Vec3D(Inv*x, Inv*y, Inv*z);}
	bool operator==(const Vec3D& v) {return (x==v.x && y==v.y && z==v.z);} //Is equal
	bool operator!=(const Vec3D& v) {return !(*this==v);} //Is not equal
	const Vec3D& operator+=(const Vec3D& s) {x += s.x; y += s.y; z += s.z; return *this;} //add and set
	const Vec3D& operator-=(const Vec3D& s) {x -= s.x; y -= s.y; z -= s.z; return *this;} //subract and set
	const Vec3D& operator*=(const T f) {x *= f; y *= f; z *= f; return *this;} //multiply and set
	const Vec3D& operator/=(const T f) {T Inv = (T)1.0/f; x *= Inv; y *= Inv; z *= Inv; return *this;} //subtract and set
	const T& operator[](int index) const {switch (index%3){case vec3_X: return x; case vec3_Y: return y; /*case vec3_Z:*/ default: return z;}}
	T& operator[](int index) {switch (index%3){case vec3_X: return x; case vec3_Y: return y; /*case vec3_Z:*/ default: return z;}}

#ifdef WIN32
	bool IsValid() const {return !_isnan((double)x) && _finite((double)x) && !_isnan((double)y) && _finite((double)y) && !_isnan((double)z) && _finite((double)z);} //is a valid vector? (funky windows _ versions...)
#else
	bool IsValid() const {return !__isnand(x) && finite(x) && !__isnand(y) && finite(y) && !__isnand(z) && finite(z);} //is a valid vector?
#endif

	bool IsNear(const Vec3D& s, const T thresh) {return Dist2(s)<thresh*thresh;}

	//Attributes
	T x, y, z;
	T getX(void) const {return x;};
	T getY(void) const {return y;};
	T getZ(void) const {return z;};
	void setX(const T XIn) {x = XIn;};
	void setY(const T YIn) {y = YIn;};
	void setZ(const T ZIn) {z = ZIn;};

	//Vector operations (change this object)
	T Normalize() {T l = sqrt(x*x+y*y+z*z); if (l > 0) {x /= l;y /= l;z /= l;} return l;} //normalizes this vector
	void NormalizeFast() {T l = sqrt(x*x+y*y+z*z); if (l>0) {T li = 1.0/l;	x*=li; y*=li; z*=li;}} //Make it slightly quicker without return value... 
	Vec3D Rot(const Vec3D u, const T a) {T ca = cos(a); T sa = sin(a); T t = 1-ca; return Vec3D((u.x*u.x*t + ca) * x + (u.x*u.y*t - u.z*sa) * y + (u.z*u.x*t + u.y*sa) * z, (u.x*u.y*t + u.z*sa) * x + (u.y*u.y*t+ca) * y + (u.y*u.z*t - u.x*sa) * z, (u.z*u.x*t - u.y*sa) * x + (u.y*u.z*t + u.x*sa) * y + (u.z*u.z*t + ca) * z);} //rotates by arbitrary vector arbitrary amount (http://www.cprogramming.com/tutorial/3d/rotation.html (Errors! LH one is actually RH one))

	void RotZ(const T a) {T xt =  x*cos(a) - y*sin(a); T yt = x*sin(a) + y*cos(a); x = xt; y = yt;} //rotates about Z axis "a" radians
	void RotY(const T a) {T xt =  x*cos(a) + z*sin(a); T zt = -x*sin(a) + z*cos(a); x = xt; z = zt;} //rotates about Y axis "a" radians
	void RotX(const T a) {T yt =  y*cos(a) + z*sin(a); T zt = -y*sin(a) + z*cos(a); y = yt; z = zt;} //rotates about X axis "a" radians

	//Vector operations (don't change this object!)
	Vec3D Cross(const Vec3D& v) const {return Vec3D(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x);} //Cross product
	T Dot(const Vec3D& v) const {return (x * v.x + y * v.y + z * v.z);} //Dot product
	Vec3D Abs() const {return Vec3D(x>=0?x:-x, y>=0?y:-y, z>=0?z:-z);} //Absolute value
	Vec3D Normalized() const {T l = sqrt(x*x+y*y+z*z); return (l>0?(*this)/l:(*this));} //returns normalized vec
	T Length() const {return sqrt(x*x+y*y+z*z);} //length of the vector
	T Length2() const {return (x*x+y*y+z*z);} //length squared of the vector
	Vec3D Min(const Vec3D& s) const {return Vec3D(x<s.x ? x:s.x, y<s.y ? y:s.y, z<s.z ? z:s.z);} //min vector of the two
	Vec3D Max(const Vec3D& s) const {return Vec3D(x>s.x ? x:s.x, y>s.y ? y:s.y, z>s.z ? z:s.z);} //max vector of the two
	T Min() const {T Min1 = (x<y ? x:y); return (z<Min1 ? z:Min1);} //minimum element of this vector
	T Max() const {T Max1 = (x>y ? x:y); return (z>Max1 ? z:Max1);} //max element of this vector
	Vec3D Scale(const Vec3D& v) const {return Vec3D(x*v.x, y*v.y, z*v.z);} //scales by another vector
	Vec3D ScaleInv(const Vec3D& v) const {return Vec3D(x/v.x, y/v.y, z/v.z);} //scales by inverse of another vector
	T Dist(const Vec3D& v) const {return sqrt(Dist2(v));} //distance from another vector
	T Dist2(const Vec3D& v) const {return (v.x-x)*(v.x-x)+(v.y-y)*(v.y-y)+(v.z-z)*(v.z-z);} //distance from another vector
	T AlignWith(const Vec3D target, Vec3D& rotax) const {Vec3D thisvec = Normalized(); Vec3D targvec = target.Normalized(); Vec3D rotaxis = thisvec.Cross(targvec); if (rotaxis.Length2() == 0) {rotaxis=target.ArbitraryNormal();} rotax = rotaxis.Normalized(); return acos(thisvec.Dot(targvec));} //returns vector (rotax) and amount (return val) to align this vector with target vector
	Vec3D ArbitraryNormal() const {Vec3D n = Normalized(); if (fabs(n.x) <= fabs(n.y) && fabs(n.x) <= fabs(n.z)){n.x = 1;} else if (fabs(n.y) <= fabs(n.x) && fabs(n.y) <= fabs(n.z)){n.y = 1;}	else {n.z = 1;}	return Cross(n).Normalized();} //generate arbitrary normal
};

#endif //_VEC3D_H
