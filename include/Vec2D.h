/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef _VEC2D_H
#define _VEC2D_H

//Possible Linux portability issues: min, max

#include <math.h>
#include <float.h>
#include <iostream>

//indices for each direction
#define vec2_X 0
#define vec2_Y 1

//! A generic 3D vector template
/*!
The template parameter is assumed to be either float or double depending on the desired numerical resolution.
*/
template <typename T = double>
class Vec2D {
public:
	T x; //!< The current X value.
	T y; //!< The current Y value.

	//Constructors
	Vec2D() :x(0), y(0) {} //!< Constructor. Initialzes x, y to zero.
	Vec2D(const T dx, const T dy) {x = dx; y = dy;} //!< Constructor with specified individual values.
	Vec2D(const T * pBegin) {x = *pBegin; y = *(pBegin+1);} //!< Constructor to create a Vec2D from two values in consectutive memory locations (i.e. in a vector). @param[in] pBegin pointer to the x value. Must be followed immediately in memory by y.
	Vec2D(const Vec2D& s) {x = s.x; y = s.y;} //!< Copy constructor.

#ifdef WIN32
	bool IsValid() const {return !_isnan((double)x) && _finite((double)x) && !_isnan((double)y) && _finite((double)y);} //!< Returns true if all values are valid numbers.
#else
	bool IsValid() const {return !__isnand(x) && finite(x) && !__isnand(y) && finite(y);} //!< Returns true if all values are valid numbers.
#endif

	//Stuff to make code with mixed template parameters work...
	template <typename U> Vec2D<T>(const Vec2D<U>& s)						{x = (T)s.x; y = (T)s.y;} //!< Copy constructor from another template type
	template <typename U> operator Vec2D<U>() const							{return Vec2D<U>(x, y);} //!< overload conversion operator for different template types
	template <typename U> Vec2D<T> operator=(const Vec2D<U>& s)				{x = s.x; y = s.y; return *this; } //!< equals operator for different template types
	template <typename U> const Vec2D<T> operator+(const Vec2D<U>& s)		{return Vec2D<T>(x+s.x, y+s.y);} //!< addition operator for different template types
	template <typename U> const Vec2D<T> operator-(const Vec2D<U>& s)		{return Vec2D<T>(x-s.x, y-s.y);} //!< subtraction operator for different template types
	template <typename U> const Vec2D<T> operator*(const U& f) const		{return Vec2D<T>(f*x, f*y);} //!< multiplication operator for different template types
	template <typename U> const friend Vec2D<T> operator*(const U f, const Vec2D<T>& v) {return v*f;} //!< multiplication operator for different template types with number first. Therefore must be a friend because scalar value comes first.
	template <typename U> const Vec2D<T>& operator+=(const Vec2D<U>& s)		{x += s.x; y += s.y; return *this;} //!< add and set for different template types
	template <typename U> const Vec2D<T>& operator-=(const Vec2D<U>& s)		{x -= s.x; y -= s.y; return *this;} //!< subract and set for different template types

	//Operators
	inline Vec2D& operator=(const Vec2D& s)			{x = s.x; y = s.y; return *this; } //!< overload equals.
	const Vec2D operator+(const Vec2D &v) const		{return Vec2D(x+v.x, y+v.y);} //!< overload addition.
	const Vec2D operator-(const Vec2D &v) const		{return Vec2D(x-v.x, y-v.y);} //!< overload subtraction.
	const Vec2D operator-() const					{return Vec2D(-x, -y);} //!< overload negation (unary).
	const Vec2D operator*(const T &f) const			{return Vec2D(f*x, f*y);} //!< overload multiplication.
	const friend Vec2D operator*(const T f, const Vec2D& v) {return v*f;} //!< overload multiplication with number first.
	const Vec2D operator/(const T &f) const			{T Inv = (T)1.0/f; return Vec2D(Inv*x, Inv*y);} //!< overload division.
	bool operator==(const Vec2D& v)					{return (x==v.x && y==v.y);} //!< overload is equal.
	bool operator!=(const Vec2D& v)					{return !(*this==v);} //!< overload is not equal.
	const Vec2D& operator+=(const Vec2D& s)			{x += s.x; y += s.y; return *this;} //!< overload add and set
	const Vec2D& operator-=(const Vec2D& s)			{x -= s.x; y -= s.y; return *this;} //!< overload subract and set
	const Vec2D& operator*=(const T f)				{x *= f; y *= f; return *this;} //!< overload multiply and set
	const Vec2D& operator/=(const T f)				{T Inv = (T)1.0/f; x *= Inv; y *= Inv; return *this;} //!< overload divide and set
	const T& operator[](const int axis) const		{switch (axis%3){case vec3_X: return x; /*case vec3_Y:*/ default: return y;}} //!< overload index operator. 0 ("vec3_X") is x, 1 ("vec3_Y") is y.
	T& operator[](const int axis)					{switch (axis%3){case vec3_X: return x; /*case vec3_Y:*/ default: return y;}}  //!< overload  index operator. 0 ("vec3_X") is x, 1 ("vec3_Y") is y.


	//Attributes
	T		getX(void) const	{return x;} //!< returns the x value
	T		getY(void) const	{return y;} //!< returns the y value
	void	setX(const T XIn)	{x = XIn;} //!< sets the x value
	void	setY(const T YIn)	{y = YIn;} //!< sets the y value

	//Vector operations (change this object)
	T		Normalize()						{T l = sqrt(x*x+y*y); if (l > 0) {x /= l;y /= l;} return l;} //!< Normalizes this vector. Returns the previous magnitude of this vector before normalization. Note: function changes this vector.
	void	NormalizeFast()					{T l = sqrt(x*x+y*y); if (l>0) {T li = 1.0/l;	x*=li; y*=li;}} //!< Normalizes this vector slightly faster than Normalize() by not returning a value. Note: function changes this vector.
	void	Rot(const T a)					{T xt =  x*cos(a) - y*sin(a); T yt = x*sin(a) + y*cos(a); x = xt; y = yt;} //!< Rotates this vector "a" radians. Note: function changes this vector. @param[in] a Radians to rotate by.

	//Vector operations (don't change this object!)
	T		Dot(const Vec2D& v) const		{return (x * v.x + y * v.y);} //!< Returns the dot product of this vector dotted with the provided vector "v". This vector is not modified. @param[in] v Vector to dot with.
	Vec2D	Abs() const						{return Vec2D(x>=0?x:-x, y>=0?y:-y);} //!< Returns the absolute value of this vector. This vector is not modified.
	Vec2D	Normalized() const				{T l = sqrt(x*x+y*y); return (l>0?(*this)/l:(*this));} //!< Returns a normalized version of this vector. Unlike Normalize() or NormalizeFast(), this vector is not modified.
	bool	IsNear(const Vec2D& s, const T thresh) {return Dist2(s)<thresh*thresh;} //!< Returns true if this vector is within "thresh" distance of the specified vector "s". Neither vector is modified. @param[in] s vector to compare this position to. @param[in] thresh Euclidian distance to determine if the other vector is within.
	T		Length() const					{return sqrt(x*x+y*y);} //!< Returns the length (magnitude) of this vector. This vector is not modified.
	T		Length2() const					{return (x*x+y*y);} //!< Returns the length (magnitude) squared of this vector. This vector is not modified.
	Vec2D	Min(const Vec2D& s) const		{return Vec2D(x<s.x ? x:s.x, y<s.y ? y:s.y);} //!< Returns a vector populated by the minimum x and y value of this vector and the specified vector "s". This vector is not modified. @param[in] s Second vector to consider.
	Vec2D	Max(const Vec2D& s) const		{return Vec2D(x>s.x ? x:s.x, y>s.y ? y:s.y);} //!< Returns a vector populated by the maximum x and y value of this vector and the specified vector "s". This vector is not modified. @param[in] s Second vector to consider.
	T		Min() const						{return x<y ? x:y;} //!< Returns the smallest of x or y of this vector. This vector is not modified.
	T		Max() const						{return x>y ? x:y;} //!< Returns the largest of x or y of this vector. This vector is not modified.
	Vec2D	Scale(const Vec2D& v) const		{return Vec2D(x*v.x, y*v.y);} //!< Returns a vector where each value of this vector is scaled by its respective value in vector "v". This vector is not modified. @param[in] v Vector with scaling values.
	Vec2D	ScaleInv(const Vec2D& v) const	{return Vec2D(x/v.x, y/v.y);} //!< Returns a vector where each value of this vector is inversely scaled by its respective value in vector "v". This vector is not modified. @param[in] v Vector with scaling values.
	T		Dist(const Vec2D& v) const		{return sqrt(Dist2(v));} //!< Returns the euclidian distance between this vector and the specified vector "v". This vector is not modified. @param[in] v Vector to compare with.
	T		Dist2(const Vec2D& v) const		{return (v.x-x)*(v.x-x)+(v.y-y)*(v.y-y);} //!< Returns the euclidian distance squared between this vector and the specified vector "v". This vector is not modified. @param[in] v Vector to compare with.

};

typedef Vec2D<float> Vec2Df;
typedef Vec2D<double> Vec2Dd;

template <typename U> std::ostream &operator<<(std::ostream &os, Vec2D<U> const &v) { 
    return os << v.x; // << "\t" << v.y << "\t" << v.z << "\t";
}

#endif //_VEC2D_H
