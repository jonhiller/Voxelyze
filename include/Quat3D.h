/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef _QUAT3D_H
#define _QUAT3D_H

#include <math.h>
#include <float.h>
#include "Vec3D.h"

#define PI 3.14159265358979
#define DBL_EPSILONx24 5.328e-15 //DBL_EPSILON*24
//#define DBL_EPSILON_SQ_x6xROOT2 4.17e-31 //DBL_EPSILON*DBL_EPSILON*6*sqrt(2.0)

//The following values are calculated based on: MAX_ERROR_PERCENT 1e-4
#define DISCARD_ANGLE_RAD 1e-7 //Anything less than this angle can be considered 0
#define SMALL_ANGLE_RAD 1.732e-2 //Angles less than this get small angle approximations. To get: Root solve atan(t)/t-1+MAX_ERROR_PERCENT. From: MAX_ERROR_PERCENT = (t-atan(t))/t 
#define SMALL_ANGLE_W 0.9999625 //quaternion W value corresponding to a SMALL_ANGLE_RAD. To calculate, cos(SMALL_ANGLE_RAD*0.5).
#define W_THRESH_ACOS2SQRT 0.9988 //Threshhold of w above which we can approximate acos(w) with sqrt(2-2w). To get: Root solve 1-sqrt(2-2wt)/acos(wt) - MAX_ERROR_PERCENT. From MAX_ERROR_PERCENT = (acos(wt)-sqrt(2-2wt))/acos(wt)
//#define SLTHRESH_DISCARD_ANGLE 1e-14; //SquareLength (x^2+y^2+z^2 comparison) threshhold for what constitutes a discard-small angle. From DISCARD_ANGLE_RAD ~= acos(w) and SquareLength also = 1-w*w. Must upcast to double or else truncated floats.
#define SLTHRESH_ACOS2SQRT 2.4e-3 //SquareLength threshhold for when we can use square root optimization for acos. From SquareLength = 1-w*w. (calculate according to 1.0-W_THRESH_ACOS2SQRT*W_THRESH_ACOS2SQRT

//quaternion properties (for reference)
//1) To rotate a vector V, form a quaternion with w = 0; To rotate by Quaternion Q, do Q*V*Q.Conjugate() and trim off the w component.
//2) To do multiple rotations: To Rotate by Q1 THEN Q2, Q2*Q1*V*Q1.Conjugate*Q2.Conjugate(), or make a Qtot = Q2*Q1 and do Qtot*V*Qtot.Conjucate()
//3) Q1*Q1.Conjugate - Identity
//4) To do a reverse rotation Q1, just do Q1.conjugate*V*Q1
//http://www.cprogramming.com/tutorial/3d/quaternions.html

//! A generic 3D quaternion template
/*!
The template parameter is assumed to be either float or double depending on the desired numerical resolution.
*/
template <typename T = double>
class Quat3D {
public:
	T w; //!< The current W value.
	T x; //!< The current X value.
	T y; //!< The current Y value.
	T z; //!< The current Z value.

	

	//constructors
	Quat3D(void) : w(1), x(0), y(0), z(0) {} //!< Constructor. Initialzes w, x, y, z to zero.
	Quat3D(const T dw, const T dx, const T dy, const T dz) {w=dw; x=dx; y=dy; z=dz;} //!< Constructor with specified individual values.
	Quat3D(const Quat3D& QuatIn) {w = QuatIn.w; x = QuatIn.x; y = QuatIn.y; z = QuatIn.z;} //!< Copy constructor
	Quat3D(const Vec3D<T>& VecIn) {FromRotationVector(VecIn);} //!< Constructs this quaternion from rotation vector VecIn. See FromRotationVector(). @param[in] VecIn A rotation vector.
	Quat3D(const T angle, const Vec3D<T> &axis){ const T a = angle * (T)0.5; const T s = sin(a); const T c = cos(a); w = c; x = axis.x * s; y = axis.y * s; z = axis.z * s; } //!< Constructs this quaternion from an angle in radians and a unit axis. @param[in] angle An angle in radians @param[in] axis A normalize rotation axis.
	Quat3D(const Vec3D<T> &RotateFrom, const Vec3D<T> &RotateTo){ 
		T theta = acos(RotateFrom.Dot(RotateTo)/sqrt(RotateFrom.Length2()*RotateTo.Length2())); //angle between vectors. from A.B=|A||B|cos(theta)
//		if (theta < DISCARD_ANGLE_RAD) {*this = Quat3D(1,0,0,0); return;} //very small angle, return no rotation
		if (theta <= 0) {*this = Quat3D(1,0,0,0); return;} //very small angle, return no rotation
		Vec3D<T> Axis = RotateFrom.Cross(RotateTo); //Axis of rotation
		Axis.NormalizeFast();
		if (theta > PI-DISCARD_ANGLE_RAD) {*this = Quat3D(Axis); return;} //180 degree rotation (180 degree rot about axis ax, ay, az is Quat(0,ax,ay,az))
		*this = Quat3D(theta, Axis); //otherwise for the quaternion from angle-axis. 
	} //!< Constructs this quaternion to represent the rotation from two vectors. The vectors need not be normalized and are not modified. @param[in] RotateFrom A vector representing a pre-rotation orientation. @param[in] RotateTo A vector representing a post-rotation orientation.

	//functions to make code with mixed template parameters work...
	template <typename U> Quat3D<T>(const Quat3D<U>& QuatIn) {w = QuatIn.w; x = QuatIn.x; y = QuatIn.y; z = QuatIn.z;} //!< Copy constructor from another template type
	template <typename U> Quat3D<T>(const Vec3D<U>& VecIn) {w = 0; x = VecIn.x; y = VecIn.y; z = VecIn.z;} //!< Copies x, y, z from the specified vector and sets w to zero.
	template <typename U> operator Quat3D<U>() const {return Quat3D<U>(w, x, y, z);} //!< overload conversion operator for different template types
	template <typename U> Quat3D<T> operator=(const Quat3D<U>& s) {w=s.w; x=s.x; y=s.y; z=s.z; return *this; } //!< Equals operator for different template types
	template <typename U> const Quat3D<T> operator+(const Quat3D<U>& s){return Quat3D<T>(w+s.w, x+s.x, y+s.y, z+s.z);} //!< Addition operator for different template types
	template <typename U> const Quat3D<T> operator*(const U& f) const {return Quat3D<T>(f*w, f*x, f*y, f*z);} //!< Scalar multiplication operator for different template types
	template <typename U> const Quat3D<T> operator*(const Quat3D<U>& f) const {return Quat3D(w*f.w - x*f.x - y*f.y - z*f.z, w*f.x + x*f.w + y*f.z - z*f.y, w*f.y - x*f.z + y*f.w + z*f.x, w*f.z + x*f.y - y*f.x + z*f.w);} //!< Quaternion multplication operator for different template types

	//overload operators
	Quat3D& operator=(const Quat3D& s) {w = s.w; x = s.x; y = s.y; z = s.z; return *this;} //!< overload equals
	const Quat3D operator+(const Quat3D& s) const {return Quat3D(w+s.w, x+s.x, y+s.y, z+s.z);} //!< overload additoon
	const Quat3D operator-(const Quat3D& s) const {return Quat3D(w-s.w, x-s.x, y-s.y, z-s.z);} //!< overload subtraction
	const Quat3D operator*(const T f) const {return Quat3D(w*f, x*f, y*f, z*f);} //!< overload scalar multiplication
	const Quat3D friend operator*(const T f, const Quat3D v) {return Quat3D(v.w*f, v.x*f, v.y*f, v.z*f);} //!< overload scalar multiplication with number first.
	const Quat3D operator*(const Quat3D& f) const {return Quat3D(w*f.w - x*f.x - y*f.y - z*f.z, w*f.x + x*f.w + y*f.z - z*f.y, w*f.y - x*f.z + y*f.w + z*f.x, w*f.z + x*f.y - y*f.x + z*f.w);} //!< overload quaternion multiplication.
	bool operator==(const Quat3D& s) const {return (w==s.w && x==s.x && y==s.y && z==s.z);} //!< overload is equal.
	bool operator!=(const Quat3D& s) const {return (w!=s.w || x!=s.x || y!=s.y || z!=s.z);} //!< overload is not equal.
	const Quat3D& operator+=(const Quat3D& s) {w += s.w; x += s.x; y += s.y; z += s.z; return *this;} //!< overload add and set
	const Quat3D& operator-=(const Quat3D& s) {w -= s.w; x -= s.x; y -= s.y; z -= s.z; return *this;} //!< overload subtract and set
	const Vec3D<T> ToVec() const {return Vec3D<T>(x, y, z);} //!< Explicit casting to a vector. Throws away w and copies x, y, z directly.

	//utilities
	const T Length() const {return sqrt(Length2());} //!< Returns the length (magnitude) of the quaternion.
	const T Length2() const {return (w*w+x*x+y*y+z*z);} //!< Returns the length (magnitude) squared of the quaternion.
	const T Normalize() {T l = Length(); if (l == 0){w = 1; x = 0; y = 0; z = 0;} else if (l > 0) {T li = 1.0/l; w*=li; x*=li; y*=li; z*=li;} return l;} //!< Normalizes this quaternion. Returns the previous magnitude of this quaternion before normalization. Note: function changes this quaternion.
	void NormalizeFast() {T l = sqrt(x*x+y*y+z*z+w*w); if (l!=0) {T li = 1.0/l;	w*=li; x*=li; y*=li; z*=li;} if (w>=1.0){w=1.0; x=0; y=0; z=0;}}  //!< Normalizes this quaternion slightly faster than Normalize() by not returning a value. Note: function changes this quaternion.
	const Quat3D Inverse() const {T n = w*w + x*x + y*y + z*z; return Quat3D(w/n, -x/n, -y/n, -z/n);} //!< Returns a quaternion that is the inverse of this quaternion. This quaternion is not modified. 
	const Quat3D Conjugate() const {return Quat3D(w, -x, -y, -z);} //!< Returns a quaternion that is the conjugate of this quaternion. This quaternion is not modified.

	//angle and/or axis calculations
	const T Angle() const {return 2.0*acos(w>1?1:w);} //!< Returns the angular rotation of this quaternion in radians.
	const T AngleDegrees() const {return Angle()*57.29577951308232;} //!< Returns the angular rotation of this quaternion in degrees.
	bool IsNegligibleAngle() const {return 2.0*acos(w) < DISCARD_ANGLE_RAD;} //!< Returns true if the angular rotation of this quaternion is likely to be considered negligible.
	bool IsSmallAngle() const {return w>SMALL_ANGLE_W;} //!< Returns true if the angular rotation of this quaternion is small enough to be a good candidate for small angle approximations.
	Vec3D<T> Axis() const { 
		T squareLength = 1.0-w*w; //because x*x + y*y + z*z + w*w = 1.0, but more susceptible to w noise
		if (squareLength <= 0){return Vec3D<T>(1,0,0);}
		else {return Vec3D<T>(x, y, z)/sqrt(squareLength);}
	} //!< Returns the normalized axis of rotation of this quaternion
	Vec3D<T> AxisUnNormalized() const {return Vec3D<T>(x, y, z);} //!< Returns the un0normalized axis of rotation of this quaternion
	void AngleAxis(T &angle, Vec3D<T> &axis) const {AngleAxisUnNormalized(angle, axis); axis.NormalizeFast();} //!< Returns the angle and a normalize axis that represent this quaternion's rotation. @param[out] angle The rotation angle in radians. @param[out] axis The rotation axis in normalized vector form.
	void AngleAxisUnNormalized(T &angle, Vec3D<T> &axis) const 
	{
		if (w >= 1.0){angle=0; axis=Vec3D<T>(1,0,0); return;}
		angle = 2.0*acos(w>1?1:w);
		axis = Vec3D<T>(x,y,z);
	} //!< Returns the angle and an un-normalized axis that represent this quaternion's rotation. @param[out] angle The rotation angle in radians. @param[out] axis The rotation axis in un-normalized vector form.
	
	const Vec3D<T> ToRotationVector() const {
		if (w >= 1.0 || w <= -1.0) return Vec3D<T>(0,0,0);
		T squareLength = 1.0-w*w; //because x*x + y*y + z*z + w*w = 1.0, but more susceptible to w noise (when 
		if (squareLength < SLTHRESH_ACOS2SQRT) return Vec3D<T>(x, y, z)*2.0*sqrt((2-2*w)/squareLength); //acos(w) = sqrt(2*(1-x)) for w close to 1. for w=0.001, error is 1.317e-6
		else return Vec3D<T>(x, y, z)*2.0*acos(w)/sqrt(squareLength);
	} //!< Returns a rotation vector representing this quaternion rotation. Adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
	
	void FromRotationVector(const Vec3D<T>& VecIn) { 
		Vec3D<T> theta = VecIn/2;
		T s, thetaMag2 = theta.Length2();
		if (thetaMag2*thetaMag2 < DBL_EPSILONx24 ){ //if the 4th taylor expansion term is negligible
			w=1.0 - 0.5*thetaMag2;
			s=1.0 - thetaMag2 / 6.0;
		}
		else {
			T thetaMag = sqrt(thetaMag2);
			w=cos(thetaMag);
			s=sin(thetaMag) / thetaMag;
		}
		x=theta.x*s;
		y=theta.y*s;
		z=theta.z*s;
	} //!< Overwrites this quaternion with values from the specified rotation vector. Adapted from http://physicsforgames.blogspot.com/2010/02/quaternions.html.  Note: function changes this quaternion. @param[in] VecIn A rotation vector to calculate this quaternion from.

	void FromAngleToPosX(const Vec3D<T>& RotateFrom){ //highly optimized at the expense of readability
		if (Vec3D<T>(0,0,0) == RotateFrom) return; //leave off if it slows down too much!!

		//Catch and handle small angle:
		T YoverX = RotateFrom.y/RotateFrom.x;
		T ZoverX = RotateFrom.z/RotateFrom.x;
		if (YoverX<SMALL_ANGLE_RAD && YoverX>-SMALL_ANGLE_RAD && ZoverX<SMALL_ANGLE_RAD && ZoverX>-SMALL_ANGLE_RAD){ //Intercept small angle and zero angle
			x=0; y=0.5*ZoverX; z=-0.5*YoverX;
			w = 1+0.5*(-y*y-z*z); //w=sqrt(1-x*x-y*y), small angle sqrt(1+x) ~= 1+x/2 at x near zero.
			return;
		}

		//more accurate non-small angle:
		Vec3D<> RotFromNorm = RotateFrom;
		RotFromNorm.NormalizeFast(); //Normalize the input...

		T theta = acos(RotFromNorm.x); //because RotFromNorm is normalized, 1,0,0 is normalized, and A.B = |A||B|cos(theta) = cos(theta)
		if (theta > PI-DISCARD_ANGLE_RAD) {w=0; x=0; y=1; z=0; return;} //180 degree rotation (about y axis, since the vector must be pointing in -x direction

		const T AxisMagInv = 1.0/sqrt(RotFromNorm.z*RotFromNorm.z+RotFromNorm.y*RotFromNorm.y);
		//Here theta is the angle, axis is RotFromNorm.Cross(Vec3D(1,0,0)) = Vec3D(0, RotFromNorm.z/AxisMagInv, -RotFromNorm.y/AxisMagInv), which is still normalized. (super rolled together)
		const T a = 0.5*theta;
		const T s = sin(a);
		w=cos(a); x=0; y=RotFromNorm.z*AxisMagInv*s; z = -RotFromNorm.y*AxisMagInv*s; //angle axis function, reduced

	} //!< Overwrites this quaternion with the calculated rotation that would transform the specified RotateFrom vector to point in the positve X direction. Note: function changes this quaternion.  @param[in] RotateFrom An arbitrary direction vector. Does not need to be normalized.



	const Vec3D<T> RotateVec3D(const Vec3D<T>& f) const { 
		T fx=f.x, fy=f.y, fz=f.z;
		T tw = fx*x + fy*y + fz*z;
		T tx = fx*w - fy*z + fz*y;
		T ty = fx*z + fy*w - fz*x;
		T tz = -fx*y + fy*x + fz*w;
		return Vec3D<T>(w*tx + x*tw + y*tz - z*ty, w*ty - x*tz + y*tw + z*tx, w*tz + x*ty - y*tx + z*tw);
	} //!< Returns a vector representing the specified vector "f" rotated by this quaternion. @param[in] f The vector to transform.

	template <typename U> const Vec3D<U> RotateVec3D(const Vec3D<U>& f) const {
		U fx = (U)(f.x), fy=(U)(f.y), fz=(U)(f.z);
		U tw = (U)(fx*x + fy*y + fz*z);
		U tx = (U)(fx*w - fy*z + fz*y);
		U ty = (U)(fx*z + fy*w - fz*x);
		U tz = (U)(-fx*y + fy*x + fz*w);
		return Vec3D<U>((U)(w*tx + x*tw + y*tz - z*ty), (U)(w*ty - x*tz + y*tw + z*tx), (U)(w*tz + x*ty - y*tx + z*tw));
	} //!< Returns a vector representing the specified vector "f" rotated by this quaternion. Mixed template parameter version. @param[in] f The vector to transform.

	const Vec3D<T> RotateVec3DInv(const Vec3D<T>& f) const { 
		T fx=f.x, fy=f.y, fz=f.z;
		T tw = x*fx + y*fy + z*fz;
		T tx = w*fx - y*fz + z*fy;
		T ty = w*fy + x*fz - z*fx;
		T tz = w*fz - x*fy + y*fx;
		return Vec3D<T>(tw*x + tx*w + ty*z - tz*y, tw*y - tx*z + ty*w + tz*x, tw*z + tx*y - ty*x + tz*w);	
	} //!< Returns a vector representing the specified vector "f" rotated by the inverse of this quaternion. This is the opposite of RotateVec3D. @param[in] f The vector to transform.
	
	template <typename U> const Vec3D<U> RotateVec3DInv(const Vec3D<U>& f) const {
		T fx=f.x, fy=f.y, fz=f.z;
		T tw = x*fx + y*fy + z*fz;
		T tx = w*fx - y*fz + z*fy;
		T ty = w*fy + x*fz - z*fx;
		T tz = w*fz - x*fy + y*fx;
		return Vec3D<U>(tw*x + tx*w + ty*z - tz*y, tw*y - tx*z + ty*w + tz*x, tw*z + tx*y - ty*x + tz*w);	
	} //!< Returns a vector representing the specified vector "f" rotated by the inverse of this quaternion. This is the opposite of RotateVec3D. Mixed template parameter version. @param[in] f The vector to transform.

};

#endif //_QUAT3D_H
