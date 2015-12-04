/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef ARRAY3DF_H
#define ARRAY3DF_H

#include "Array3D.h"
#include "Vec3D.h"
#include "json.h"

class CArray3Df : public CArray3D<float>
{
public:

	enum interpolateType {
		TRILINEAR,
		TRICUBIC,
		AVG_TRILINEAR //looks +/- 1/2 grid size in all three dimensions and averages the 6 bilinear interpolations. Has a smoothing effect.
	};

	CArray3Df() : CArray3D() {aspc=1.0f;}
	CArray3Df(const CArray3Df& rArray){*this = rArray;}
	CArray3Df(const Index3D& size, const Index3D& offset = Index3D(0,0,0)) : CArray3D() { aspc = 1.0f; resize(size, offset); }
	CArray3Df(Index3D& min, Index3D& max, const float spacing, const float defaultValue = 0.0f) : CArray3D() { setDefaultValue(defaultValue); aspc = spacing; resizeToMinMax(min, max); }
	CArray3Df& operator=(const CArray3Df& rArray); //!Operator "=" overload
	CArray3Df& operator=(const CArray3D<float>& rArray); //!Operator "=" overload

	bool writeJSON(rapidjson_Writer& w, float minMagToWrite = 0) const;
	bool readJSON(rapidjson::Value& v);

	void multiplyElements(CArray3Df& multiplyBy); //element-wise scaling. Only applied if default array value is zero.
	void divideElements(CArray3Df& divideBy); //element-wise inverse scaling (i.e. dividing be each element of divideByBy. divideByBy Values of zero will result in zero, not INF. Only applied if default array value is zero.
	void multiplyElements(float multiplyBy); //scale all elements equally. Only applied if default array value is zero.
	void addElements(float add); //add to all elements equally. Changes the default value, too.
	void sqrtElements(); //square roots all elements. Negative values end up zero. Only applied if default array value is zero.

	float maxMagnitude() const; //returns the maximum magnitude (positive or negative) in this array

	//a value representing the space between array indices (in some sort of real unit, so scale indices by)
	void setSpacing(float arraySpacing){aspc=arraySpacing;}
	float spacing() const {return aspc;}

	Vec3Df indexToLocation(const Index3D& index) const;
	Index3D locationToIndex(const Vec3Df& location) const; //returns nearest (normal rounding)
	Vec3Df locationToContinuousIndex(const Vec3Df& location) const; //returns location, in index scale, but without truncating to integer
	Vec3Df indexToContinuousIndex(const Index3D& index) const { return Vec3Df(index.x, index.y, index.z); }
	Index3D continuousIndexToIndex(const Vec3Df& cIndex) const;

	void gaussianBlur(float sigma = 1.0f, float extent = 3.0f);
	void linearBlur(float radius = 1.0f);
	void stepBlur(float radius = 1.0f);

	void sampleFromArray(CArray3Df* sampleFrom); //sets each existing element of this array (so, after setting spacing and resizing the array) by trilinearly interpolating the same location (accounting for spacing) in sampleFrom

	Vec3Df arrayGradient(const Index3D& index) const;
	Vec3Df arrayGradientInterp(const Vec3Df& cIndex, float delta, interpolateType type = TRILINEAR) const; //delta in voxel units. i.e. 0.5 = 1/2 aspc 

	void oversample(int oSample, interpolateType type = TRILINEAR); //oversample self
	void oversample(CArray3Df& in, int oSample, interpolateType type = TRILINEAR);
	float interpolate(const Vec3Df& interpIndex, interpolateType type) const;
	float interpolateTriLinear(const Vec3Df& interpIndex) const;
	float interpolateTriLinearAvg(const Vec3Df& interpIndex) const;
	float interpolateTriCubic(const Vec3Df& interpIndex) const;

private:
	void normalizeLinearKernel(std::vector<float>* kernel);
	void applyLinearKernel(std::vector<float>* kernel);
	static const int _C[64][64];
	float aspc;
	};
#endif //ARRAY3DF_H
