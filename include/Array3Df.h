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
	CArray3Df(const Index3D& size, const Index3D& offset = Index3D()) : CArray3D(){aspc=1.0f; resize(size, offset);}
	CArray3Df& operator=(const CArray3Df& rArray); //!Operator "=" overload
	CArray3Df& operator=(const CArray3D<float>& rArray); //!Operator "=" overload

	bool writeJSON(rapidjson_Writer& w, float minMagToWrite = 0);
	bool readJSON(rapidjson::Value& v);

	void multiplyElements(CArray3Df& multiplyBy); //element-wise scaling. Only applied if default array value is zero.
	void divideElements(CArray3Df& divideBy); //element-wise inverse scaling (i.e. dividing be each element of divideByBy. divideByBy Values of zero will result in zero, not INF. Only applied if default array value is zero.
	void multiplyElements(float multiplyBy); //scale all elements equally. Only applied if default array value is zero.
	void sqrtElements(); //square roots all elements. Negative values end up zero. Only applied if default array value is zero.

	float maxMagnitude(); //returns the maximum magnitude (positive or negative) in this array

	//a value representing the space between array indices (in some sort of real unit, so scale indices by)
	void setSpacing(float arraySpacing){aspc=arraySpacing;}
	float spacing(){return aspc;}

	Vec3Df indexToLocation(const Index3D& index);
	Index3D locationToIndex(Vec3Df& location); //returns nearest (normal rounding)
	Vec3Df locationToContinuousIndex(Vec3Df& location); //returns location, in index scale, but without truncating to integer

	void gaussianBlur(float sigma = 1.0f, float extent = 3.0f);
	void linearBlur(float radius = 1.0f);

	void sampleFromArray(CArray3Df* sampleFrom); //sets each existing element of this array (so, after setting spacing and resizing the array) by trilinearly interpolating the same location (accounting for spacing) in sampleFrom

	Vec3Df arrayGradient(Index3D index);
	void oversample(int oSample, interpolateType type = TRILINEAR); //oversample self
	void oversample(CArray3Df& in, int oSample, interpolateType type = TRILINEAR);
	float interpolate(Vec3Df interpIndex, interpolateType type);
	float interpolateTriLinear(Vec3Df interpIndex);
	float interpolateTriLinearAvg(Vec3Df interpIndex);
	float interpolateTriCubic(Vec3Df interpIndex);

private:
	void normalizeLinearKernel(std::vector<float>* kernel);
	void applyLinearKernel(std::vector<float>* kernel);
	static const int _C[64][64];
	float aspc;
	};
#endif //ARRAY3DF_H
