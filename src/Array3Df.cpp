/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/


#include "Array3Df.h"
#include <algorithm>

CArray3Df& CArray3Df::operator=(const CArray3Df& rArray) {
	CArray3D::operator=(rArray);
	aspc = rArray.aspc;
	return *this;
}

CArray3Df& CArray3Df::operator=(const CArray3D<float>& rArray)
{
	CArray3D::operator=(rArray);
	aspc = 1.0;
	return *this;
}

bool CArray3Df::writeJSON(rapidjson_Writer& w, float minMagToWrite) const {
	w.StartObject();
	w.Key("spacing");
	w.Double(aspc);

	w.Key("defaultValue");
	w.Double(defaultValue);

	w.Key("size");
	w.StartArray();
	w.Int(aSize.x);
	w.Int(aSize.y);
	w.Int(aSize.z);
	w.EndArray();

	w.Key("offset");
	w.StartArray();
	w.Int(aOff.x);
	w.Int(aOff.y);
	w.Int(aOff.z);
	w.EndArray();

	w.Key("cMin");
	w.StartArray();
	w.Int(cMin.x);
	w.Int(cMin.y);
	w.Int(cMin.z);
	w.EndArray();

	w.Key("cMax");
	w.StartArray();
	w.Int(cMax.x);
	w.Int(cMax.y);
	w.Int(cMax.z);
	w.EndArray();

	w.Key("arrayData");
	w.StartArray();
	int dataSize = (int)data.size();
	for (int i=0; i<dataSize; i++){
		float thisData = data[i];
		if (abs(thisData)<minMagToWrite) w.Double(0.0);
		else w.Double(data[i]);
	}
	w.EndArray();
	w.EndObject();

	return true;
}

bool CArray3Df::readJSON(rapidjson::Value& v){
	clear();

	if (v.HasMember("spacing") && v["spacing"].IsDouble()) aspc = (float) v["spacing"].GetDouble();
	else return false;

	if (v.HasMember("defaultValue") && v["defaultValue"].IsDouble()) defaultValue = (float) v["defaultValue"].GetDouble();
	else return false;

	if (v.HasMember("size") && v["size"].IsArray() && v["size"].Size() == 3){
		if (v["size"][0].IsNumber()) aSize.x=v["size"][0].GetInt();
		if (v["size"][1].IsNumber()) aSize.y=v["size"][1].GetInt();
		if (v["size"][2].IsNumber()) aSize.z=v["size"][2].GetInt();
	}
	else return false;

	if (v.HasMember("offset") && v["offset"].IsArray() && v["offset"].Size() == 3){
		if (v["offset"][0].IsNumber()) aOff.x=v["offset"][0].GetInt();
		if (v["offset"][1].IsNumber()) aOff.y=v["offset"][1].GetInt();
		if (v["offset"][2].IsNumber()) aOff.z=v["offset"][2].GetInt();
	}
	else return false;

	if (v.HasMember("cMin") && v["cMin"].IsArray() && v["cMin"].Size() == 3){
		if (v["cMin"][0].IsNumber()) cMin.x=v["cMin"][0].GetInt();
		if (v["cMin"][1].IsNumber()) cMin.y=v["cMin"][1].GetInt();
		if (v["cMin"][2].IsNumber()) cMin.z=v["cMin"][2].GetInt();
	}
	else return false;

	if (v.HasMember("cMax") && v["cMax"].IsArray() && v["cMax"].Size() == 3){
		if (v["cMax"][0].IsNumber()) cMax.x=v["cMax"][0].GetInt();
		if (v["cMax"][1].IsNumber()) cMax.y=v["cMax"][1].GetInt();
		if (v["cMax"][2].IsNumber()) cMax.z=v["cMax"][2].GetInt();
	}
	else return false;

	if (v.HasMember("arrayData") && v["arrayData"].IsArray()){
		rapidjson::Value& va = v["arrayData"];
		int size = va.Size();
		data.reserve(size);
		for (int i=0; i<size; i++) data.push_back((float)(va[i].GetDouble()));
	}

	return true;
}

Vec3Df CArray3Df::indexToLocation(const Index3D& index) const
{
	return aspc*Vec3Df((float)index.x, (float)index.y, (float)index.z);
}

Index3D CArray3Df::locationToIndex(const Vec3Df& location) const
{
	Vec3Df xformed = (location+0.5*Vec3Df(aspc, aspc, aspc))/aspc;
	return Index3D((int)xformed.x, (int)xformed.y, (int)xformed.z);
}

Vec3Df CArray3Df::locationToContinuousIndex(const Vec3Df& location) const //returns location, in index scale, but without truncating to integer
{
	return location/aspc;
}

Index3D CArray3Df::continuousIndexToIndex(const Vec3Df& cIndex) const
{
	Vec3Df xformed = cIndex + Vec3Df(0.5, 0.5, 0.5);
	return Index3D((int)xformed.x, (int)xformed.y, (int)xformed.z);
}

void CArray3Df::multiplyElements(CArray3Df& multiplyBy)
{
	if (defaultValue != 0.0f) return;

	if (aSize == multiplyBy.size() && aOff == multiplyBy.offset()){ //operate directly on buffer data
		int numEl = (int)data.size();
		for (int i=0; i<numEl; i++) data[i] *= multiplyBy.data[i];
	}
	else {
		for (int k=cMin.x; k<=cMax.x; k++){
			for (int j=cMin.y; j<=cMax.y; j++){
				for (int i=cMin.z; i<=cMax.z; i++){
					at(i,j,k) *= multiplyBy(i,j,k);
				}
			}
		}
	}

}

void CArray3Df::divideElements(CArray3Df& divideBy)
{
	if (defaultValue != 0.0f) return;

	if (aSize == divideBy.size() && aOff == divideBy.offset()){ //operate directly on buffer data
		int numEl = (int)data.size();
		for (int i=0; i<numEl; i++) {
			float divideByValue = divideBy.data[i];
			if (divideByValue == 0) data[i] = 0;
			else data[i] /= divideByValue;
		}
	}
	else {
		for (int k=cMin.x; k<=cMax.x; k++){
			for (int j=cMin.y; j<=cMax.y; j++){
				for (int i=cMin.z; i<=cMax.z; i++){
					float divideByValue = divideBy(i,j,k);
					if (divideByValue == 0) at(i,j,k) = 0;
					else at(i,j,k) /= divideByValue;
				}
			}
		}
	}
}

void CArray3Df::multiplyElements(float multiplyBy)
{
	if (defaultValue != 0.0f) return;

	int numEl = (int)data.size();
	for (int i=0; i<numEl; i++) data[i] *= multiplyBy;
}

void CArray3Df::addElements(float add)
{
	defaultValue += add;

	int numEl = (int)data.size();
	for (int i = 0; i<numEl; i++) data[i] += add;
}

void CArray3Df::sqrtElements()
{
	if (defaultValue != 0.0f) return;
	int numEl = (int)data.size();
	for (int i=0; i<numEl; i++){
		if (data[i] > 0) data[i] = sqrt(data[i]);
		else data[i] = 0;
	}
}

float CArray3Df::maxMagnitude() const
{
	float maxVal = 0;
	int numEl = (int)data.size();
	for (int i=0; i<numEl; i++)
		if (abs(data[i]) > maxVal) maxVal = abs(data[i]);

	return maxVal;
}

//gaussian blur this array with the sepcified sigma. Values outside initialized array will naturally be counted as the default array value (0).
//sigma: distance (in non-dimensional units of array spacing (1.e. "1" = whatever the array spacing is) for the guassian kernel
//extent: number of sigmas to consider values. Large values will be more accurate but increase calculation time dramatically.
void CArray3Df::gaussianBlur(float sigma, float extent){
	//create linear gaussian kernel
	int numOut = (int)ceilf(sigma*extent); //X sigma gaussian filter
	int linSize = 1+2*numOut;
	std::vector<float> linKernel(linSize);

//	float sumKernel = 0;
	for (int i=0; i<linSize; i++){
		linKernel[i] = exp(-((i-numOut)*(i-numOut))/(2*sigma*sigma))/sqrt((2*3.1415926f*sigma*sigma));
//		sumKernel += linKernel[i];
	}
//	for (int i=0; i<linSize; i++)linKernel[i]/=sumKernel; //normalize

	normalizeLinearKernel(&linKernel);
	applyLinearKernel(&linKernel);

	////apply in X, then Y, then Z (that are independent!)
	//CArray3Df arrCopy(*this);
	//Index3D min = offset(), max = min + size();

	////apply in X:
	//for (int ak=min.z; ak<=max.z; ak++){
	//	for (int aj=min.y; aj<=max.y; aj++){
	//		for (int ai=min.x; ai<=max.x; ai++){
	//			float acc = 0;
	//			for (int ti=-numOut; ti<=numOut; ti++){
	//				int thisI = ai+ti;
	//				if (thisI >= min.x && thisI <= max.x){
	//					acc += linKernel[ti+numOut]*arrCopy(thisI, aj, ak);
	//				}
	//			}
	//			addValue(ai, aj, ak, acc, false);
	//		}
	//	}
	//}

	////apply in Y:
	//for (int ak=min.z; ak<=max.z; ak++){
	//	for (int ai=min.x; ai<=max.x; ai++){
	//		for (int aj=min.y; aj<=max.y; aj++){
	//			float acc = 0;
	//			for (int tj=-numOut; tj<=numOut; tj++){
	//				int thisJ = aj+tj;
	//				if (thisJ >= min.y && thisJ <= max.y){
	//					acc += linKernel[tj+numOut]* (*this)(ai, thisJ, ak);
	//				}
	//			}
	//			arrCopy.addValue(ai, aj, ak, acc, false);
	//		}
	//	}
	//}

	////apply in Z:
	//for (int aj=min.y; aj<=max.y; aj++){
	//	for (int ai=min.x; ai<=max.x; ai++){
	//		for (int ak=min.z; ak<=max.z; ak++){
	//			float acc = 0;
	//			for (int tk=-numOut; tk<=numOut; tk++){
	//				int thisK = ak+tk;
	//				if (thisK >= min.z && thisK <= max.z){
	//					acc += linKernel[tk+numOut]*arrCopy(ai, aj, thisK);
	//				}
	//			}
	//			addValue(ai, aj, ak, acc, false);
	//		}
	//	}
	//}
}

//blur with a linear falloff. Unfortunately not linearly independent like gaussian.
//radius is in voxel units (i.e. 1 = arraySpacing in real units).
void CArray3Df::linearBlur(float radius)
{
//	Index3D min = offset(), max = min + size()-Index3D(1,1,1);
	Index3D min = minAllocated(), max = maxAllocated();

	float fRad = radius; //filtering radius in voxels
	int fRadI = (int)(1+fRad); //number of voxels to search up and down.

	CArray3Df arrCopy(*this);

	for (int k=min.z; k<=max.z; k++){
		for (int j=min.y; j<=max.y; j++){
			for (int i=min.x; i<=max.x; i++){
				float sum = 0;
				float newValue = 0;
				for (int l = std::max(min.x, i-fRadI); l <= std::min(max.x, i+fRadI); l++){
					for (int m = std::max(min.y, j-fRadI); m <= std::min(max.y, j+fRadI); m++){
						for (int n = std::max(min.z, k-fRadI); n <= std::min(max.z, k+fRadI); n++){

							float fac = std::max(0.0f, fRad-(float)sqrt((float)(i-l)*(i-l)+(j-m)*(j-m)+(k-n)*(k-n))); //linear drop-off of weight (fac) from rmin @ centered vox to 0 at fRad and above.
							
					//		if (fac > fRad /2) fac = fRad;
					//		else fac = 0;

							newValue += fac*arrCopy(l,m,n);
							sum += fac;

						}
					}
				}

				addValue(i,j,k,newValue/sum, false);
			}
		}
	}
}

void CArray3Df::stepBlur(float radius)
{
	//	Index3D min = offset(), max = min + size()-Index3D(1,1,1);
	Index3D min = minAllocated(), max = maxAllocated();

	float fRad = radius; //filtering radius in voxels
	int fRadI = (int)(1 + fRad); //number of voxels to search up and down.

	CArray3Df arrCopy(*this);

	for (int k = min.z; k <= max.z; k++) {
		for (int j = min.y; j <= max.y; j++) {
			for (int i = min.x; i <= max.x; i++) {
				float sum = 0;
				float newValue = 0;
				for (int l = std::max(min.x, i - fRadI); l <= std::min(max.x, i + fRadI); l++) {
					for (int m = std::max(min.y, j - fRadI); m <= std::min(max.y, j + fRadI); m++) {
						for (int n = std::max(min.z, k - fRadI); n <= std::min(max.z, k + fRadI); n++) {

							float rad = (float)sqrt((float)(i - l)*(i - l) + (j - m)*(j - m) + (k - n)*(k - n)); 
							float fac = rad <= radius ? 1.0 : 0.0;

							newValue += fac*arrCopy(l, m, n);
							sum += fac;

						}
					}
				}

				addValue(i, j, k, newValue / sum, false);
			}
		}
	}
}


//sets each existing element of this array (so, after setting spacing and resizing the array) by trilinearly interpolating the same location (accounting for spacing) in sampleFrom
void CArray3Df::sampleFromArray(CArray3Df* sampleFrom)
{
	if (sampleFrom->size() == size() && sampleFrom->offset() == offset() && sampleFrom->spacing() == spacing()){
		*this = *sampleFrom; //if equal, just copy over.
	}
	else {
		erase(); //ensures min and max are accurate as we add values

		Index3D min = minAllocated(), max = maxAllocated();
		for (int k=min.z; k<=max.z; k++){
			for (int j=min.y; j<=max.y; j++){
				for (int i=min.x; i<=max.x; i++){
					Vec3Df thisLocation = indexToLocation(Index3D(i,j,k));
					Vec3Df thatContinuousIndex = sampleFrom->locationToContinuousIndex(thisLocation);
					float interpDens = sampleFrom->interpolateTriLinear(thatContinuousIndex);

					addValue(i,j,k,interpDens,false);
				}
			}
		}
	}
}

//normalizes a linear kernel such that the sum of the values = 1.0
void CArray3Df::normalizeLinearKernel(std::vector<float>* kernel)
{
	float sumKernel = 0;
	int kSize = (int)kernel->size();
	for (int i=0; i<kSize; i++) sumKernel += (*kernel)[i]; //sum
	for (int i=0; i<kSize; i++) (*kernel)[i]/=sumKernel; //normalize
}

//applies the linear kernel to this 3D array in X, then Y, then Z. (Order shouldn't matter for a linear kernel though)
void CArray3Df::applyLinearKernel(std::vector<float>* kernel)
{
	CArray3Df arrCopy(*this);
	Index3D min = offset(), max = min + size()-Index3D(1,1,1);
	int numOut = (int)(kernel->size()-1)/2;

	//apply in X:
	for (int ak=min.z; ak<=max.z; ak++){
		for (int aj=min.y; aj<=max.y; aj++){
			for (int ai=min.x; ai<=max.x; ai++){
				float acc = 0, div = 0;
				for (int ti=-numOut; ti<=numOut; ti++){
					int thisI = ai+ti;
					if (thisI >= min.x && thisI <= max.x){
						float kern = (*kernel)[ti + numOut];
						div += kern;
						acc += kern*arrCopy(thisI, aj, ak);
					}
				}
				addValue(ai, aj, ak, acc/div, false);
			}
		}
	}

	//apply in Y:
	for (int ak=min.z; ak<=max.z; ak++){
		for (int ai=min.x; ai<=max.x; ai++){
			for (int aj=min.y; aj<=max.y; aj++){
				float acc = 0, div = 0;
				for (int tj=-numOut; tj<=numOut; tj++){
					int thisJ = aj+tj;
					if (thisJ >= min.y && thisJ <= max.y){
						float kern = (*kernel)[tj + numOut];
						div += kern;
						acc += kern * (*this)(ai, thisJ, ak);
					}
				}
				arrCopy.addValue(ai, aj, ak, acc/div, false);
			}
		}
	}

	//apply in Z:
	for (int aj=min.y; aj<=max.y; aj++){
		for (int ai=min.x; ai<=max.x; ai++){
			for (int ak=min.z; ak<=max.z; ak++){
				float acc = 0, div = 0;
				for (int tk=-numOut; tk<=numOut; tk++){
					int thisK = ak+tk;
					if (thisK >= min.z && thisK <= max.z){
						float kern = (*kernel)[tk + numOut];
						div += kern;
						acc += kern * arrCopy(ai, aj, thisK);
					}
				}
				addValue(ai, aj, ak, acc/div, false);
			}
		}
	}
}


//simplest of finite difference methods.
Vec3Df CArray3Df::arrayGradient(const Index3D& index) const
{
	return Vec3Df(
		(at(index-Index3D(1,0,0)) - at(index+Index3D(1,0,0)))/(2*aspc),
		(at(index-Index3D(0,1,0)) - at(index+Index3D(0,1,0)))/(2*aspc),
		(at(index-Index3D(0,0,1)) - at(index+Index3D(0,0,1)))/(2*aspc));
}

Vec3Df CArray3Df::arrayGradientInterp(const Vec3Df& cIndex, float delta, interpolateType type) const
{
	return Vec3Df(
		(interpolate(cIndex - Vec3Df(delta, 0, 0), type) - interpolate(cIndex + Vec3Df(delta, 0, 0), type)) / (2 * delta * aspc),
		(interpolate(cIndex - Vec3Df(0, delta, 0), type) - interpolate(cIndex + Vec3Df(0, delta, 0), type)) / (2 * delta * aspc),
		(interpolate(cIndex - Vec3Df(0, 0, delta), type) - interpolate(cIndex + Vec3Df(0, 0, delta), type)) / (2 * delta * aspc));
}

void CArray3Df::oversample(int oSample, interpolateType type)
{
	CArray3Df tmp(*this);
	oversample(tmp, oSample, type);
}

//erases this array and resizes it to contain the desired oversample of the arra3D "in".
//oSample is the multiple to infill. range anything above 2, although beware the oSample^3 size increase in number of elements
// the final number of elements will be (x-1)*oSample + 1 for each dimension.
//does not modify "in".
void CArray3Df::oversample(CArray3Df& in, int oSample, interpolateType type)
{
	if (oSample == 1){ *this = in; return;}
	else if (oSample < 1) return;

	//supersample!
	erase();
	resize((in.size()-Index3D(1,1,1))*oSample+Index3D(1,1,1), in.offset()*oSample);

	Index3D min = in.offset(), max = in.offset() + in.size() - Index3D(1,1,1);
	Index3D /*osMin = offset(),*/ osMax = offset() + size() - Index3D(1,1,1);

	for (int k=min.z; k<=max.z; k++){
		for (int j=min.y; j<=max.y; j++){
			for (int i=min.x; i<=max.x; i++){

				//for the oversample:
				for (int k2=0; k2<oSample; k2++){
					for (int j2=0; j2<oSample; j2++){
						for (int i2=0; i2<oSample; i2++){
							int curI = i*oSample+i2;
							int curJ = j*oSample+j2;
							int curK = k*oSample+k2;

							if (curI > osMax.x || curJ > osMax.y || curK > osMax.z) continue;

							//interpolate homogenization
							float xp = (float)i2/(float)oSample;
							float yp = (float)j2/(float)oSample;
							float zp = (float)k2/(float)oSample;

							float interpVal = in.interpolate(Vec3Df(i+xp, j+yp, k+zp), type);


							//if (interpVal > 0.5)
							//	int stop = 0;

							//switch (type){
							//	case TRILINEAR: interpVal = in.interpolateTriLinear(Vec3Df(i+xp, j+yp, k+zp)); break;
							//	case TRICUBIC: interpVal = in.interpolateTriCubic(Vec3Df(i+xp, j+yp, k+zp)); break;
							//	case AVG_TRILINEAR: interpVal = in.interpolateTriLinearAvg(Vec3Df(i+xp, j+yp, k+zp)); break;
							//}

							addValue(curI, curJ, curK, interpVal, false);

						}
					}
				}
			}
		}
	}

	aspc=in.spacing()/oSample;
//	for (int i=1; i<oSample; i++) aspc /= 2;
}


//trilinear interpolation

float CArray3Df::interpolate(const Vec3Df& interpIndex, interpolateType type) const
{
	switch (type){
		case TRILINEAR: return interpolateTriLinear(interpIndex);
		case TRICUBIC: return interpolateTriCubic(interpIndex);
		case AVG_TRILINEAR: return interpolateTriLinearAvg(interpIndex);
		default: return interpolateTriLinear(interpIndex);
	}
}


float CArray3Df::interpolateTriLinear(const Vec3Df& interpIndex) const
{
	int i = floor(interpIndex.x), j = floor(interpIndex.y), k = floor(interpIndex.z);
	float xp = interpIndex.x - i, yp = interpIndex.y - j, zp = interpIndex.z - k;

	float val = 	(*this)(i,j,k) *		(1-xp) *	(1-yp) *	(1-zp)	+
					(*this)(i+1,j,k) *		(xp) *		(1-yp) *	(1-zp)	+
					(*this)(i,j+1,k) *		(1-xp) *	(yp) *		(1-zp)	+
					(*this)(i,j,k+1) *		(1-xp) *	(1-yp) *	(zp)	+
					(*this)(i+1,j,k+1) *	(xp) *		(1-yp) *	(zp)	+
					(*this)(i,j+1,k+1) *	(1-xp) *	(yp) *		(zp)	+
					(*this)(i+1,j+1,k) *	(xp) *		(yp) *		(1-zp)	+
					(*this)(i+1,j+1,k+1) *	(xp) *		(yp) *		(zp)	;
	return val;
}

float CArray3Df::interpolateTriLinearAvg(const Vec3Df& interpIndex) const
{
	float avgSum = 0;
	float eps = 0.5f;
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(eps, 0, 0));
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(-eps, 0, 0));
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(0, eps, 0));
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(0, -eps, 0));
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(0, 0, eps));
	avgSum += interpolateTriLinear(interpIndex+Vec3Df(0, 0, -eps));
	return avgSum / 6.0f;
}

float CArray3Df::interpolateTriCubic(const Vec3Df& interpIndex) const
{
	int xi = floor(interpIndex.x), yi = floor(interpIndex.y), zi = floor(interpIndex.z);
	float dx = interpIndex.x - xi, dy = interpIndex.y - yi, dz = interpIndex.z - zi;

		float x[64] = {
			// values of f(x,y,z) at each corner.
			at(xi,yi,zi),at(xi+1,yi,zi),at(xi,yi+1,zi),
			at(xi+1,yi+1,zi),at(xi,yi,zi+1),at(xi+1,yi,zi+1),
			at(xi,yi+1,zi+1),at(xi+1,yi+1,zi+1),
			// values of df/dx at each corner.
			0.5f*(at(xi+1,yi,zi)-at(xi-1,yi,zi)),
			0.5f*(at(xi+2,yi,zi)-at(xi,yi,zi)),
			0.5f*(at(xi+1,yi+1,zi)-at(xi-1,yi+1,zi)),
			0.5f*(at(xi+2,yi+1,zi)-at(xi,yi+1,zi)),
			0.5f*(at(xi+1,yi,zi+1)-at(xi-1,yi,zi+1)),
			0.5f*(at(xi+2,yi,zi+1)-at(xi,yi,zi+1)),
			0.5f*(at(xi+1,yi+1,zi+1)-at(xi-1,yi+1,zi+1)),
			0.5f*(at(xi+2,yi+1,zi+1)-at(xi,yi+1,zi+1)),
			// values of df/dy at each corner.
			0.5f*(at(xi,yi+1,zi)-at(xi,yi-1,zi)),
			0.5f*(at(xi+1,yi+1,zi)-at(xi+1,yi-1,zi)),
			0.5f*(at(xi,yi+2,zi)-at(xi,yi,zi)),
			0.5f*(at(xi+1,yi+2,zi)-at(xi+1,yi,zi)),
			0.5f*(at(xi,yi+1,zi+1)-at(xi,yi-1,zi+1)),
			0.5f*(at(xi+1,yi+1,zi+1)-at(xi+1,yi-1,zi+1)),
			0.5f*(at(xi,yi+2,zi+1)-at(xi,yi,zi+1)),
			0.5f*(at(xi+1,yi+2,zi+1)-at(xi+1,yi,zi+1)),
			// values of df/dz at each corner.
			0.5f*(at(xi,yi,zi+1)-at(xi,yi,zi-1)),
			0.5f*(at(xi+1,yi,zi+1)-at(xi+1,yi,zi-1)),
			0.5f*(at(xi,yi+1,zi+1)-at(xi,yi+1,zi-1)),
			0.5f*(at(xi+1,yi+1,zi+1)-at(xi+1,yi+1,zi-1)),
			0.5f*(at(xi,yi,zi+2)-at(xi,yi,zi)),
			0.5f*(at(xi+1,yi,zi+2)-at(xi+1,yi,zi)),
			0.5f*(at(xi,yi+1,zi+2)-at(xi,yi+1,zi)),
			0.5f*(at(xi+1,yi+1,zi+2)-at(xi+1,yi+1,zi)),
			// values of d2f/dxdy at each corner.
			0.25f*(at(xi+1,yi+1,zi)-at(xi-1,yi+1,zi)-at(xi+1,yi-1,zi)+at(xi-1,yi-1,zi)),
			0.25f*(at(xi+2,yi+1,zi)-at(xi,yi+1,zi)-at(xi+2,yi-1,zi)+at(xi,yi-1,zi)),
			0.25f*(at(xi+1,yi+2,zi)-at(xi-1,yi+2,zi)-at(xi+1,yi,zi)+at(xi-1,yi,zi)),
			0.25f*(at(xi+2,yi+2,zi)-at(xi,yi+2,zi)-at(xi+2,yi,zi)+at(xi,yi,zi)),
			0.25f*(at(xi+1,yi+1,zi+1)-at(xi-1,yi+1,zi+1)-at(xi+1,yi-1,zi+1)+at(xi-1,yi-1,zi+1)),
			0.25f*(at(xi+2,yi+1,zi+1)-at(xi,yi+1,zi+1)-at(xi+2,yi-1,zi+1)+at(xi,yi-1,zi+1)),
			0.25f*(at(xi+1,yi+2,zi+1)-at(xi-1,yi+2,zi+1)-at(xi+1,yi,zi+1)+at(xi-1,yi,zi+1)),
			0.25f*(at(xi+2,yi+2,zi+1)-at(xi,yi+2,zi+1)-at(xi+2,yi,zi+1)+at(xi,yi,zi+1)),
			// values of d2f/dxdz at each corner.
			0.25f*(at(xi+1,yi,zi+1)-at(xi-1,yi,zi+1)-at(xi+1,yi,zi-1)+at(xi-1,yi,zi-1)),
			0.25f*(at(xi+2,yi,zi+1)-at(xi,yi,zi+1)-at(xi+2,yi,zi-1)+at(xi,yi,zi-1)),
			0.25f*(at(xi+1,yi+1,zi+1)-at(xi-1,yi+1,zi+1)-at(xi+1,yi+1,zi-1)+at(xi-1,yi+1,zi-1)),
			0.25f*(at(xi+2,yi+1,zi+1)-at(xi,yi+1,zi+1)-at(xi+2,yi+1,zi-1)+at(xi,yi+1,zi-1)),
			0.25f*(at(xi+1,yi,zi+2)-at(xi-1,yi,zi+2)-at(xi+1,yi,zi)+at(xi-1,yi,zi)),
			0.25f*(at(xi+2,yi,zi+2)-at(xi,yi,zi+2)-at(xi+2,yi,zi)+at(xi,yi,zi)),
			0.25f*(at(xi+1,yi+1,zi+2)-at(xi-1,yi+1,zi+2)-at(xi+1,yi+1,zi)+at(xi-1,yi+1,zi)),
			0.25f*(at(xi+2,yi+1,zi+2)-at(xi,yi+1,zi+2)-at(xi+2,yi+1,zi)+at(xi,yi+1,zi)),
			// values of d2f/dydz at each corner.
			0.25f*(at(xi,yi+1,zi+1)-at(xi,yi-1,zi+1)-at(xi,yi+1,zi-1)+at(xi,yi-1,zi-1)),
			0.25f*(at(xi+1,yi+1,zi+1)-at(xi+1,yi-1,zi+1)-at(xi+1,yi+1,zi-1)+at(xi+1,yi-1,zi-1)),
			0.25f*(at(xi,yi+2,zi+1)-at(xi,yi,zi+1)-at(xi,yi+2,zi-1)+at(xi,yi,zi-1)),
			0.25f*(at(xi+1,yi+2,zi+1)-at(xi+1,yi,zi+1)-at(xi+1,yi+2,zi-1)+at(xi+1,yi,zi-1)),
			0.25f*(at(xi,yi+1,zi+2)-at(xi,yi-1,zi+2)-at(xi,yi+1,zi)+at(xi,yi-1,zi)),
			0.25f*(at(xi+1,yi+1,zi+2)-at(xi+1,yi-1,zi+2)-at(xi+1,yi+1,zi)+at(xi+1,yi-1,zi)),
			0.25f*(at(xi,yi+2,zi+2)-at(xi,yi,zi+2)-at(xi,yi+2,zi)+at(xi,yi,zi)),
			0.25f*(at(xi+1,yi+2,zi+2)-at(xi+1,yi,zi+2)-at(xi+1,yi+2,zi)+at(xi+1,yi,zi)),
			// values of d3f/dxdydz at each corner.
			0.125f*(at(xi+1,yi+1,zi+1)-at(xi-1,yi+1,zi+1)-at(xi+1,yi-1,zi+1)+at(xi-1,yi-1,zi+1)-at(xi+1,yi+1,zi-1)+at(xi-1,yi+1,zi-1)+at(xi+1,yi-1,zi-1)-at(xi-1,yi-1,zi-1)),
			0.125f*(at(xi+2,yi+1,zi+1)-at(xi,yi+1,zi+1)-at(xi+2,yi-1,zi+1)+at(xi,yi-1,zi+1)-at(xi+2,yi+1,zi-1)+at(xi,yi+1,zi-1)+at(xi+2,yi-1,zi-1)-at(xi,yi-1,zi-1)),
			0.125f*(at(xi+1,yi+2,zi+1)-at(xi-1,yi+2,zi+1)-at(xi+1,yi,zi+1)+at(xi-1,yi,zi+1)-at(xi+1,yi+2,zi-1)+at(xi-1,yi+2,zi-1)+at(xi+1,yi,zi-1)-at(xi-1,yi,zi-1)),
			0.125f*(at(xi+2,yi+2,zi+1)-at(xi,yi+2,zi+1)-at(xi+2,yi,zi+1)+at(xi,yi,zi+1)-at(xi+2,yi+2,zi-1)+at(xi,yi+2,zi-1)+at(xi+2,yi,zi-1)-at(xi,yi,zi-1)),
			0.125f*(at(xi+1,yi+1,zi+2)-at(xi-1,yi+1,zi+2)-at(xi+1,yi-1,zi+2)+at(xi-1,yi-1,zi+2)-at(xi+1,yi+1,zi)+at(xi-1,yi+1,zi)+at(xi+1,yi-1,zi)-at(xi-1,yi-1,zi)),
			0.125f*(at(xi+2,yi+1,zi+2)-at(xi,yi+1,zi+2)-at(xi+2,yi-1,zi+2)+at(xi,yi-1,zi+2)-at(xi+2,yi+1,zi)+at(xi,yi+1,zi)+at(xi+2,yi-1,zi)-at(xi,yi-1,zi)),
			0.125f*(at(xi+1,yi+2,zi+2)-at(xi-1,yi+2,zi+2)-at(xi+1,yi,zi+2)+at(xi-1,yi,zi+2)-at(xi+1,yi+2,zi)+at(xi-1,yi+2,zi)+at(xi+1,yi,zi)-at(xi-1,yi,zi)),
			0.125f*(at(xi+2,yi+2,zi+2)-at(xi,yi+2,zi+2)-at(xi+2,yi,zi+2)+at(xi,yi,zi+2)-at(xi+2,yi+2,zi)+at(xi,yi+2,zi)+at(xi+2,yi,zi)-at(xi,yi,zi))
		};

		float coefs[64]; //should be cached per voxel...

		// Convert voxel values and partial derivatives to interpolation coefficients.
    	for (int i=0;i<64;++i) {
    		coefs[i] = 0.0;
    		for (int j=0;j<64;++j) {
    			coefs[i] += _C[i][j]*x[j];
    		}
    	}
		// Remember this voxel for next time.
		//_i1 = xi;
		//_i2 = yi;
		//_i3 = zi;
		//_initialized = true;


	// Evaluate the interpolation within this grid voxel.
//    dx -= xi;
//    dy -= yi;
//    dz -= zi;
	int ijkn(0);
	double dzpow(1);
	double result(0);
	for(int k = 0; k < 4; ++k) {
		double dypow(1);
		for(int j = 0; j < 4; ++j) {
			result += dypow*dzpow*
				(coefs[ijkn] + dx*(coefs[ijkn+1] + dx*(coefs[ijkn+2] + dx*coefs[ijkn+3])));
			ijkn += 4;
			dypow *= dy;
		}
		dzpow *= dz;
	}
	return result;
}

const int CArray3Df::_C[64][64] = {
	{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-3, 3, 0, 0, 0, 0, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 2,-2, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 9,-9,-9, 9, 0, 0, 0, 0, 6, 3,-6,-3, 0, 0, 0, 0, 6,-6, 3,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-6, 6, 6,-6, 0, 0, 0, 0,-3,-3, 3, 3, 0, 0, 0, 0,-4, 4,-2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-2,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-6, 6, 6,-6, 0, 0, 0, 0,-4,-2, 4, 2, 0, 0, 0, 0,-3, 3,-3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-1,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 4,-4,-4, 4, 0, 0, 0, 0, 2, 2,-2,-2, 0, 0, 0, 0, 2,-2, 2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0, 0, 0, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9,-9,-9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 3,-6,-3, 0, 0, 0, 0, 6,-6, 3,-3, 0, 0, 0, 0, 4, 2, 2, 1, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-6, 6, 6,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3,-3, 3, 3, 0, 0, 0, 0,-4, 4,-2, 2, 0, 0, 0, 0,-2,-2,-1,-1, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-6, 6, 6,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4,-2, 4, 2, 0, 0, 0, 0,-3, 3,-3, 3, 0, 0, 0, 0,-2,-1,-2,-1, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4,-4,-4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2,-2,-2, 0, 0, 0, 0, 2,-2, 2,-2, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
	{-3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0, 0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0, 0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 9,-9, 0, 0,-9, 9, 0, 0, 6, 3, 0, 0,-6,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6,-6, 0, 0, 3,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-6, 6, 0, 0, 6,-6, 0, 0,-3,-3, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4, 4, 0, 0,-2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-2, 0, 0,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0, 0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0, 0, 0,-1, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9,-9, 0, 0,-9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 3, 0, 0,-6,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6,-6, 0, 0, 3,-3, 0, 0, 4, 2, 0, 0, 2, 1, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-6, 6, 0, 0, 6,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3,-3, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4, 4, 0, 0,-2, 2, 0, 0,-2,-2, 0, 0,-1,-1, 0, 0},
	{ 9, 0,-9, 0,-9, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 3, 0,-6, 0,-3, 0, 6, 0,-6, 0, 3, 0,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 2, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 9, 0,-9, 0,-9, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 3, 0,-6, 0,-3, 0, 6, 0,-6, 0, 3, 0,-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 2, 0, 2, 0, 1, 0},
	{-27,27,27,-27,27,-27,-27,27,-18,-9,18, 9,18, 9,-18,-9,-18,18,-9, 9,18,-18, 9,-9,-18,18,18,-18,-9, 9, 9,-9,-12,-6,-6,-3,12, 6, 6, 3,-12,-6,12, 6,-6,-3, 6, 3,-12,12,-6, 6,-6, 6,-3, 3,-8,-4,-4,-2,-4,-2,-2,-1},
	{18,-18,-18,18,-18,18,18,-18, 9, 9,-9,-9,-9,-9, 9, 9,12,-12, 6,-6,-12,12,-6, 6,12,-12,-12,12, 6,-6,-6, 6, 6, 6, 3, 3,-6,-6,-3,-3, 6, 6,-6,-6, 3, 3,-3,-3, 8,-8, 4,-4, 4,-4, 2,-2, 4, 4, 2, 2, 2, 2, 1, 1},
	{-6, 0, 6, 0, 6, 0,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0,-3, 0, 3, 0, 3, 0,-4, 0, 4, 0,-2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-2, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0,-6, 0, 6, 0, 6, 0,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 0,-3, 0, 3, 0, 3, 0,-4, 0, 4, 0,-2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-2, 0,-1, 0,-1, 0},
	{18,-18,-18,18,-18,18,18,-18,12, 6,-12,-6,-12,-6,12, 6, 9,-9, 9,-9,-9, 9,-9, 9,12,-12,-12,12, 6,-6,-6, 6, 6, 3, 6, 3,-6,-3,-6,-3, 8, 4,-8,-4, 4, 2,-4,-2, 6,-6, 6,-6, 3,-3, 3,-3, 4, 2, 4, 2, 2, 1, 2, 1},
	{-12,12,12,-12,12,-12,-12,12,-6,-6, 6, 6, 6, 6,-6,-6,-6, 6,-6, 6, 6,-6, 6,-6,-8, 8, 8,-8,-4, 4, 4,-4,-3,-3,-3,-3, 3, 3, 3, 3,-4,-4, 4, 4,-2,-2, 2, 2,-4, 4,-4, 4,-2, 2,-2, 2,-2,-2,-2,-2,-1,-1,-1,-1},
	{ 2, 0, 0, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{-6, 6, 0, 0, 6,-6, 0, 0,-4,-2, 0, 0, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0,-3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2,-1, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 4,-4, 0, 0,-4, 4, 0, 0, 2, 2, 0, 0,-2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-6, 6, 0, 0, 6,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4,-2, 0, 0, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0,-3, 3, 0, 0,-2,-1, 0, 0,-2,-1, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4,-4, 0, 0,-4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0,-2,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 2,-2, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
	{-6, 0, 6, 0, 6, 0,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4, 0,-2, 0, 4, 0, 2, 0,-3, 0, 3, 0,-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0,-2, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0,-6, 0, 6, 0, 6, 0,-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-4, 0,-2, 0, 4, 0, 2, 0,-3, 0, 3, 0,-3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,-1, 0,-2, 0,-1, 0},
	{18,-18,-18,18,-18,18,18,-18,12, 6,-12,-6,-12,-6,12, 6,12,-12, 6,-6,-12,12,-6, 6, 9,-9,-9, 9, 9,-9,-9, 9, 8, 4, 4, 2,-8,-4,-4,-2, 6, 3,-6,-3, 6, 3,-6,-3, 6,-6, 3,-3, 6,-6, 3,-3, 4, 2, 2, 1, 4, 2, 2, 1},
	{-12,12,12,-12,12,-12,-12,12,-6,-6, 6, 6, 6, 6,-6,-6,-8, 8,-4, 4, 8,-8, 4,-4,-6, 6, 6,-6,-6, 6, 6,-6,-4,-4,-2,-2, 4, 4, 2, 2,-3,-3, 3, 3,-3,-3, 3, 3,-4, 4,-2, 2,-4, 4,-2, 2,-2,-2,-1,-1,-2,-2,-1,-1},
	{ 4, 0,-4, 0,-4, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0,-2, 0,-2, 0, 2, 0,-2, 0, 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 0, 0, 0, 0, 0, 0, 0, 4, 0,-4, 0,-4, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0,-2, 0,-2, 0, 2, 0,-2, 0, 2, 0,-2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0},
	{-12,12,12,-12,12,-12,-12,12,-8,-4, 8, 4, 8, 4,-8,-4,-6, 6,-6, 6, 6,-6, 6,-6,-6, 6, 6,-6,-6, 6, 6,-6,-4,-2,-4,-2, 4, 2, 4, 2,-4,-2, 4, 2,-4,-2, 4, 2,-3, 3,-3, 3,-3, 3,-3, 3,-2,-1,-2,-1,-2,-1,-2,-1},
	{ 8,-8,-8, 8,-8, 8, 8,-8, 4, 4,-4,-4,-4,-4, 4, 4, 4,-4, 4,-4,-4, 4,-4, 4, 4,-4,-4, 4, 4,-4,-4, 4, 2, 2, 2, 2,-2,-2,-2,-2, 2, 2,-2,-2, 2, 2,-2,-2, 2,-2, 2,-2, 2,-2, 2,-2, 1, 1, 1, 1, 1, 1, 1, 1}
};
