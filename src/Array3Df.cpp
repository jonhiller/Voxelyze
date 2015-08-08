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

bool CArray3Df::writeJSON(rapidjson_Writer& w){
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
	for (int i=0; i<dataSize; i++) w.Double(data[i]);
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

Vec3Df CArray3Df::indexToLocation(Index3D& index)
{
	return aspc*Vec3Df((float)index.x, (float)index.y, (float)index.z);
}

Index3D CArray3Df::locationToIndex(Vec3Df& location)
{
	Vec3Df xformed = (location+0.5*Vec3Df(aspc, aspc, aspc))/aspc;
	return Index3D((int)xformed.x, (int)xformed.y, (int)xformed.z);
}

Vec3Df CArray3Df::locationToContinuousIndex(Vec3Df& location) //returns location, in index scale, but without truncating to integer
{
	return location/aspc;
}

//gaussian blur this array with the sepcified sigma. Values outside initialized array will naturally be counted as the default array value (0).
//sigma: distance (in non-dimensional units of array spacing (1.e. "1" = whatever the array spacing is) for the guassian kernel
//extent: number of sigmas to consider values. Large values will be more accurate but increase calculation time dramatically.
void CArray3Df::gaussianBlur(float sigma, float extent){
	//create linear gaussian kernel
	int numOut = (int)ceilf(sigma*extent); //X sigma gaussian filter
	int linSize = 1+2*numOut;
	std::vector<float> linKernel(linSize);

	float sumKernel = 0;
	for (int i=0; i<linSize; i++){
		linKernel[i] = exp(-((i-numOut)*(i-numOut))/(2*sigma*sigma))/sqrt((2*3.1415926f*sigma*sigma));
		sumKernel += linKernel[i];
	}
	for (int i=0; i<linSize; i++)linKernel[i]/=sumKernel; //normalize

	//apply in X, then Y, then Z (that are independent!)
	CArray3Df arrCopy(*this);
	Index3D min = offset(), max = min + size();

	//apply in X:
	for (int ak=min.z; ak<=max.z; ak++){
		for (int aj=min.y; aj<=max.y; aj++){
			for (int ai=min.x; ai<=max.x; ai++){
				float acc = 0;
				for (int ti=-numOut; ti<=numOut; ti++){
					int thisI = ai+ti;
					if (thisI >= min.x && thisI <= max.x){
						acc += linKernel[ti+numOut]*arrCopy(thisI, aj, ak);
					}
				}
				addValue(ai, aj, ak, acc);
			}
		}
	}

//		arrCopy = *this;
	//"this" has the latest values
	//apply in Y:
	for (int ak=min.z; ak<=max.z; ak++){
		for (int ai=min.x; ai<=max.x; ai++){
			for (int aj=min.y; aj<=max.y; aj++){
				float acc = 0;
				for (int tj=-numOut; tj<=numOut; tj++){
					int thisJ = aj+tj;
					if (thisJ >= min.y && thisJ <= max.y){
						acc += linKernel[tj+numOut]* (*this)(ai, thisJ, ak);
					}
				}
				arrCopy.addValue(ai, aj, ak, acc);
			}
		}
	}

	//arrCopy = arr;
	//apply in Z:
	for (int aj=min.y; aj<=max.y; aj++){
		for (int ai=min.x; ai<=max.x; ai++){
			for (int ak=min.z; ak<=max.z; ak++){
				float acc = 0;
				for (int tk=-numOut; tk<=numOut; tk++){
					int thisK = ak+tk;
					if (thisK >= min.z && thisK <= max.z){
						acc += linKernel[tk+numOut]*arrCopy(ai, aj, thisK);
					}
				}
				addValue(ai, aj, ak, acc);
			}
		}
	}
}

//simplest of finite difference methods. Provide a gridSpacing value (in real units between grid point) for a quantitaive gradient.
Vec3Df CArray3Df::arrayGradient(Index3D index)
{
	return Vec3Df(
		(at(index-Index3D(1,0,0)) - at(index+Index3D(1,0,0)))/(2*aspc),
		(at(index-Index3D(0,1,0)) - at(index+Index3D(0,1,0)))/(2*aspc),
		(at(index-Index3D(0,0,1)) - at(index+Index3D(0,0,1)))/(2*aspc));
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

							addValue(curI, curJ, curK, interpVal);

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

float CArray3Df::interpolate(Vec3Df interpIndex, interpolateType type)
{
	switch (type){
		case TRILINEAR: return interpolateTriLinear(interpIndex);
		case TRICUBIC: return interpolateTriCubic(interpIndex);
		case AVG_TRILINEAR: return interpolateTriLinearAvg(interpIndex);
		default: return interpolateTriLinear(interpIndex);
	}
}


float CArray3Df::interpolateTriLinear(Vec3D<float> interpIndex)
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

float CArray3Df::interpolateTriLinearAvg(Vec3D<float> interpIndex)
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

float CArray3Df::interpolateTriCubic(Vec3D<float> interpIndex)
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