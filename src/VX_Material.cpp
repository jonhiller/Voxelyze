/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_Material.h"
#include <assert.h>

CVX_Material::CVX_Material(float youngsModulus, float density)
{
	clear();
	rho = density;
	setModelLinear(youngsModulus);
	updateDerived();

}

CVX_Material& CVX_Material::operator=(const CVX_Material& vIn)
{
	error = vIn.error;
	myName = vIn.myName;
	r = vIn.r;
	g = vIn.g;
	b = vIn.b;
	a = vIn.a;
	linear = vIn.linear;
	E = vIn.E;
	sigmaYield = vIn.sigmaYield;
	sigmaFail = vIn.sigmaFail;
	epsilonYield = vIn.epsilonYield;
	epsilonFail = vIn.epsilonFail;
	strainData = vIn.strainData;
	stressData = vIn.stressData;
	nu = vIn.nu;
	rho = vIn.rho;
	alphaCTE = vIn.alphaCTE;
	muStatic = vIn.muStatic;
	muKinetic = vIn.muKinetic;
	zetaInternal = vIn.zetaInternal;
	zetaGlobal = vIn.zetaGlobal;
	zetaCollision = vIn.zetaCollision;

	_eHat = vIn._eHat;

	return *this;
}

void CVX_Material::clear()
{
	r = -1;
	g = -1;
	b = -1;
	a = -1;
	nu = 0.0f;
	rho = 1.0f;
	alphaCTE = 0.0f;
	muStatic = 0.0f;
	muKinetic = 0.0f;
	zetaInternal = 1.0f;
	zetaGlobal = 0.0f;
	zetaCollision = 0.0f;

	extScale=Vec3D<>(1.0, 1.0, 1.0);

	setModelLinear(1.0);
	updateDerived();
}

void CVX_Material::writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w)
{
	//rapidjson::PrettyWriter<rapidjson::StringBuffer>* pW = (rapidjson::PrettyWriter<rapidjson::StringBuffer>*)writer;
	w.StartObject();

	if (linear){
		w.Key("youngsModulus");	w.Double((double)E);
		if (epsilonFail != -1){ w.Key("epsilonFail");	w.Double((double)epsilonFail);}
	}
	else {
		w.Key("strainData");
		w.StartArray();
		for (int i=0; i<(int)strainData.size(); i++) w.Double((double)strainData[i]);
		w.EndArray();

		w.Key("stressData");
		w.StartArray();
		for (int i=0; i<(int)stressData.size(); i++) w.Double((double)stressData[i]);
		w.EndArray();

	}


	if (rho != 1.0f){			w.Key("density");			w.Double(rho);}
	if (myName != ""){			w.Key("name");				w.String(myName.c_str());}
	if (r != -1){				w.Key("red");				w.Int(r);}
	if (g != -1){				w.Key("green");				w.Int(g);}
	if (b != -1){				w.Key("blue");				w.Int(b);}
	if (a != -1){				w.Key("alpha");				w.Int(a);}
	if (nu != 0){				w.Key("poissonsRatio");		w.Double(nu);}
	if (alphaCTE != 0){			w.Key("CTE");				w.Double(alphaCTE);}
	if (muStatic != 0){			w.Key("staticFriction");	w.Double(muStatic);}
	if (muKinetic != 0){		w.Key("kineticFriction");	w.Double(muKinetic);}
	if (zetaInternal != 1){		w.Key("internalDamping");	w.Double(zetaInternal);}
	if (zetaGlobal != 0){		w.Key("globalDamping");		w.Double(zetaGlobal);}
	if (zetaCollision != 1){	w.Key("collisionDamping");	w.Double(zetaCollision);}
	if (extScale.x != 1 || extScale.y != 1 || extScale.z != 1){
		w.Key("externalScaleFactor");
		w.StartArray();
		for (int i=0; i<3; i++) w.Double(extScale[i]);
		w.EndArray();
	}

	w.EndObject();

}

bool CVX_Material::readJSON(rapidjson::Value& m)
{
	clear();
	if (m.HasMember("youngsModulus") && m["youngsModulus"].IsDouble()){
		float failStress = -1.0f;
		if (m.HasMember("epsilonFail") && m["epsilonFail"].IsDouble()){
			failStress = m["epsilonFail"].GetDouble()*m["youngsModulus"].GetDouble();
		}
		setModelLinear(m["youngsModulus"].GetDouble(), failStress);
	}
	else if (m.HasMember("strainData") && m["strainData"].IsArray() && m.HasMember("stressData") && m["stressData"].IsArray() && m["strainData"].Size()==m["stressData"].Size()){
		std::vector<float> stress, strain;
		int dataCount = m["strainData"].Size();
		for (int i=0; i<dataCount; i++){
			stress.push_back((float)m["stressData"].GetDouble());
			strain.push_back((float)m["strainData"].GetDouble());
		}
		setModel(dataCount, &strain[0], &stress[0]); 
	}
	else return false; //no valid model

	if (m.HasMember("density") && m["density"].IsDouble())						rho = m["density"].GetDouble();
	if (m.HasMember("name") && m["name"].IsString())							myName = std::string(m["name"].GetString());
	if (m.HasMember("red") && m["red"].IsInt())									r = m["red"].GetInt();
	if (m.HasMember("green") && m["green"].IsInt())								g = m["green"].GetInt();
	if (m.HasMember("blue") && m["blue"].IsInt())								b = m["blue"].GetInt();
	if (m.HasMember("alpha") && m["alpha"].IsInt())								a = m["alpha"].GetInt();
	if (m.HasMember("poissonsRatio") && m["poissonsRatio"].IsDouble())			nu = m["poissonsRatio"].GetDouble();
	if (m.HasMember("CTE") && m["CTE"].IsDouble())								alphaCTE = m["CTE"].GetDouble();
	if (m.HasMember("staticFriction") && m["staticFriction"].IsDouble())		muStatic = m["staticFriction"].GetDouble();
	if (m.HasMember("kineticFriction") && m["kineticFriction"].IsDouble())		muKinetic = m["kineticFriction"].GetDouble();
	if (m.HasMember("internalDamping") && m["internalDamping"].IsDouble())		zetaInternal = m["internalDamping"].GetDouble();
	if (m.HasMember("globalDamping") && m["globalDamping"].IsDouble())			zetaGlobal = m["globalDamping"].GetDouble();
	if (m.HasMember("collisionDamping") && m["collisionDamping"].IsDouble())	zetaCollision = m["collisionDamping"].GetDouble();
	if (m.HasMember("externalScaleFactor") && m["externalScaleFactor"].IsArray() && m["externalScaleFactor"].Size()==3){
		for (int i=0; i<3; i++) extScale[i] = m["externalScaleFactor"][i].GetDouble();
	}

	updateDerived();

	return true;
}

float CVX_Material::stress(float strain, float transverseStrainSum, bool forceLinear)
{
	//reference: http://www.colorado.edu/engineering/CAS/courses.d/Structures.d/IAST.Lect05.d/IAST.Lect05.pdf page 10
	if (isFailed(strain)) return 0.0f; //if a failure point is set and exceeded, we've broken!
	
	if (strain <= strainData[1] || linear || forceLinear){ //for compression/first segment and linear materials (forced or otherwise), simple calculation
		if (nu==0.0f) return E*strain;
		else return _eHat*((1-nu)*strain + nu*transverseStrainSum); 
//		else return eHat()*((1-nu)*strain + nu*transverseStrainSum); 
	}

	//the non-linear feature with non-zero poissons ratio is currently experimental
	int DataCount = modelDataPoints();
	for (int i=2; i<DataCount; i++){ //go through each segment in the material model (skipping the first segment because it has already been handled.
		if (strain <= strainData[i] || i==DataCount-1){ //if in the segment ending with this point (or if this is the last point extrapolate out) 
			float Perc = (strain-strainData[i-1])/(strainData[i]-strainData[i-1]);
			float basicStress = stressData[i-1] + Perc*(stressData[i]-stressData[i-1]);
			if (nu==0.0f) return basicStress;
			else { //accounting for volumetric effects
				float modulus = (stressData[i]-stressData[i-1])/(strainData[i]-strainData[i-1]);
				float modulusHat = modulus/((1-2*nu)*(1+nu));
				float effectiveStrain = basicStress/modulus; //this is the strain at which a simple linear stress strain line would hit this point at the definied modulus
				float effectiveTransverseStrainSum = transverseStrainSum*(effectiveStrain/strain);
				return modulusHat*((1-nu)*effectiveStrain + nu*effectiveTransverseStrainSum);
			}
		}
	}

	assert(false); //should never reach this point
	return 0.0f;
}

float CVX_Material::strain(float stress)
{
	if (stress <= stressData[1] || linear) return stress/E; //for compression/first segment and linear materials (forced or otherwise), simple calculation

	int DataCount = modelDataPoints();
	for (int i=2; i<DataCount; i++){ //go through each segment in the material model (skipping the first segment because it has already been handled.
		if (stress <= stressData[i] || i==DataCount-1){ //if in the segment ending with this point (or if this is the last point extrapolate out) 
			float Perc = (stress-stressData[i-1])/(stressData[i]-stressData[i-1]);
			return strainData[i-1] + Perc*(strainData[i]-strainData[i-1]);
		}
	}

	assert(false); //should never reach this point
	return 0.0f;
}


float CVX_Material::modulus(float strain)
{
	if (isFailed(strain)) return 0.0f; //if a failure point is set and exceeded, we've broken!
	if (strain <= strainData[1] || linear) return E; //for compression/first segment and linear materials, simple calculation

	int DataCount = modelDataPoints();
	for (int i=2; i<DataCount; i++){ //go through each segment in the material model (skipping the first segment because it has already been handled.
		if (strain <= strainData[i] || i==DataCount-1) return (stressData[i]-stressData[i-1])/(strainData[i]-strainData[i-1]); //if in the segment ending with this point
	}
	assert(false); //we should never reach this point in the function
	return 0.0f;

}

void CVX_Material::setColor(int red, int green, int blue, int alpha)
{
	setRed(red);
	setGreen(green);
	setBlue(blue);
	setAlpha(alpha);
}

void CVX_Material::setRed(int red)
{
	if (red>255) red=255;
	if (red<0) red=0;
	r = red;
}

void CVX_Material::setGreen(int green)
{
	if (green>255) green=255;
	if (green<0) green=0;
	g = green;
}

void CVX_Material::setBlue(int blue)
{
	if (blue>255) blue=255;
	if (blue<0) blue=0;
	b = blue;
}

void CVX_Material::setAlpha(int alpha)
{
	if (alpha>255) alpha=255;
	if (alpha<0) alpha=0;
	a = alpha;
}

/*! The arrays are assumed to be of equal length.
The first data point is assumed to be [0,0] and need not be provided.
At least 1 non-zero data point must be provided.
The inital segment from [0,0] to the first strain and stress value is interpreted as young's modulus.
The slope of the stress/strain curve should never exceed this value in subsequent segments.
The last data point is assumed to represent failure of the material. The 0.2% offset method is used to calculate the yield point.

Restrictions on pStrainValues:
	- The values must be positive and increasing in order.
	- Strains are defined in absolute numbers according to delta l / L.

Restrictions on pStressValues:
	- The values must be positive and increasing in order.

Special cases:
	- 1 data point (linear): Yield and failure are assumed to occur simultaneously at the single data point.
	- 2 data points (bilinear): Yield is taken as the first data point, failure at the second.

*/
bool CVX_Material::setModel(int dataPointCount, float* pStrainValues, float* pStressValues)
{
	//Pre-checks
	if (*pStrainValues==0 && *pStressValues==0) { //if first data point is 0,0, ignore it
		pStrainValues++; //advance the pointers...
		pStressValues++;
		dataPointCount--; //decrement the count
	}
	if (dataPointCount<=0){
		error = "Not enough data points";
		return false;
	}
	if (*pStrainValues<=0 || *pStressValues<=0){
		error = "First stress and strain data points negative or zero";
		return false; 
	}

	//Copy the data into something more usable (and check for monotonically increasing)
	std::vector<float> tmpStrainData;
	std::vector<float> tmpStressData;
	tmpStrainData.push_back(0); //add in the zero data point (required always)
	tmpStressData.push_back(0);
	float sweepStrain = 0.0f, sweepStress = 0.0f;
	for (int i=0; i<dataPointCount; i++){
		float thisStrain = *(pStrainValues+i); //grab the values
		float thisStress = *(pStressValues+i);

		if (thisStrain <= sweepStrain){
			error = "Out of order strain data";
			return false;
		}

		if (thisStress <= sweepStress){
			error = "Stress data is not monotonically increasing";
		}

		if (i>0 && (thisStress-sweepStress)/(thisStrain-sweepStrain) > tmpStressData[0]/tmpStrainData[0]){
			error = "Slope of stress/strain curve should never exceed that of the first line segment (youngs modulus)";
			return false;
		}

		sweepStrain = thisStrain;
		sweepStress = thisStress;

		tmpStrainData.push_back(thisStrain); //add to the temporary vector
		tmpStressData.push_back(thisStress);
	}

	assert((int)tmpStrainData.size() == dataPointCount+1 && (int)tmpStressData.size() == dataPointCount+1); //sizes match up? (+1 to include the zero point)

	//at this point, we know we have valid data and will return true
	strainData = tmpStrainData;
	stressData = tmpStressData;
	E=stressData[1]/strainData[1]; //youngs modulus is the inital slope
	sigmaFail = stressData[stressData.size()-1]; //failure stress is the highest stress data point
	epsilonFail = strainData[strainData.size()-1]; //failure strain is the highest strain data point
	linear = (dataPointCount==1);

	if (dataPointCount == 1 || dataPointCount == 2){ //linear or bilinear
		sigmaYield = stressData[1];
		epsilonYield = strainData[1];
	}
	else { //.2% (0.002) strain offset to find a good yield point...
		setYieldFromData();
	}

	return updateDerived();
}

/*! Specified Young's modulus and failure stress must both be positive.
Yield stress is interpreted as identical to failure stress. If failure stress is not specified an arbitrary data point consistent with the specified Young's modulus is added to the model.
*/
bool CVX_Material::setModelLinear(float youngsModulus, float failureStress)
{
	if (youngsModulus<=0){
		error = "Young's modulus must be positive";
		return false;
	}
	if (failureStress != -1.0f && failureStress<=0){
		error = "Failure stress must be positive";
		return false;
	}

	float tmpfailureStress = failureStress; //create a dummy failure stress if none was provided
	if (tmpfailureStress == -1) tmpfailureStress = 1000000;
	float tmpfailStrain = tmpfailureStress/youngsModulus;

	strainData.clear();
	stressData.clear();
	strainData.push_back(0); //add in the zero data point (required always)
	stressData.push_back(0);
	strainData.push_back(tmpfailStrain);
	stressData.push_back(tmpfailureStress);

	linear=true;
	E=youngsModulus;
	sigmaYield = failureStress; //yield and failure are one in the same here.
	sigmaFail = failureStress;
	epsilonYield = (failureStress == -1) ? -1 : tmpfailStrain;
	epsilonFail = (failureStress == -1) ? -1 : tmpfailStrain;
	return updateDerived();
}

/*! Specified Young's modulus, plastic modulus, yield stress, and failure stress must all be positive.
Plastic modulus must be less than Young's modulus and failure stress must be greater than the yield stress.
*/
bool CVX_Material::setModelBilinear(float youngsModulus, float plasticModulus, float yieldStress, float failureStress)
{
	if (youngsModulus<=0){
		error = "Young's modulus must be positive";
		return false;
	}
	if (plasticModulus<=0 || plasticModulus>=youngsModulus){
		error = "Plastic modulus must be positive but less than Young's modulus";
		return false;
	}
	if (yieldStress<=0){
		error = "Yield stress must be positive";
		return false;
	}
	if (failureStress != -1.0f && failureStress <= yieldStress){
		error = "Failure stress must be positive and greater than the yield stress";
		return false;
	}

	float yieldStrain = yieldStress / youngsModulus;
	float tmpfailureStress = failureStress; //create a dummy failure stress if none was provided
	if (tmpfailureStress == -1) tmpfailureStress = 3*yieldStress;

	float tM = plasticModulus;
	float tB = yieldStress-tM*yieldStrain; //y-mx=b
	float tmpfailStrain = (tmpfailureStress-tB)/tM; // (y-b)/m = x

	strainData.clear();
	strainData.push_back(0); //add in the zero data point (required always)
	strainData.push_back(yieldStrain);
	strainData.push_back(tmpfailStrain);

	stressData.clear();
	stressData.push_back(0);
	stressData.push_back(yieldStress);
	stressData.push_back(tmpfailureStress);

	linear = false;
	E=youngsModulus;
	sigmaYield = yieldStress;
	sigmaFail = failureStress;
	epsilonYield = yieldStrain;
	epsilonFail = failureStress == -1.0f ? -1.0f : tmpfailStrain;
	return updateDerived();

}


bool CVX_Material::setYieldFromData(float percentStrainOffset)
{
	sigmaYield = -1.0f; //assume we fail until we succeed.
	epsilonYield = -1.0f; //assume we fail until we succeed.

	float oM = E; //the offset line slope (y=Mx+B)
	float oB = (-percentStrainOffset/100*oM); //offset line intercept (100 factor turns percent into absolute

	assert(strainData.size() == stressData.size());
	assert(strainData.size() > 2); // more than 2 data points (more than bilinear)
	int dataPoints = strainData.size()-1;
	for (int i=1; i<dataPoints-1; i++){
		float x1=strainData[i];
		float x2=strainData[i+1];
		float y1=stressData[i];
		float y2=stressData[i+1];

		float tM = (y2-y1)/(x2-x1); //temporary slope
		float tB = y1-tM*x1; //temporary intercept

		if (oM!=tM){ //if not parallel lines...
			float xIntersect = (tB-oB)/(oM-tM);
			if (xIntersect>x1 && xIntersect<x2){ //if intersects at this segment...
				float percentBetweenPoints = (xIntersect-x1)/(x2-x1);
				sigmaYield = y1+percentBetweenPoints*(y2-y1);
				epsilonYield = xIntersect;
				return true;
			}
		}
	}
	sigmaYield = sigmaFail;
	epsilonYield = epsilonFail;
	return false;
}

void CVX_Material::setPoissonsRatio(float poissonsRatio)
{
	if (poissonsRatio < 0) poissonsRatio = 0;
	if (poissonsRatio >= 0.5 ) poissonsRatio = 0.5-FLT_EPSILON*2; //exactly 0.5 will still cause problems, but it can get very close.
	nu = poissonsRatio;
	updateDerived();
}

void CVX_Material::setDensity(float density)
{
	if (density <= 0) density = FLT_MIN; //density of exactly 0 will cause problems, but can get as close as desired.
	rho = density;
	updateDerived();
}

void CVX_Material::setStaticFriction(float staticFrictionCoefficient)
{
	if (staticFrictionCoefficient <= 0) staticFrictionCoefficient = 0;
	muStatic = staticFrictionCoefficient;
}

void CVX_Material::setKineticFriction(float kineticFrictionCoefficient)
{
	if (kineticFrictionCoefficient <= 0) kineticFrictionCoefficient = 0;
	muKinetic = kineticFrictionCoefficient;
}

void CVX_Material::setInternalDamping(float zeta)
{
	if (zeta <= 0) zeta = 0;
	zetaInternal = zeta;
}

void CVX_Material::setGlobalDamping(float zeta)
{
	if (zeta <= 0) zeta = 0;
	zetaGlobal = zeta;
}

void CVX_Material::setCollisionDamping(float zeta)
{
	if (zeta <= 0) zeta = 0;
	zetaCollision = zeta;
}

void CVX_Material::setExternalScaleFactor(Vec3D<double> factor)
{
	if (factor.x <= 0) factor.x = FLT_MIN;
	if (factor.y <= 0) factor.y = FLT_MIN;
	if (factor.z <= 0) factor.z = FLT_MIN;
	extScale = factor;
}

bool CVX_Material::updateDerived() 
{
	_eHat = E/((1-2*nu)*(1+nu));

	for (std::vector<CVX_Material*>::iterator it = dependentMaterials.begin(); it != dependentMaterials.end(); it++) (*it)->updateAll(); //update material properties of any that depend on this...

	return true;
}
