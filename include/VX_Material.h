/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_MATERIAL_H
#define VX_MATERIAL_H

//#include "VX_Enums.h"
#include <string>
#include <vector>
#include "Vec3D.h"

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"

//!Defines the properties a raw material 
/*!Contains all information relevant to a physical material to be simulated. All units are SI standard.

A physical material model may be specified as a series of true stress/strain data points, or by simple linear or bilinear parameters.

If a function returns unsuccessfully, check lastError() for the cause.

*/
class CVX_Material {
	public:
	CVX_Material(float youngsModulus=1e6f, float density=1e3f); //!< Default Constructor. @param[in] youngsModulus The Young's Modulus (stiffness) of this material in Pascals. @param[in] density The density of this material in Kg/m^3
	CVX_Material(rapidjson::Value& mat) {readJSON(mat);} //!< Constructs this CVX_Material object from a rapidjson parser node that contains valid "materials" sub-nodes. @param[in] mat pointer to a rapidjson Value that contains material information. See rapidjson documentation and the *.vxl.json format info in the voxelyze user guide.
	virtual ~CVX_Material(void) {}; //!< Destructor. Specified as virtual so we can just keep track of generic material pointers for voxel and link materials.
	CVX_Material(const CVX_Material& vIn) {*this = vIn;} //!< Copy constructor
	virtual CVX_Material& operator=(const CVX_Material& vIn); //!< Equals operator

	void clear(); //!< Resets all material information to default.
	const char* lastError() const {return error.c_str();} //!< Returns the last error encountered for this object.

	void setName(const char* name) {myName = std::string(name);} //!< Adds an optional name to the material. @param[in] name Desired name. 
	const char* name() const {return myName.c_str();} //!< Returns the optional material name if one was specifed. Otherwise returns an empty string.

	float stress(float strain, float transverseStrainSum=0.0f, bool forceLinear = false); //!<returns the stress of the material model accounting for volumetric strain effects. @param [in] strain The strain to query. The resulting stress in this direction will be returned. @param [in] transverseStrainSum The sum of the two principle normal strains in the plane perpendicular to strain. @param [in] forceLinear If true, the result will be calculated according to the elastic modulus of the material regardless of non-linearities in the model.
	float modulus(float strain); //!<returns the modulus (slope of the stress/strain curve) of the material model at the specified strain. @param [in] strain The strain to query.
	bool isYielded(float strain) {return epsilonYield != -1.0f && strain>epsilonYield;} //!< Returns true if the specified strain is past the yield point (if one is specified). @param [in] strain The strain to query.
	bool isFailed(float strain) {return epsilonFail != -1.0f && strain>epsilonFail;} //!< Returns true if the specified strain is past the failure point (if one is specified). @param [in] strain The strain to query.

	//color
	void setColor(int red, int green, int blue, int alpha=255); //!< Sets the material color. Values from [0,255]. @param [in] red Red channel @param [in] green Green channel @param [in] blue Blue channel @param [in] alpha Alpha channel
	void setRed(int red); //!< Sets the red channel of the material color. @param [in] red Red channel [0,255]
	void setGreen(int green); //!< Sets the green channel of the material color. @param [in] green Green channel [0,255]
	void setBlue(int blue); //!< Sets the blue channel of the material color. @param [in] blue Blue channel [0,255]
	void setAlpha(int alpha); //!< Sets the alpha channel of the material color. @param [in] alpha Alpha channel [0,255]
	int red() const {return r;} //!< Returns the red channel of the material color [0,255] or -1 if unspecified.
	int green() const {return g;} //!< Returns the green channel of the material color [0,255] or -1 if unspecified.
	int blue() const {return b;} //!< Returns the blue channel of the material color [0,255] or -1 if unspecified.
	int alpha() const {return a;} //!< Returns the alpha channel of the material color [0,255] or -1 if unspecified.

	//Material Model
	bool setModel(int dataPointCount, float* pStrainValues, float* pStressValues); //!< Defines the physical material behavior with a series of true stress/strain data points. @param [in] dataPointCount The expected number of data points. @param [in] pStrainValues pointer to the first strain value data point in a contiguous array (Units: Pa). @param [in] pStressValues pointer to the first stress value data point in a contiguous array (Units: Pa).
	bool setModelLinear(float youngsModulus, float failureStress=-1); //!< Convenience function to quickly define a linear material. @param [in] youngsModulus Young's modulus (Units: Pa). @param [in] failureStress Optional failure stress (Units: Pa). -1 indicates failure is not an option.
	bool setModelBilinear(float youngsModulus, float plasticModulus, float yieldStress, float failureStress=-1); //!< Convenience function to quickly define a bilinear material. @param [in] youngsModulus Young's modulus (Units: Pa). @param [in] plasticModulus Plastic modulus (Units: Pa). @param [in] yieldStress Yield stress. @param [in] failureStress Optional failure stress (Units: Pa). -1 indicates failure is not an option.
	bool isModelLinear() const {return linear;} //!< Returns true if the material model is a simple linear behavior.
	
	float youngsModulus() const {return E;} //!< Returns Youngs modulus in Pa.
	float yieldStress() const {return sigmaYield;} //!<Returns the yield stress in Pa or -1 if unspecified.
	float failureStress() const {return sigmaFail;} //!<Returns the failure stress in Pa or -1 if unspecified.
	int modelDataPoints() const {return strainData.size();} //!< Returns the number of data points in the current material model data arrays.
	const float* modelDataStrain() const {return &strainData[0];} //!< Returns a pointer to the first strain value data point in a continuous array. The number of values can be determined from modelDataPoints(). The assumed first value of 0 is included.
	const float* modelDataStress() const {return &stressData[0];} //!< Returns a pointer to the first stress value data point in a continuous array. The number of values can be determined from modelDataPoints(). The assumed first value of 0 is included.

	void setPoissonsRatio(float poissonsRatio); //!< Defines Poisson's ratio for the material. @param [in] poissonsRatio Desired Poisson's ratio [0, 0.5).
	float poissonsRatio() const {return nu;} //!< Returns the current Poissons ratio
	float bulkModulus() const {return E/(3*(1-2*nu));} //!< Calculates the bulk modulus from Young's modulus and Poisson's ratio.
	float lamesFirstParameter() const {return (E*nu)/((1+nu)*(1-2*nu));} //!< Calculates Lame's first parameter from Young's modulus and Poisson's ratio.
	float shearModulus() const {return E/(2*(1+nu));} //!< Calculates the shear modulus from Young's modulus and Poisson's ratio.
	bool isXyzIndependent() const {return nu==0.0f;} //!< Returns true if poisson's ratio is zero - i.e. deformations in each dimension are independent of those in other dimensions.

	//other material properties
	void setDensity(float density); //!< Defines the density for the material in Kg/m^3. @param [in] density Desired density (0, INF)
	float density() const {return rho;} //!< Returns the current density.
	void setStaticFriction(float staticFrictionCoefficient); //!< Defines the coefficient of static friction. @param [in] staticFrictionCoefficient Coefficient of static friction [0, INF).
	float staticFriction() const {return muStatic;} //!< Returns the current coefficient of static friction.
	void setKineticFriction(float kineticFrictionCoefficient); //!< Defines the coefficient of kinetic friction. @param [in] kineticFrictionCoefficient Coefficient of kinetc friction [0, INF).
	float kineticFriction() const {return muKinetic;} //!< Returns the current coefficient of kinetic friction.

	//damping
	//http://www.roush.com/Portals/1/Downloads/Articles/Insight.pdf
	void setInternalDamping(float zeta); //!<Defines the internal material damping ratio. The effect is to damp out vibrations within a structure. zeta = mu/2 (mu = loss factor) = 1/(2Q) (Q = amplification factor). High values of zeta may lead to simulation instability. Recommended value: 1.0.  @param [in] zeta damping ratio [0, INF). (unitless)
	float internalDamping() const {return zetaInternal;} //!< Returns the internal material damping ratio.
	void setGlobalDamping(float zeta); //!<Defines the viscous damping of any voxels using this material relative to ground (no motion). Translation C (damping coefficient) is calculated according to zeta*2*sqrt(m*k) where k=E*nomSize. Rotational damping coefficient is similarly calculated High values relative to 1.0 may cause simulation instability. @param [in] zeta damping ratio [0, INF). (unitless)
	float globalDamping() const {return zetaGlobal;} //!< Returns the global material damping ratio.
	void setCollisionDamping(float zeta); //!<Defines the material damping ratio for when this material collides with something. This gives some control over the elasticity of a collision. A value of zero results in a completely elastic collision. @param [in] zeta damping ratio [0, INF). (unitless)
	float collisionDamping() const {return zetaCollision;} //!< Returns the collision material damping ratio.


	//size and scaling
	void setExternalScaleFactor(Vec3D<double> factor); //!< Scales all voxels of this material by a specified factor in each dimension (1.0 is no scaling). This allows enables volumetric displacement-based actuation within a structure. As such, mass is unchanged when the external scale factor changes. Actual size is obtained by multiplying nominal size by the provided factor. @param[in] factor Multiplication factor (0, INF) for the size of all voxels of this material in its local x, y, and z axes. (unitless) 
	void setExternalScaleFactor(double factor) {setExternalScaleFactor(Vec3D<double>(factor, factor, factor));} //!< Convenience function to specify isotropic external scaling factor. See setExternalScaleFactor(Vec3D<> factor). @param[in] factor external scaling factor (0, INF).
	Vec3D<double> externalScaleFactor() {return extScale;} //!< Returns the current external scaling factor (unitless). See description of setExternalScaleFactor().

	//thermal expansion
	void setCte(float cte) {alphaCTE=cte;} //!< Defines the coefficient of thermal expansion. @param [in] cte Desired coefficient of thermal expansion per degree C (-INF, INF)
	float cte() const {return alphaCTE;} //!< Returns the current coefficient of thermal expansion per degree C.

protected:
	std::string error; //!< The last error encountered
	std::string myName; //!< The name of this material. Default is "".
	int r; //!< Red color value of this material from 0-255. Default is -1 (invalid/not set).
	int g; //!< Green color value of this material from 0-255. Default is -1 (invalid/not set).
	int b; //!< Blue color value of this material from 0-255. Default is -1 (invalid/not set).
	int a; //!< Alpha value of this material from 0-255. Default is -1 (invalid/not set).

	//material model
	bool linear; //!< Set to true if this material is specified as linear.
	float E; //!< Young's modulus (stiffness) in Pa.
	float sigmaYield; //!< Yield stress in Pa.
	float sigmaFail; //!< Failure stress in Pa
	float epsilonYield; //!< Yield strain
	float epsilonFail; //!< Failure strain
	std::vector<float> strainData; //!< strain data points
	std::vector<float> stressData; //!< stress data points
	float nu; //!< Poissonss Ratio
	float rho; //!< Density in Kg/m^3
	float alphaCTE; //!< Coefficient of thermal expansion (CTE)
	float muStatic; //!< Static coefficient of friction
	float muKinetic; //!< Kinetic coefficient of friction
	float zetaInternal; //!< Internal damping ratio
	float zetaGlobal; //!< Global damping ratio
	float zetaCollision; //!< Collision damping ratio

	Vec3D<double> extScale; //!< A prescribed scaling factor. default of (1,1,1) is no scaling.

	//derived quantities to cache
	virtual bool updateAll() {return false;} //!< Updates and recalculates eveything possible (used by inherited classed when material properties have changed)
	virtual bool updateDerived(); //!< Updates all the derived quantities cached as member variables for this and derived classes. (Especially if density, size or elastic modulus changes.)
	float _eHat; //!< Cached effective elastic modulus for materials with non-zero Poisson's ratio.


	//Future parameters:
	//piezo?
	//compressive strength? (/compressive data)
	//heat conduction

	bool setYieldFromData(float percentStrainOffset=0.2); //!< Sets sigmaYield and epsilonYield assuming strainData, stressData, E, and failStrain are set correctly.
	float strain(float stress); //!< Returns a simple reverse lookup of the first strain that yields this stress from data point lookup.

	std::vector<CVX_Material*> dependentMaterials; //!< Any materials in this list will have updateDerived() called whenever it's called for this material. For example, in Voxelyze this is used for updatng link materials when one or both voxel materials change

	void writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w); //!< Writes this material's data to the rapidjson writing object.
	bool readJSON(rapidjson::Value& mat); //!< reads this material data from the rapidjson Value.

private:
	friend class CVoxelyze; //give the main simulation class full access
	friend class CVX_Voxel; //give our voxel class direct access to all the members for quick access};
	friend class CVX_Link; 
};

#endif //VX_MATERIAL_H
