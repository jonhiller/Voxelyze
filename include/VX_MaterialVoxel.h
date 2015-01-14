/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_MATERIALVOXEL_H
#define VX_MATERIALVOXEL_H

#include "VX_Material.h"

//!Defines a voxel type of a specific material.
/*!
The only fundamental addition to CVX_Material is the physical size of the voxel, but this allows precomputing of mass, moments of inertia, and therefore all the dynamic properties of these voxels. 
*/

class CVX_MaterialVoxel : public CVX_Material {
	public:
	CVX_MaterialVoxel(float youngsModulus=1e6f, float density=1e3f, double nominalSize=0.001); //!< Default Constructor. @param[in] youngsModulus The Young's Modulus (stiffness) of this material in Pascals. @param[in] density The density of this material in Kg/m^3. @param[in] nominalSize The nominal voxel size in meters.
	CVX_MaterialVoxel(rapidjson::Value& mat, double nominalSize=0.001); //!< Constructs this CVX_Material object from a rapidjson parser node that contains valid "materials" sub-nodes. @param[in] mat pointer to a rapidjson Value that contains material information. See rapidjson documentation and the *.vxl.json format info in the voxelyze user guide. @param[in] nominalSize The nominal voxel size in meters.
	CVX_MaterialVoxel(const CVX_Material& mat, double nominalSize=0.001); //!< Constructs from an existing material. @param[in] mat Material to construct from. @param[in] nominalSize The nominal voxel size in meters
	//virtual ~CVX_MaterialVoxel(void); //!< Destructor. Virtual so we can just keep track of CVX_Material pointers.
	CVX_MaterialVoxel(const CVX_MaterialVoxel& vIn) {*this = vIn;} //!< Copy constructor
	virtual CVX_MaterialVoxel& operator=(const CVX_MaterialVoxel& vIn); //!< Equals operator

	//size and scaling
	bool setNominalSize(double size); //!< Sets the nominal cubic voxel size in order to calculate mass, moments of inertia, etc of this material. In ordinary circumstances this should be controlled by the overall simulation and never called directly. Use setExternalScaleFactor() if you wish to change the size of voxels of this material. @param[in] size The size of a voxel as measured by its linear outer dimension. (units: meters)
	double nominalSize(){return nomSize;} //!< Returns the nominal cubic voxel size in meters.
	Vec3D<double> size() {return nomSize*extScale;} //!< Returns the current nominal size (in meters) of any voxels of this material-including external scaling factors. The size is calculated according to baseSize()*externalScaleFactor(). This represents the nominal size for voxels of this material, and every instantiated voxel may have a different actual size based on the forces acting upon it in context. This value does not include thermal expansions and contractions which may also change the nominal size of a given voxel depending on its CTE and current temperature.

	//mass and inertia
	float mass(){return _mass;} //!<Returns the mass of a voxel of this material in Kg. Mass cannot be specified directly. Mass is indirectly calculated according to setDensity() and setBaseSize().
	float momentInertia(){return _momentInertia;} //!<Returns the first moment of inertia of a voxel of this material in kg*m^2. This quantity is indirectly calculated according to setDensity() and setBaseSize().

	//damping convenience functions
	float internalDampingTranslateC() const {return zetaInternal*_2xSqMxExS;} //!< Returns the internal material damping coefficient (translation).
	float internalDampingRotateC() const {return zetaInternal*_2xSqIxExSxSxS;} //!< Returns the internal material damping coefficient (rotation).
	float globalDampingTranslateC() const {return zetaGlobal*_2xSqMxExS;} //!< Returns the global material damping coefficient (translation)
	float globalDampingRotateC() const {return zetaGlobal*_2xSqIxExSxSxS;} //!< Returns the global material damping coefficient (rotation)
	float collisionDampingTranslateC() const {return zetaCollision*_2xSqMxExS;} //!< Returns the global material damping coefficient (translation)
	float collisionDampingRotateC() const {return zetaCollision*_2xSqIxExSxSxS;} //!< Returns the global material damping coefficient (rotation)

	//stiffness
	float penetrationStiffness() const {return (float)(2*E*nomSize);} //!< returns the stiffness with which this voxel will resist penetration. This is calculated according to E*A/L with L = voxelSize/2.

protected:
	void initialize(double nominalSize); //!< Initializes this voxel material with the specified voxel size. @param[in] nominalSize The nominal voxel size in meters.
	
	//only the main simulation should update gravity
	void setGravityMultiplier(float gravityMultiplier){gravMult = gravityMultiplier;} //!< Sets the multiple of gravity for this material. In normal circumstances this should only be called by the parent voxelyze object. @param[in] gravityMultiplier Gravity multiplier (1 = 1G gravity).
	float gravityMuliplier() {return gravMult;} //!< Returns the current gravity multiplier.
	float gravityForce() {return -_mass*9.80665f*gravMult;} //!< Returns the current gravitational force on this voxel according to F=ma.

	virtual bool updateAll() {return false;} //!< Updates and recalculates eveything possible (used by inherited classed when material properties have changed)
	virtual bool updateDerived(); //!< Updates all the derived quantities cached as member variables for this and derived classes. (Especially if density, size or elastic modulus changes.)

	double nomSize; //!< Nominal size (i.e. lattice dimension) (m)
	float gravMult; //!< Multiplier for how strongly gravity should affect this block in g (1.0 = -9.81m/s^2)
	float _mass; //!< Cached mass of this voxel (kg)
	float _massInverse; //!< Cached 1/Mass (1/kg)
	float _sqrtMass; //!< Cached sqrt(mass). (sqrt(Kg))
	float _firstMoment; //!< Cached 1st moment "inertia" (needed for certain calculations) (kg*m)
	float _momentInertia; //!< Cached mass moment of inertia (i.e. rotational "mass") (kg*m^2)
	float _momentInertiaInverse; //!< Cached 1/Inertia (1/(kg*m^2))
	float _2xSqMxExS; //!< Cached value needed for quick damping calculations (Kg*m/s)
	float _2xSqIxExSxSxS; //!< Cached value needed for quick rotational damping calculations (Kg*m^2/s)

	friend class CVoxelyze; //give the main simulation class full access
	friend class CVX_Voxel; //give our voxel class direct access to all the members for quick access};
	friend class CVX_MaterialLink;
};



#endif //VX_MATERIALVOXEL_H