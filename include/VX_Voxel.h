/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_VOXEL_H
#define VX_VOXEL_H

#include "Vec3D.h"
#include "VX_Link.h"
#include "VX_External.h"
#include "VX_MaterialVoxel.h" //needed for inline of some "get" functions
#include "VX_Collision.h"
#include <list>


//!Defines a specific instance of a voxel and holds its current state.
/*!The voxel class contains all information about a voxel's physical characteristics, state, and CVX_Link's to other adjacent voxels.

A voxel needs a CVX_MaterialVoxel to be created and define its physical properties. Later, these can be accessed by material().

A voxel can have external forced or prescribed displacements applied by accessing its CVX_External object using the external() function. To save memory allocation the CVX_External object is not created until the first time external() is called. Use externalExists() to determine if this voxel has any externals applied.

The state of the voxel includes it position(), size(), orientation(), velocity(), etc.

A voxel has a local coordinate system (LCS) that always stays centered on the center of the voxel and oriented with the axes of the cube. The global coordinate system (GCS) is just that - the global coordinate system.

*/
class CVX_Voxel
{
public:
	//! Defines the direction of a link relative to a given voxel.
	enum linkDirection {	
		X_POS=0,			//!< Positive X direction
		X_NEG=1,			//!< Negative X direction
		Y_POS=2,			//!< Positive Y direction
		Y_NEG=3,			//!< Negative Y direction
		Z_POS=4,			//!< Positive Z direction
		Z_NEG=5				//!< Negative Z direction
	}; 
	//! Defines each of 8 corners of a voxel.
	enum voxelCorner {
		NNN = 0, //0b000
		NNP = 1, //0b001
		NPN = 2, //0b010
		NPP = 3, //0b011
		PNN = 4, //0b100
		PNP = 5, //0b101
		PPN = 6, //0b110
		PPP = 7  //0b111
	}; 

	CVX_Voxel(CVX_MaterialVoxel* material, short indexX, short indexY, short indexZ); //!< Default constuctor. @param [in] material Links this CVX_Material to define the physical properties for this voxel. @param[in] indexX The global X index of this voxel. @param[in] indexY The global Y index of this voxel. @param[in] indexZ The global Z index of this voxel.
	~CVX_Voxel(); //!< Destructor
	void reset(); //!< Resets this voxels to its original position, orientation, temperature, etc. and zeros its momentum. Does not affect any externals.

	CVX_Link* link(linkDirection direction) const {return links[direction];} //!< Returns a pointer to the link object in the specified direction if it exists. Returns null if a link does not exist in this direction.
	int linkCount() const {int retVal =0; for (int i=0; i<6; i++) if (links[i]) retVal++; return retVal;} //!< Returns the number of links present for this voxel out of a total 6 possible.
	CVX_Voxel* adjacentVoxel(linkDirection direction) const; //!< Returns a pointer to the voxel in the specified direction if one exists, or NULL otherwise. @param[in] direction Positive or negative X, Y, or Z direction according to the linkDirection enum.
	short indexX() {return ix;} //!< Returns the global X index of this voxel.
	short indexY() {return iy;} //!< Returns the global Y index of this voxel.
	short indexZ() {return iz;} //!< Returns the global Z index of this voxel.

	CVX_MaterialVoxel* material() {return mat;} //!<Returns the linked material object containing the physical properties of this voxel.
	
	bool externalExists() {return ext?true:false;} //!< Returns true if this voxel has had its CVX_External object created. This does not mecessarily imply that this external object actually contains any fixes or forces.
	CVX_External* external() {if (!ext) ext = new CVX_External(); return ext;} //!< Returns a pointer to this voxel's unique external object that contains fixes, forces, and/or displacements. Allocates a new empty one if it doesn't already exist. Use externalExists() to determine if external() has been previously called at any time.


	void timeStep(float dt); //!< Advances this voxel's state according to all forces and moments acting on it. Large timesteps will cause instability. Use CVoxelyze::recommendedTimeStep() to get the recommended largest stable timestep. @param[in] dt Timestep (in second) to advance.

	//physical location
	Vec3D<double> position() const {return pos;} //!< Returns the center position of this voxel in meters (GCS). This is the origin of the local coordinate system (LCS).
	Vec3D<double> originalPosition() const {double s=mat->nominalSize(); return Vec3D<double>(ix*s, iy*s, iz*s);} //!< Returns the initial (nominal) position of this voxel.
	Vec3D<double> displacement() const {return (pos - originalPosition());} //!< Returns the 3D displacement of this voxel from its original location in meters (GCS)/
	Vec3D<float> size() const {return cornerOffset(PPP)-cornerOffset(NNN);} //!< Returns the current deformed size of this voxel in the local voxel coordinates system (LCS). If asymmetric forces are acting on this voxel, the voxel may not be centered on position(). Use cornerNegative() and cornerPositive() to determine this information.
	Vec3D<float> cornerPosition(voxelCorner corner) const; //!< Returns the deformed location of the voxel corner in the specified corner in the global coordinate system (GCS). Essentially cornerOffset() with the voxel's current global position/rotation applied.
	Vec3D<float> cornerOffset(voxelCorner corner) const; //!< Returns the deformed location of the voxel corner in the specified corner in the local voxel coordinate system (LCS). Used to draw the deformed voxel in the correct position relative to the position().
	bool isInterior() const {return (boolStates & SURFACE)?true:false;} //!< Returns true if the voxel is surrounded by other voxels on its 6 coordinate faces. Returns false if 1 or more faces are exposed.
	bool isSurface() const {return !isInterior();} //!< Convenience function to enhance code readibility. The inverse of isInterior(). Returns true 1 or more faces are exposed. Returns false if the voxel is surrounded by other voxels on its 6 coordinate faces.

	Vec3D<double> baseSize() const {return mat->size()*(1+temp*mat->alphaCTE);} //!<Returns the nominal size of this voxel (LCS) accounting for any specified temperature and external actuation. Specifically, returns the zero-stress size of the voxel if all forces/moments were removed.
	double baseSize(CVX_Link::linkAxis axis) const {return mat->size()[axis]*(1+temp*mat->alphaCTE);} //!<Returns the nominal size of this voxel in the specified axis accounting for any specified temperature and external actuation. Specifically, returns the zero-stress dimension of the voxel if all forces/moments were removed.
	double baseSizeAverage() const {Vec3D<double> bSize=baseSize(); return (bSize.x+bSize.y+bSize.z)/3.0f;} //!<Returns the average nominal size of the voxel in a zero-stress (no force) state. (X+Y+Z/3)

	Quat3D<double> orientation() const {return orient;} //!< Returns the orientation of this voxel in quaternion form (GCS). This orientation defines the relative orientation of the local coordinate system (LCS). The unit quaternion represents the original orientation of this voxel.
	float orientationAngle() const {return (float)orient.Angle();} //!< Use with orientationAxis() to get the orientation of this voxel in angle/axis form. Returns the angle in radians.
	Vec3D<double> orientationAxis() const {return orient.Axis();} //!< Use with orientationAngle() to get the orientation of this voxel in angle/axis form. Returns a unit vector in the global coordinate system (GCS).

	float displacementMagnitude() const {return (float)displacement().Length();} //!< Returns the distance (magnitude of displacement) this voxel has moved from its initial nominal position. (GCS)
	float angularDisplacementMagnitude() const {return (float)orient.Angle();} //!< Returns the angle (magnitude of angular displacement) this voxel has rotated from its initial nominal orientation. (GCS)
	Vec3D<double> velocity() const {return linMom*mat->_massInverse;} //!< Returns the 3D velocity of this voxel in m/s (GCS)
	float velocityMagnitude() const {return (float)(linMom.Length()*mat->_massInverse);} //!< Returns the velocity of this voxel in m/s.
	Vec3D<double> angularVelocity() const {return angMom*mat->_momentInertiaInverse;} //!< Returns the 3D angular velocity of this voxel in rad/s (GCS)
	float angularVelocityMagnitude() const {return (float)(angMom.Length()*mat->_momentInertiaInverse);} //!< Returns the angular velocity of this voxel in rad/s.
	float kineticEnergy() const {return (float)(0.5*(mat->_massInverse*linMom.Length2() + mat->_momentInertiaInverse*angMom.Length2()));} //!< Returms the kinetic energy of this voxel in Joules.
	float volumetricStrain() const {return (float)(strain(false).x+strain(false).y+strain(false).z);} //!< Returns the volumetric strain of the voxel according to the definition at http://www.colorado.edu/engineering/CAS/courses.d/Structures.d/IAST.Lect05.d/IAST.Lect05.pdf
	float pressure() const {return -mat->youngsModulus()*volumetricStrain()/(3*(1-2*mat->poissonsRatio()));} //!< Returns the engineering internal "pressure" in Pa according to the definition at http://www.colorado.edu/engineering/CAS/courses.d/Structures.d/IAST.Lect05.d/IAST.Lect05.pdf

	//material state
	bool isYielded() const; //!< Returns true if the stress in this voxel has ever exceeded the yield stress. Technically, this returns true if any of the connected links have yielded since the stress state of the voxel is never expressly calculated.
	bool isFailed() const; //!< Returns true if the stress in this voxel has ever exceeded the failure stress. Technically, this returns true if any of the connected links have failed since the stress state of the voxel is never expressly calculated.

	//@ voxel level for heat diffusion experiments later
	float temperature() {return temp;} //!< Returns the current temperature of this voxel in degrees Celsius.
	void setTemperature(float temperature); //!< Specifies the temperature for this voxel. This adds (or subtracts) the correct amount of thermal energy to leave the voxel at ths specified temperature, but this temperature will not be maintaned without subsequent determines the amount of scaling from the temperature

	Vec3D<float> externalForce(); //!< Returns the current external force applied to this voxel in newtons. If the voxel is not fixed this will return any applied external forces. If fixed it will return the current reaction force necessary to enforce the zero-motion constraint.
	Vec3D<float> externalMoment(); //!< Returns the current external moment applied to this voxel in N-m. If the voxel is not fixed this will return any applied external moments. If fixed it will return the current reaction moment necessary to enforce the zero-motion constraint.

	void haltMotion(){linMom = angMom = Vec3D<>(0,0,0);} //!< Halts all momentum of this block. Unless fixed the voxel will continue to move in subsequent timesteps.

	void enableFloor(bool enabled) {enabled ? boolStates |= FLOOR_ENABLED : boolStates &= ~FLOOR_ENABLED;} //!< Enables this voxel interacting with the floor at Z=0. @param[in] enabled Enable interaction
	bool isFloorEnabled() const {return boolStates & FLOOR_ENABLED ? true : false;} //!< Returns true of this voxel will interact with the floor at Z=0.
	bool isFloorStaticFriction() const {return boolStates & FLOOR_STATIC_FRICTION ? true : false;} //!< Returns true if this voxel is in contact with the floor and stationary in the horizontal directions. This corresponds to that voxel being in the mode of static friction (as opposed to kinetic) with the floor.
	float floorPenetration() const {return (float)(baseSizeAverage()/2 - mat->nominalSize()/2 - pos.z);} //!< Returns the interference (in meters) between the collision envelope of this voxel and the floor at Z=0. Positive numbers correspond to interference. If the voxel is not touching the floor 0 is returned.

	Vec3D<double> force(); //!< Calculates and returns the sum of the current forces on this voxel. This would normally only be called internally, but can be used to query the state of a voxel for visualization or debugging.
	Vec3D<double> moment(); //!< Calculates and returns the sum of the current moments on this voxel. This would normally only be called internally, but can be used to query the state of a voxel for visualization or debugging.

	float transverseArea(CVX_Link::linkAxis axis); //!< Returns the transverse area of this voxel with respect to the specified axis. This would normally be called only internally, but can be used to calculate the correct relationship between force and stress for this voxel if Poisson's ratio is non-zero.
	float transverseStrainSum(CVX_Link::linkAxis axis); //!< Returns the sum of the current strain of this voxel in the two mutually perpindicular axes to the specified axis. This would normally be called only internally, but can be used to correctly calculate stress for this voxel if Poisson's ratio is non-zero.

	float dampingMultiplier() {return 2*mat->_sqrtMass*mat->zetaInternal/previousDt;} //!< Returns the damping multiplier for this voxel. This would normally be called only internally for the internal damping calculations.

	//a couple global convenience functions to have wherever the link enums are used
	static inline CVX_Link::linkAxis toAxis(linkDirection direction) {return (CVX_Link::linkAxis)((int)direction/2);} //!< Returns the link axis of the specified link direction.
	static inline linkDirection toDirection(CVX_Link::linkAxis axis, bool positiveDirection) {return (linkDirection)(2*((int)axis) + positiveDirection?0:1);} //!< Returns the link direction of the specified link axis and sign.
	static inline bool isNegative(linkDirection direction) {return direction%2==1;} //!< Returns true if the specified link direction is negative.
	static inline bool isPositive(linkDirection direction) {return direction%2==0;} //!< Returns true if the specified link direction is positive.
	static inline linkDirection toOpposite(linkDirection direction) {return (linkDirection)(direction-direction%2 + (direction+1)%2);} //!< Returns the opposite (negated) link direction of the specified direction.



private:
	typedef int voxState;
	enum voxFlags { //default of each should be zero for easy clearing
		SURFACE = 1<<1, //on the surface?
		FLOOR_ENABLED = 1<<2, //interact with a floor at z=0?
		FLOOR_STATIC_FRICTION = 1<<3, //is the voxel in a state of static friction with the floor?
		COLLISIONS_ENABLED = 1<<5
	};

	CVX_MaterialVoxel* mat;
	short ix, iy, iz;
	CVX_External* ext;

	void replaceMaterial(CVX_MaterialVoxel* newMaterial); //!<Replaces the material properties of this voxel (but not links) to this new CVX_Material. May cause unexpected behavior if certain material properties are changed mid-simulation. @param [in] newMaterial The new material properties for this voxel.

	void addLinkInfo(linkDirection direction, CVX_Link* link); //adds the information about a link connected to this voxel in the specified direction
	void removeLinkInfo(linkDirection direction); //removes the information about a link connected to this voxel in the specified direction
	CVX_Link* links[6]; //links in the 6 cardinal directions according to linkDirection enumeration



	//voxel state
	Vec3D<double> pos;					//current center position (meters) (GCS)
	Vec3D<double> linMom;				//current linear momentum (kg*m/s) (GCS)
	Quat3D<double> orient;				//current orientation (GCS)
	Vec3D<double> angMom;				//current angular momentum (kg*m^2/s) (GCS)

	voxState boolStates;				//single int to store many boolean state values as bit flags according to 
	void setFloorStaticFriction(bool active) {active? boolStates |= FLOOR_STATIC_FRICTION : boolStates &= ~FLOOR_STATIC_FRICTION;}

	float temp; //0 is no expansion

	void floorForce(float dt, Vec3D<double>* pTotalForce); //modifies pTotalForce to include the object's interaction with a floor. This should be calculated as the last step of sumForce so that pTotalForce is complete.


	Vec3D<float> strain(bool poissonsStrain) const; //LCS returns voxel strain. if tensionStrain true and no actual tension in that
	Vec3D<float> poissonsStrain();
	
	Vec3D<float> pStrain; //cached poissons strain
	bool poissonsStrainInvalid; //flag for recomputing poissons strain.

	void eulerStep(float dt); //execute an euler time step at the specified dt
	float previousDt; //remember the duration of the last timestep of this voxel

	void updateSurface();
	void enableCollisions(bool enabled, float watchRadius = 0.0f); //watchRadius in voxel units
	bool isCollisionsEnabled() const {return boolStates & COLLISIONS_ENABLED ? true : false;}
	void generateNearby(int linkDepth, bool surfaceOnly = true);

	Vec3D<float>* lastColWatchPosition;
	std::vector<CVX_Collision*>* colWatch;
	std::vector<CVX_Voxel*>* nearby;


	friend class CVoxelyze; //give access to private members directly
	friend class CVXS_SimGLView; //TEMPORARY
	friend class CVX_LinearSolver;

};


//References:
//http://gafferongames.com/game-physics/physics-in-3d/

//Poissons ratio (volumetric expansion)
//http://www.colorado.edu/engineering/CAS/courses.d/Structures.d/IAST.Lect05.d/IAST.Lect05.pdf


#endif //VX_VOXEL_H