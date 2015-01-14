/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VOXELYZE_H
#define VOXELYZE_H

//#include "VX_Enums.h"
#include "Array3D.h"
#include "VX_Link.h"
#include "VX_Voxel.h"
#include <vector> //delete if PIMPL'd
#include <list> //delete if PIMPL'd
#include <algorithm> //delete if PIMPL'd

#define DEFAULT_VOXEL_SIZE 0.001 //1mm default voxel size

class CVX_Material;
class CVX_MaterialVoxel;
class CVX_MaterialLink;
class CVX_Collision;

//! Defines and simulates a configuration of voxels.
/*!
The primary user-accesible concepts of Voxelyze are materials, voxels, and links.

Materials represent an abstract material with physical properties and must be added via one of the addMaterial() functions before any voxels may be added. Materials hold no state - they are essentially just a container for a set of material properties.

Voxels are added using setVoxel() and use a previously added material. Voxels always begin as cubes on a regular lattice - therefore only x, y, and z indices are need to fully specify where to add a voxel. A voxel object may have an "external" influence (such as being fixed to ground in one or more degrees of freedom, or an external force applied) which is accessed through its external() member function. A voxel also contains state information such as its current position and velocity.

Links are never added directly. Instead whenenver a voxel is added a link is automatically created to any adjacent voxels. Links hold all the structural information of the simulated Euler-Bernoulli beam connecting two adjacent voxels. In the case of two voxels with dissimilar materials, the link object automatically calculates the appropriate composite material properties. Links hold state information such as stress, strain, and strain energy. 

The pointers returned for materials, voxels, and links are valid for the entire lifetime of that particular material, voxel, or link. It is also possible to iterate over all materials, voxels, or links using, for example, materialCount() and material().

Once the desired voxels are set up, the simulation is run by simply calling doTimeStep() repeatedly until the desired result is acheived. Because Voxelyze utilizes numerical integration, too large of a timestep will cause divergent instability. This depends on many things, most notably the stiffness and density of voxels in use. Heavy, stiff voxels will be stable for larger timesteps. Calling doTimeStep() without an argument will use a good estimation of the larger stable timestep duration. This timestep is used synchronously for all voxels, so the entire simulation will run at the slowest timestep (as determined usually by the stiffest/lightest material in use).

*/
class CVoxelyze {
public:
	//! Defines various types of information to query about the state of a voxelyze object
	enum stateInfoType { 
		DISPLACEMENT, //!< Displacement from a nominal position in meters
		VELOCITY, //!< Velocity in meters per second
		KINETIC_ENERGY, //!< Kinetic energy in joules
		ANGULAR_DISPLACEMENT, //!< Angular displacement from nominal orientation in radians
		ANGULAR_VELOCITY, //!< Angular velocity in radians per second
		ENG_STRESS, //!< Engineering stress in pascals
		ENG_STRAIN, //!< Engineering strain (unitless)
		STRAIN_ENERGY, //!< Strain energy in joules
		PRESSURE, //!< pressure in pascals
		MASS //!< mass in Kg
	};

	//! The type if value desired for a given stateInfoType. Considers all voxels or all links (depending on stateInfoType).
	enum valueType {
		MIN, //!< Minimum of all values
		MAX, //!< Maximum of all values
		TOTAL, //!< Total (sum) of all values
		AVERAGE //!< Average of all values
	};

	CVoxelyze(double voxelSize = DEFAULT_VOXEL_SIZE); //!< Constructs an empty voxelyze object. @param[in] voxelSize base size of the voxels in this instance in meters.
	CVoxelyze(const char* jsonFilePath) {loadJSON(jsonFilePath);} //!< Constructs a voxelyze object from a *.vxl.json file. The details of this file format are available in the Voxelyze user guide. @param[in] jsonFilePath path to the json file
	CVoxelyze(rapidjson::Value* pV); //!< Constructs a voxelyze object from a rapidjson parser node that contains valid voxelyze sub-nodes. @param[in] pV pointer to a rapidjson Value that contains Voxelyze information. See rapidjson documentation and the *.vxl.json format info in the voxelyze user guide.
	~CVoxelyze(void); //!< Destructor
	CVoxelyze(CVoxelyze& VIn) {*this = VIn;} //!< Copy constructor
	CVoxelyze& operator=(CVoxelyze& VIn); //!< Equals operator

	void clear(); //!< Erases all voxels and materials and restores the voxelyze object to its default (empty) state.
	bool loadJSON(const char* jsonFilePath); //!< Clears this voxelyze instance and loads fresh from a *.vxl.json file. The details of this file format are available in the Voxelyze user guide. @param[in] jsonFilePath path to the json file
	bool saveJSON(const char* jsonFilePath); //!< Saves this voxelyze instance to a json file. All voxels are saved at their default locations - the state is not captured. It is recommended to specify the standard *.vxl.json file suffix. @param[in] jsonFilePath path to the desired json file. Will create or overwrite a file at this path.

	bool doLinearSolve(/*SOLVER thisSolver, float stepPercentage = 1.0f*/); //!< Linearizes the voxelyze object and does a one-time linear solution to set the position and orientation of all voxels. The current state of the voxel object will be discarded. Currently only the pardiso solver is supported. To make use of this feature voxelyze must be built with PARDISO_5 defined in the preprocessor. A valid pardiso 5 license file and library file (i.e libpardiso500-WIN-X86-64.dll for windows) should be obtained from www.pardiso-project.org and placed in the directory your executable will be run from.

	bool doTimeStep(float dt = -1.0f); //!< Executes a single timestep on this voxelyze object and updates all state information (voxel positions and orientations) accordingly. In most situations this function will be called repeatedly until the desired result is obtained. @param[in] dt The timestep to take in seconds. If this value is too large the system will display divergent instability. Use recommendedTimeStep() to get a conservative estimate of the largest stable timestep. Also the default value of -1.0f will blindly use this recommended timestep.
	float recommendedTimeStep() const; //!< Returns an estimate of the largest stable time step based on the current state of the simulation. If poisson's ratios are all zero and material properties do not otherwise change this can be called once and the same timestep value used for all subsequent doTimeStep() calls. Otherwise the timestep should be recalculated whenever the simulation has changed.
	void resetTime(); //!< Resets all voxels to their initial state and zeroes the elapsed time counter. Call this to "start over" without changing any of the voxels.

	CVX_Material* addMaterial(float youngsModulus = 1e6f, float density = 1e3f); //!< Adds a material to this voxelyze object with the minimum necessary information for dynamic simulation (stiffness, density). Returns a pointer to the newly created material that can be used to further specify properties using CVX_Material public memer functions. See CVX_Material documentation. This function does not create any voxels, but a returned CVX_material pointer is a necessary parameter for the setVoxel() function that does add voxels. @param[in] youngsModulus the desired stiffness (Young's Modulus) of this material in Pa (N/m^2). @param[in] density the desired density of this material in Kg/m^3.
	CVX_Material* addMaterial(rapidjson::Value& mat); //!< Adds a material to this voxelyze object from a rapidjson parser node that contains valid CVX_Material sub-nodes. @param[in] mat reference to a rapidjson Value that contains CVX_Material information. See rapidjson documentation and the *.vxl.json format info in the voxelyze user guide.
	CVX_Material* addMaterial(const CVX_Material& mat); //!< Adds a material to this voxelyse object and copies all material properties from the specified CVX_Material. @param[in] mat the material to copy all information from. This material is not affected, and can either be a part of this voxelyze object or constructed externally.

	bool removeMaterial(CVX_Material* toRemove); //!< Removes the specified material from the voxelyze object and deletes all voxels currently using it. @param[in] toRemove pointer to a material to remove from this voxelyze object. 
	bool replaceMaterial(CVX_Material* replaceMe, CVX_Material* replaceWith); //!< Replaces all voxels of one material with another material. @param[in] replaceMe material to be replaced @param[in] replaceWith material to replace with. This material must already be a part of the simulation - the pointer will have originated from addMaterial() or material().
	int materialCount() {return voxelMats.size();} //!< Returns the number of materials currently in this voxelyze object.
	CVX_Material* material(int materialIndex) {return (CVX_Material*)voxelMats[materialIndex];} //!< Returns a pointer to a material that has been added to this voxelyze object. CVX_Material public member functions can be safely called on this pointer to modify the material. A given index may or may not always return the same material - Use material pointers to keep permanent handles to specific materials. This function is primarily used while iterating through all materials in conjuntion with materialCount(). @param[in] materialIndex the current index of a material. Valid range from 0 to materialCount()-1.


	CVX_Voxel* setVoxel(CVX_Material* material, int xIndex, int yIndex, int zIndex); //!< Adds a voxel made of material at the specified index. If a voxel already exists here it is replaced. The returned pointer can be safely modified by calling any CVX_Voxel public member function on it. @param[in] material material this voxel is made from. This material must already be a part of the simulation - the pointer will have originated from addMaterial() or material(). @param[in] xIndex the X index of this voxel. @param[in] yIndex the Y index of this voxel. @param[in] zIndex the Z index of this voxel.
	CVX_Voxel* voxel(int xIndex, int yIndex, int zIndex) const {return voxels(xIndex, yIndex, zIndex);} //!< Returns a pointer to the voxel at this location if one exists, or null otherwise. The returned pointer can be safely modified by calling any CVX_Voxel public member function on it. @param[in] xIndex the X index to query. @param[in] yIndex the Y index to query. @param[in] zIndex the Z index to query.
	int voxelCount() const {return voxelsList.size();} //!< Returns the number of voxels currently in this voxelyze object.
	CVX_Voxel* voxel(int voxelIndex) const {return voxelsList[voxelIndex];} //!< Returns a pointer to a voxel that has been added to this voxelyze object. CVX_Voxel public member functions can be safely called on this pointer to query or modify the voxel. A given index may or may not always return the same voxel - Use voxel pointers to keep permanent handles to specific voxels. This function is primarily used while iterating through all voxels in conjuntion with voxelCount(). @param[in] voxelIndex the current index of a voxel. Valid range from 0 to voxelCount()-1.
	const std::vector<CVX_Voxel*>* voxelList() const {return &voxelsList;} //!< Returns a pointer to the internal list of voxels in this voxelyze object. In some situations where all voxels must be iterated over quickly there may be performance gains from iterating directly on the underlying std::vector container accessed with this function.


	int indexMinX() const {return voxels.minIndices().x;} //!< The minimum X index of any voxel in this voxelyze object. Use to determine limits.
	int indexMaxX() const {return voxels.maxIndices().x;} //!< The minimum Y index of any voxel in this voxelyze object. Use to determine limits.
	int indexMinY() const {return voxels.minIndices().y;} //!< The minimum Z index of any voxel in this voxelyze object. Use to determine limits.
	int indexMaxY() const {return voxels.maxIndices().y;} //!< The maximum X index of any voxel in this voxelyze object. Use to determine limits.
	int indexMinZ() const {return voxels.minIndices().z;} //!< The maximum Y index of any voxel in this voxelyze object. Use to determine limits.
	int indexMaxZ() const {return voxels.maxIndices().z;} //!< The maximum Z index of any voxel in this voxelyze object. Use to determine limits.

	CVX_Link* link(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction) const; //!< Returns a pointer to the link at this voxel location in the direction indicated if one exists, or null otherwise. The returned pointer can be safely modified by calling any CVX_Link public member function on it. @param[in] xIndex the X index of the voxel to query. @param[in] yIndex the Y index of the voxel to query. @param[in] zIndex the Z index of the voxel to query. @param direction the direction from the specified voxel to look for a link.
	int linkCount() const {return linksList.size();} //!< Returns the number of links currently in this voxelyze object.
	CVX_Link* link(int linkIndex) {return linksList[linkIndex];} //!< Returns a pointer to a link that is a part of this voxelyze object. CVX_Link public member functions can be safely called on this pointer to query the link. A given index may or may not always return the same link - Use link pointers to keep permanent handles to specific voxels. This function is primarily used while iterating through all links in conjuntion with linkCount(). @param[in] linkIndex the current index of a link. Valid range from 0 to linkCount()-1.
	const std::vector<CVX_Link*>* linkList() const {return &linksList;}  //!< Returns a pointer to the internal list of links in this voxelyze object. In some situations where all links must be iterated over quickly there may be performance gains from iterating directly on the underlying std::vector container accessed with this function.

	const std::vector<CVX_Collision*>* collisionList() const {return &collisionsList;} //!< Returns a pointer to the internal list of collisions in this voxelyze object. See CVX_Collision documentation for more information on collision objects.

	void setVoxelSize(double voxelSize); //!< Sets the base voxel size for the entire voxelyze object. @param[in] voxelSize base size of the voxels in this instance in meters.
	double voxelSize() const {return voxSize;} //!< Returns the base voxel size in meters.

	//temperature
	void setAmbientTemperature(float relativeTemperature, bool allVoxels = false); //!< Sets the relative ambient temperature of the environment. A relative temperature of zero indicates that materials will experience zero thermal expansion. @param[in] relativeTemperature Relative temperature in degrees celsius. @param[in] allVoxels Flag to denote whether all voxels should take this temperature immediately. If true any diffusion time is skipped and thermal energy is added (or removed) from each voxel to reach this temperature. If false voxels will not change temperature.
	float ambientTemperature(void) const {return ambientTemp;} //!< Returns the current relative ambient temperature

	void setGravity(float g=1.0f); //!< Sets the gravitational acceleration to apply to all voxels. Gravity acts in the -Z direction. Set to 0 to disable. @param[in] g Gravitational acceleration in g's. 1 g = -9.80665 m/s^2.
	float gravity(void) const {return grav;} //!< Returns the current gravitational acceleration in g's. 1 g = -9.80665 m/s^2.

	void enableFloor(bool enabled = true); //!< Enables or disables a floor that resists voxel penetration at z=0. @param[in] enabled If true, enables the floor. Otherwise disables it.
	bool isFloorEnabled(void) const {return floor;} //!< Returns a boolean value indication if the floor is enabled or not.

	void enableCollisions(bool enabled = true); //!< Enables or disables a collision watcher that results in voxels resisting penetration with one another. This may slow down the simulation significantly. @param[in] enabled If true, enables the collision detection. Otherwise disables it.
	bool isCollisionsEnabled(void) const {return collisions;} //!< Returns a boolean value indication if the collision watcher is enabled or not.

	//info
	float stateInfo(stateInfoType info, valueType type); //!< Returns a specific piece of information about the current state of the simulation. This method is not gaurenteed threadafe. @param[in] info The class of information desired. @param[in] type The type of the value to be returned.

private:
	double voxSize; //lattice size
	float currentTime; //current time of the simulation in seconds
	float ambientTemp;
	float grav;
	bool floor, collisions;

	//constants... somewhere else?
	float boundingRadius; //(in voxel units) radius to collide a voxel at
	float watchDistance; //(in voxel units) Distance between voxels (not including 2*boundingRadius for each voxel) to watch for collisions from.


	std::vector<CVX_MaterialVoxel*> voxelMats; //up-to-date list of all voxel materials existing in this simulation
	bool exists(const CVX_MaterialVoxel* toCheck); //returns the iterator in materialList if a material exists, NULL otherwise.

	//generated materials are strictly combos of two materials in voxelMats used for links that bridge materials.
	CVX_MaterialLink* combinedMaterial(CVX_MaterialVoxel* mat1, CVX_MaterialVoxel* mat2); //returns a pointer to a combined material of the two provided materials. if the materials are identical, it just returns that. Otherwise checks if it exists in linkMats at returns that. Otherwise, generates the combined material, adds it to linkMats, and returns a pointer to it.
	std::list<CVX_MaterialLink*> linkMats; //any generated material combinations

	CVX_Voxel* addVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex); //creates a new voxel if there isn't one here. Otherwise
	void removeVoxel(int xIndex, int yIndex, int zIndex);
	void replaceVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex); //replaces the material of this voxel while retaining its position, velocity, etc.


	CArray3D<CVX_Voxel*> voxels; //main voxel array 3D lookup
	std::vector<CVX_Voxel*> voxelsList; //main list of existing voxels (no particular order) (always kept syncd with voxels)

	CArray3D<CVX_Link*> links[3]; //main link arrays in the X[0], Y[1] and Z[2] directions. (0,0,0) is the bond pointting in the positive direction from voxel (0,0,0)
	std::vector<CVX_Link*> linksList; //main list of all existing links (no particular order) (always kept syncd with voxels)

	CVX_Link* addLink(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction); //adds a link (if one isn't already present) and updates parameters
	void removeLink(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction); //removes just the link and all references to it in connected voxels

	int xIndexLinkOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::X_NEG) ? -1 : 0;} //the link X index offset from a voxel index and link direction
	int yIndexLinkOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::Y_NEG) ? -1 : 0;} //the link Y index offset from a voxel index and link direction
	int zIndexLinkOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::Z_NEG) ? -1 : 0;} //the link Z index offset from a voxel index and link direction
	int xIndexVoxelOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::X_NEG) ? -1 : ((direction == CVX_Voxel::X_POS) ? 1 : 0);} //the voxel X index offset of a voxel across a link in the specified direction
	int yIndexVoxelOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::Y_NEG) ? -1 : ((direction == CVX_Voxel::Y_POS) ? 1 : 0);} //the voxel Y index offset of a voxel across a link in the specified direction
	int zIndexVoxelOffset(CVX_Voxel::linkDirection direction) const {return (direction == CVX_Voxel::Z_NEG) ? -1 : ((direction == CVX_Voxel::Z_POS) ? 1 : 0);} //the voxel Z index offset of a voxel across a link in the specified direction

	std::vector<CVX_Collision*> collisionsList;
	bool collisionsStale, nearbyStale; //flags to recalculate collision lists and voxel nearby lists.

	void updateCollisions();
	void clearCollisions(); //remove all existing collisions
	void regenerateCollisions(float threshRadiusSq);

	bool writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w);
	bool readJSON(rapidjson::Value& vxl);

};


#endif //VOXELYZE_H
