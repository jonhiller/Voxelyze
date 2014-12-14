//*******************************************************************************
//Copyright (c) 2014, Jonathan Hiller (Cornell University)
//If used in publication cite "J. Hiller and H. Lipson "Dynamic Simulation of Soft Heterogeneous Objects" In press. (2011)"
//
//This file is part of Voxelyze.
//Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
//Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
//*******************************************************************************/

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

*/
class CVoxelyze {
public:
	CVoxelyze(double voxelSize = DEFAULT_VOXEL_SIZE);
	CVoxelyze(rapidjson::Value* pV);
	~CVoxelyze(void);
	CVoxelyze(const CVoxelyze& VIn) {*this = VIn;} //copy constructor
	CVoxelyze& operator=(const CVoxelyze& VIn); //equal operator

	void clear(); //deallocates and returns everything to defaults 
	bool loadJSON(const char* jsonFilePath);
	bool saveJSON(const char* jsonFilePath);



	bool doLinearSolve(/*SOLVER thisSolver, float stepPercentage = 1.0f*/); //linearizes at current point and solves

	bool doTimeStep(float dt = -1.0f); //timestep. -1.0f = calculate the recommended timestep
	float recommendedTimeStep() const; //returns recommended time step
	void resetTime(); //resets simulation

//	std::vector<CVX_Material*> voxelMats; //up-to-date list of all voxel materials existing in this simulation
	CVX_Material* addMaterial(float youngsModulus = 1e6f, float density = 1e3f);
	CVX_Material* addMaterial(rapidjson::Value& mat);
	bool removeMaterial(CVX_Material* toRemove);
	bool replaceMaterial(CVX_Material* replaceMe, CVX_Material* replaceWith); //replace all voxels of replaceMe with replaceWith
	int materialCount() {return voxelMats.size();}
	CVX_Material* material(int index) {return (CVX_Material*)voxelMats[index];}

//	std::list<CVX_Material*>* materialList(){return &voxelMats;}

//	CVX_Material* material(int index);
//	int materialCount() const; //returns the total number of materials
//	CVX_Material* materialList() const; //returns pointer to the beginning of a null-terminated list of material pointers

	CVX_Voxel* setVoxel(CVX_Material* material, int xIndex, int yIndex, int zIndex);
	CVX_Voxel* voxel(int xIndex, int yIndex, int zIndex) const {return voxels(xIndex, yIndex, zIndex);}
	int voxelCount() const {return voxelsList.size();}
	CVX_Voxel* voxel(int voxelIndex) const {return voxelsList[voxelIndex];} //?
	const std::vector<CVX_Voxel*>* voxelList() const {return &voxelsList;} //vector for parallel ctimestep computation

//	float voxelInfoMax(CVX_Voxel::voxelInfoType info) const;
//	float linkInfoMax(CVX_Link::linkInfoType info) const;


	int indexMinX() const {return voxels.minIndices().x;}
	int indexMaxX() const {return voxels.maxIndices().x;}
	int indexMinY() const {return voxels.minIndices().y;}
	int indexMaxY() const {return voxels.maxIndices().y;}
	int indexMinZ() const {return voxels.minIndices().z;}
	int indexMaxZ() const {return voxels.maxIndices().z;}

//	CVX_Voxel* voxelsList() const; //null-terminated list
//	int voxelCount() const; //total voxels
//	int voxelCount(CVX_Material* pMaterialToCount) const; //count of voxels of this material

	//hide?
	CVX_Link* link(int xIndex, int yIndex, int zIndex, linkDirection direction) const;
	int linkCount() const {return linksList.size();}
	CVX_Link* link(int linkIndex) {return linksList[linkIndex];}
	const std::vector<CVX_Link*>* linkList() const {return &linksList;}  //vector for parallel ctimestep computation

	const std::vector<CVX_Collision*>* collisionList() const {return &collisionsList;}  //vector for parallel ctimestep computation

	//CVX_Collision* CollisionList() const;
	//int CollisionCount() const;

	void setVoxelSize(double voxelSize); //sets the base voxel size.
	double voxelSize() const {return voxSize;}

	//temperature
	void setAmbientTemperature(float temperature, bool allVoxels = false); //sets the ambient temperature (relative to 0 for no expansion). optionally updates all voxel temps immediately
	float ambientTemperature(void) const {return ambientTemp;}

	void setGravity(float g=1.0f);
	float gravity(void) const {return grav;}

	void enableFloor(bool enabled = true);
	bool isFloorEnabled(void) const {return floor;}

	void enableCollisions(bool enabled = true);
	bool isCollisionsEnabled(void) const {return collisions;}

	//info

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

	//	CVX_Voxel* voxel(int xIndex, int yIndex, int zIndex);
//	CVX_Link* link(int xIndex, int yIndex, int zIndex, linkDirection direction) const;
	
	CVX_Voxel* addVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex); //creates a new voxel if there isn't one here. Otherwise
	void removeVoxel(int xIndex, int yIndex, int zIndex);
	void replaceVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex); //replaces the material of this voxel while retaining its position, velocity, etc.


	CArray3D<CVX_Voxel*> voxels; //main voxel array 3D lookup
	std::vector<CVX_Voxel*> voxelsList; //main list of existing voxels (no particular order) (always kept syncd with voxels)

	//maybe don't need this 3D array?
	CArray3D<CVX_Link*> links[3]; //main link arrays in the X[0], Y[1] and Z[2] directions. (0,0,0) is the bond pointting in the positive direction from voxel (0,0,0)
	std::vector<CVX_Link*> linksList; //main list of all existing links (no particular order) (always kept syncd with voxels)

	CVX_Link* addLink(int xIndex, int yIndex, int zIndex, linkDirection direction); //adds a link (if one isn't already present) and updates parameters
	void removeLink(int xIndex, int yIndex, int zIndex, linkDirection direction); //removes just the link and all references to it in connected voxels

	int xIndexLinkOffset(linkDirection direction) const {return (direction == X_NEG) ? -1 : 0;} //the link X index offset from a voxel index and link direction
	int yIndexLinkOffset(linkDirection direction) const {return (direction == Y_NEG) ? -1 : 0;} //the link Y index offset from a voxel index and link direction
	int zIndexLinkOffset(linkDirection direction) const {return (direction == Z_NEG) ? -1 : 0;} //the link Z index offset from a voxel index and link direction
	int xIndexVoxelOffset(linkDirection direction) const {return (direction == X_NEG) ? -1 : ((direction == X_POS) ? 1 : 0);} //the voxel X index offset of a voxel across a link in the specified direction
	int yIndexVoxelOffset(linkDirection direction) const {return (direction == Y_NEG) ? -1 : ((direction == Y_POS) ? 1 : 0);} //the voxel Y index offset of a voxel across a link in the specified direction
	int zIndexVoxelOffset(linkDirection direction) const {return (direction == Z_NEG) ? -1 : ((direction == Z_POS) ? 1 : 0);} //the voxel Z index offset of a voxel across a link in the specified direction
	//updateCollisionList()

	//float envTemp; //environment temperature
	//bool collisionsInvalid; //set to true if we need to recalculate all collisions from scratch
	std::vector<CVX_Collision*> collisionsList;
	bool collisionsStale, nearbyStale; //flags to recalculate collision lists and voxel nearby lists.

	void updateCollisions();
	//bool collisionsInvalid(float maxDistanceSq); //returns true if any voxels have moved further than maxDistanceSq from the last position collisions were calculated
	void clearCollisions(); //remove all existing collisions
	void regenerateCollisions(float threshRadiusSq);
	//void addConnectedVoxelsToList(CVX_Voxel* pV, std::list<CVX_Voxel*>* pList, Vec3D<>* pBeginLocation, float searchRadiusSq);
	//convenience only...:
	//bool isInList(CVX_Voxel* pV, std::list<CVX_Voxel*>* pList) {return std::find(pList->begin(), pList->end(), pV) != pList->end();} //returns true if the specified voxel is in the list
	//bool isInVector(CVX_Voxel* pV, std::vector<CVX_Voxel*>* pVector) {return std::find(pList->pVector(), pVector->end(), pV) != pVector->end();} //returns true if the specified voxel is in the list

	bool writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w);
	bool readJSON(rapidjson::Value& vxl);
	//void addJSON(rapidjson::Writer* pW);

};


#endif //VOXELYZE_H