/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "Voxelyze.h"
#include "VX_Material.h"
#include "VX_MaterialLink.h"
#include "VX_MaterialVoxel.h"
#include "VX_Voxel.h"
#include "VX_Link.h"
#include "VX_LinearSolver.h"
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <assert.h>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"

CVoxelyze::CVoxelyze(double voxelSize)
{
	clear();
	voxSize = voxelSize <= 0 ? DEFAULT_VOXEL_SIZE : voxelSize;
}

CVoxelyze::~CVoxelyze(void)
{
	clear();
}

CVoxelyze& CVoxelyze::operator=(CVoxelyze& VIn)
{
	setVoxelSize(VIn.voxSize);
	setAmbientTemperature(VIn.ambientTemperature(), true);
	setGravity(VIn.gravity());
	enableFloor(VIn.isFloorEnabled());
	enableCollisions(VIn.isCollisionsEnabled());

	//add all materials, map from VIn material to this material
	std::unordered_map<CVX_Material*, CVX_Material*> matMap;
	for (int i=0; i<VIn.materialCount(); i++) matMap[VIn.material(i)] = addMaterial(*(VIn.material(i)));

	//for each voxel in VIn, call setVoxel here...
	for (int i=0; i<VIn.voxelCount(); i++){
		CVX_Voxel* pVIn = VIn.voxel(i);
		CVX_Voxel* pVOut = setVoxel(matMap[pVIn->material()], pVIn->indexX(), pVIn->indexY(), pVIn->indexZ());
		*pVOut->external() = *pVIn->external();
	}
	return *this;
}


bool CVoxelyze::loadJSON(const char* jsonFilePath)
{
	std::ifstream t(jsonFilePath);
	if (t){
		std::stringstream buffer;
		buffer << t.rdbuf();

		rapidjson::Document doc;
		doc.Parse(buffer.str().c_str());
		readJSON(doc);

		t.close();
		return true;
	}
	return false;
	//else error!
}

bool CVoxelyze::saveJSON(const char* jsonFilePath)
{
	std::ofstream t(jsonFilePath);
	if (t){
		rapidjson::StringBuffer s;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> w(s);
		writeJSON(w);
		
		t << s.GetString();
		t.close();
		return true;
	}
	return false;
	//else error!
}

bool CVoxelyze::readJSON(rapidjson::Value& vxl)
{
	clear();

	if (!vxl.IsObject()) {return false;}
	
	if (!vxl.HasMember("voxelSize") || !vxl["voxelSize"].IsDouble()) {return false;}
	voxSize = vxl["voxelSize"].GetDouble();

	if(!vxl.HasMember("materials") || !vxl["materials"].IsArray()) {return false;}
	rapidjson::Value& m = vxl["materials"];
	for (int i=0; i<(int)m.Size(); i++) {
		addMaterial(m[i]);
	}


	//parse externals?
	//allocate space
	if (vxl.HasMember("voxels") && vxl["voxels"].IsArray() && vxl["voxels"].Size()%4 == 0){
		rapidjson::Value& v = vxl["voxels"];

		//get min.max
		int minX=INT_MAX, maxX=INT_MIN, minY=INT_MAX, maxY=INT_MIN, minZ=INT_MAX, maxZ=INT_MIN;
		for (int i=0; i<(int)v.Size()/4; i++){
			int x = v[4*i].GetInt(), y=v[4*i+1].GetInt(), z=v[4*i+2].GetInt();
			if (x<minX) minX=x;
			if (x>maxX) maxX=x;
			if (y<minY) minY=y;
			if (y>maxY) maxY=y;
			if (z<minZ) minZ=z;
			if (z>maxZ) maxZ=z;
		}

		voxels.resize(maxX-minX, maxY-minY, maxZ-minZ, minX, minY, minZ);
		voxelsList.reserve(v.Size()/4);
		for (int i=0; i<3; i++) links[i].resize(maxX-minX+1, maxY-minY+1, maxZ-minZ+1, minX-1, minY-1, minZ-1);

		//add 'em!
		for (int i=0; i<(int)v.Size()/4; i++) addVoxel(voxelMats[v[i*4+3].GetInt()], v[4*i].GetInt(), v[4*i+1].GetInt(), v[4*i+2].GetInt());
	}

	if (vxl.HasMember("externals") && vxl["externals"].IsArray()){
		for (int i=0; i<(int)vxl["externals"].Size(); i++){
			rapidjson::Value& ext = vxl["externals"][i];
			if (!(ext.HasMember("voxelIndices") && ext["voxelIndices"].IsArray())) continue; //invalid external

			bool dof[6] = {false};
			double disp[6] = {0};
			Vec3D<float> force, moment;

			if (ext.HasMember("fixed") && ext["fixed"].IsArray() && ext["fixed"].Size()==6) for (int j=0; j<6; j++){dof[j] = ext["fixed"][j].GetBool();}
			if (ext.HasMember("translate") && ext["translate"].IsArray() && ext["translate"].Size()==3) for (int j=0; j<3; j++){disp[j] = ext["translate"][j].GetDouble();}
			if (ext.HasMember("rotate") && ext["rotate"].IsArray() && ext["rotate"].Size()==3) for (int j=0; j<3; j++){disp[3+j] = ext["rotate"][j].GetDouble();}
			if (ext.HasMember("force") && ext["force"].IsArray() && ext["force"].Size()==3) for (int j=0; j<3; j++){force[j] = (float)ext["force"][j].GetDouble();}
			if (ext.HasMember("moment") && ext["moment"].IsArray() && ext["moment"].Size()==3) for (int j=0; j<3; j++){moment[j] = (float)ext["moment"][j].GetDouble();}

			for (int j=0; j<(int)ext["voxelIndices"].Size(); j++){
				CVX_External* pE = voxelsList[ext["voxelIndices"][j].GetInt()]->external();
				for (int k=0; k<6; k++)	if (dof[k]) pE->setDisplacement((dofComponent)(1<<k), disp[k]); //fixed degree of freedom
				pE->addForce(force);
				pE->addMoment(moment);
			}
		}
	}

	return true;
}

#include <iostream>
bool CVoxelyze::writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w)
{
	w.StartObject();
	w.Key("voxelSize");		w.Double(voxSize);
	if (ambientTemp != 0){		w.Key("relativeAmbientTemperature");	w.Double((double)ambientTemp);}
	if (grav != 0){				w.Key("gravityAcceleration");			w.Double((double)grav);}
	if (floor == true){			w.Key("floorEnabled");					w.Bool(floor);}
	if (collisions == true){	w.Key("collisionsEnabled");				w.Bool(collisions);}

	std::unordered_map<CVX_Material*, int> m2i; //lookup from material pointer to material index

	w.Key("materials");
	w.StartArray();
	for (int i=0; i<materialCount(); i++){
		m2i[material(i)] = i;
		material(i)->writeJSON(w);
	}
	w.EndArray();

	std::vector<CVX_External*> exts; //catalog of externals
	std::vector<std::vector<int>> extVoxIndices; //array of voxels associated with each external
	w.Key("voxels");
	w.StartArray();
	for (int i=0; i<voxelCount(); i++) { //array of x0, y0, z0, material0, x1, y1, z1, material1, ...
		CVX_Voxel* pVox = voxel(i);
		w.Int(pVox->indexX());
		w.Int(pVox->indexY());
		w.Int(pVox->indexZ());
		w.Uint(m2i[pVox->material()]);

		if (pVox->externalExists() && !pVox->external()->isEmpty()){
			bool match = false;
			for (int j=0; j<(int)exts.size(); j++){
				if (*pVox->external() == *exts[j]){ //found one!
					extVoxIndices[j].push_back(i);
					match = true;
					break;
				}
			}
			if (!match){ //doesn't exist yet
				exts.push_back(pVox->external());
				extVoxIndices.push_back(std::vector<int>(1,i));
			}
		}
	}
	w.EndArray();

	if(exts.size() > 0){
		w.Key("externals");
		w.StartArray();
		for (int i=0; i<(int)exts.size(); i++){
			CVX_External* e = exts[i];
			w.StartObject();
	
			if (e->isFixedAny()){ //if anything is fixed
				w.Key("fixed");
				w.StartArray();
				w.Bool(e->isFixed(X_TRANSLATE)); w.Bool(e->isFixed(Y_TRANSLATE)); w.Bool(e->isFixed(Z_TRANSLATE)); w.Bool(e->isFixed(X_ROTATE)); w.Bool(e->isFixed(Y_ROTATE)); w.Bool(e->isFixed(Z_ROTATE));
				w.EndArray();
			}

			if (e->isFixedAnyTranslation() && e->translation() != Vec3D<double>()){w.Key("translate"); w.StartArray(); for (int j=0; j<3; j++) w.Double(e->translation()[j]); w.EndArray();}
			if (e->isFixedAnyRotation() && e->rotation() != Vec3D<double>()){w.Key("rotate"); w.StartArray(); for (int j=0; j<3; j++) w.Double(e->rotation()[j]); w.EndArray();}
			if (!e->isFixedAllTranslation() && e->force() != Vec3D<float>()){w.Key("force"); w.StartArray(); for (int j=0; j<3; j++) w.Double(e->force()[j]); w.EndArray();}
			if (!e->isFixedAllRotation() && e->moment() != Vec3D<float>()){w.Key("moment"); w.StartArray(); for (int j=0; j<3; j++) w.Double(e->moment()[j]); w.EndArray();}

			w.Key("voxelIndices");
			w.StartArray();
			for (int j=0; j<(int)extVoxIndices[i].size(); j++) w.Uint(extVoxIndices[i][j]);
			w.EndArray();
			w.EndObject();
		}
		w.EndArray();
		w.EndObject();
	}

	return true;
}

bool CVoxelyze::doLinearSolve() //linearizes at current point and solves
{
	CVX_LinearSolver solver(this);
	solver.solve();

	return true;
}

bool CVoxelyze::doTimeStep(float dt)
{
	if (dt==0) return true;
	else if (dt<0) dt = recommendedTimeStep();

	//Euler integration:
	bool Diverged = false;
	int linkCount = linksList.size();

#ifdef USE_OMP
#pragma omp parallel for
#endif
	for (int i = 0; i<linkCount; i++){
		linksList[i]->updateForces();
		if (linksList[i]->axialStrain() > 100) Diverged = true; //catch divergent condition! (if any thread sets true we will fail, so don't need mutex...
	}


	if (Diverged) return false;

	if (collisions) updateCollisions();
	int voxCount = voxelsList.size();

#ifdef USE_OMP
#pragma omp parallel for
#endif
	for (int i=0; i<voxCount; i++){
		voxelsList[i]->timeStep(dt);
	}


	currentTime += dt;
	return true;
}

float CVoxelyze::recommendedTimeStep() const
{
	//find the largest natural frequency (sqrt(k/m)) that anything in the simulation will experience, then multiply by 2*pi and invert to get the optimally largest timestep that should retain stability
	float MaxFreq2 = 0.0f; //maximum frequency in the simulation in rad/sec

	for (std::vector<CVX_Link*>::const_iterator it=linksList.begin(); it != linksList.end(); it++){ //for each link
		CVX_Link* pL = (*it);
		//axial
		float m1 = pL->pVNeg->mat->mass(),  m2 = pL->pVPos->mat->mass();
		float thisMaxFreq2 = pL->axialStiffness()/(m1<m2?m1:m2);
		if (thisMaxFreq2 > MaxFreq2) MaxFreq2 = thisMaxFreq2;

		//rotational will always be less than or equal
	}


	if (MaxFreq2 <= 0.0f){ //didn't find anything (i.e no links) check for individual voxelss
		for (std::vector<CVX_Voxel*>::const_iterator it=voxelsList.begin(); it != voxelsList.end(); it++){ //for each link
			float thisMaxFreq2 = (*it)->mat->youngsModulus()*(*it)->mat->nomSize/(*it)->mat->mass(); 
			if (thisMaxFreq2 > MaxFreq2) MaxFreq2 = thisMaxFreq2;
		}
	}
	
	if (MaxFreq2 <= 0.0f) return 0.0f;
	else return 1.0f/(6.283185f*sqrt(MaxFreq2)); //the optimal timestep is to advance one radian of the highest natural frequency
}

void CVoxelyze::resetTime()
{
	currentTime=0.0f;
	collisionsStale = true;
	nearbyStale = true;

	for (std::vector<CVX_Voxel*>::iterator it=voxelsList.begin(); it != voxelsList.end(); it++) (*it)->reset(); //reset each voxel
	for (std::vector<CVX_Link*>::iterator it=linksList.begin(); it != linksList.end(); it++) (*it)->reset(); //for each link
}

void CVoxelyze::clear() //deallocates and returns everything to defaults (except voxel size)
{
	//delete and remove links
	for (std::vector<CVX_Link*>::iterator it = linksList.begin(); it!=linksList.end(); it++) delete *it;
	for (int i=0; i<3; i++)	links[i].clear();
	linksList.clear();

	//delete and remove voxels
	for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it!=voxelsList.end(); it++) delete *it;
	voxelsList.clear();
	voxels.clear();

	//delete and remove materials
	for (std::vector<CVX_MaterialVoxel*>::iterator it = voxelMats.begin(); it!=voxelMats.end(); it++) delete *it;
	voxelMats.clear();

	for (std::list<CVX_MaterialLink*>::iterator it = linkMats.begin(); it!=linkMats.end(); it++) delete *it;
	linkMats.clear();

	//voxSize = DEFAULT_VOXEL_SIZE;
	currentTime=0.0f;
	ambientTemp = 0.0f;
	grav = 0.0f;
	floor = false;
	collisions = false;

	clearCollisions();
	collisionsStale = true;
	nearbyStale = true;

	boundingRadius = 0.75f;
	watchDistance = 1.0f;
}

CVX_Material* CVoxelyze::addMaterial(float youngsModulus, float density)
{
	try {
		CVX_MaterialVoxel* pMat = new CVX_MaterialVoxel(youngsModulus, density, voxSize);
		pMat->setGravityMultiplier(grav);
		voxelMats.push_back(pMat);
		return pMat; 
	}
	catch (std::bad_alloc&){return NULL;}
}

CVX_Material* CVoxelyze::addMaterial(rapidjson::Value& mat)
{
	CVX_MaterialVoxel* pMat = new CVX_MaterialVoxel(mat, voxSize);
	pMat->setGravityMultiplier(grav);
	voxelMats.push_back(pMat);
	return pMat; 
}

CVX_Material* CVoxelyze::addMaterial(const CVX_Material& mat)
{
	CVX_MaterialVoxel* pMat = new CVX_MaterialVoxel(mat, voxSize);
	pMat->setGravityMultiplier(grav);
	voxelMats.push_back(pMat);
	return pMat; 
}

bool CVoxelyze::removeMaterial(CVX_Material* toRemove)
{
	CVX_MaterialVoxel* pMat = (CVX_MaterialVoxel*)toRemove;
	if (!exists(pMat)) return false;

	//remove all voxels that use this material
	for (int k=indexMinZ(); k<=indexMaxZ(); k++){
		for (int j=indexMinY(); j<=indexMaxY(); j++){
			for (int i=indexMinX(); i<=indexMaxX(); i++){
				if (voxels(i, j, k)->material() == pMat) removeVoxel(i, j, k);
			}
		}
	}

	//remove the material
	delete pMat;
	for (int i=0; i<materialCount(); i++) if (voxelMats[i] == pMat) voxelMats.erase(voxelMats.begin()+i);
	assert(!exists(pMat)); //the material should no longer exist.

	return true;
}

bool CVoxelyze::replaceMaterial(CVX_Material* replaceMe, CVX_Material* replaceWith)
{
	if (!exists((CVX_MaterialVoxel*)replaceMe) || !exists((CVX_MaterialVoxel*)replaceWith)) return false;
	
	//switch all voxel references
	for (int iz=indexMinZ(); iz<=indexMaxZ(); iz++){
		for (int iy=indexMinY(); iy<=indexMaxY(); iy++){
			for (int ix=indexMinX(); ix<=indexMaxX(); ix++){
				CVX_Voxel* pV = voxel(ix, iy, iz);
				if (pV->material() == (CVX_MaterialVoxel*)replaceMe) setVoxel(replaceWith, ix, iy, iz);
			}
		}
	}
	return true;
}

CVX_Voxel* CVoxelyze::setVoxel(CVX_Material* material, int xIndex, int yIndex, int zIndex)
{
	if (material == NULL){
		removeVoxel(xIndex, yIndex, zIndex);
		return NULL;
	}
	
	CVX_Voxel* pV = voxels(xIndex, yIndex, zIndex);
	if (pV != NULL){
		replaceVoxel((CVX_MaterialVoxel*)material, xIndex, yIndex, zIndex);
		return pV;
	}
	else {
		return addVoxel((CVX_MaterialVoxel*)material, xIndex, yIndex, zIndex);
	}
}

CVX_Voxel* CVoxelyze::addVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex) //creates a new voxel if there isn't one here. Otherwise
{
	try {
		nearbyStale = collisionsStale = true;

		CVX_Voxel* pV = new CVX_Voxel(newVoxelMaterial, xIndex, yIndex, zIndex);
		voxels.addValue(xIndex, yIndex, zIndex, pV); //add to the array
		voxelsList.push_back(pV);
		pV->pos = Vec3D<double>(xIndex*voxSize, yIndex*voxSize, zIndex*voxSize); //set initial voxel location (extrapolate?)
		pV->enableFloor(floor);
		pV->setTemperature(ambientTemp); //add it at environment temperature
		pV->enableCollisions(collisions);

		//add any possible links utilizing this voxel
		for (int i=0; i<6; i++){ //from X_POS to Z_NEG (0-5 enums)
			addLink(xIndex, yIndex, zIndex, (CVX_Voxel::linkDirection)i); 
		}
		return pV;
	}
	catch (std::bad_alloc&){
		return NULL;
	}
}


void CVoxelyze::removeVoxel(int xIndex, int yIndex, int zIndex)
{
	nearbyStale = collisionsStale = true;

	const CVX_Voxel* pV = voxel(xIndex, yIndex, zIndex);
	if (pV==NULL) return; //no voxel exists here.
	delete pV;
	voxels.removeValue(xIndex, yIndex, zIndex); //remove from the array
	for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it!=voxelsList.end(); it++){ //remove from the list
		if (*it == pV) voxelsList.erase(it);
	}

	//make sure no references are left in the list This should be compiled away in release
	for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it!=voxelsList.end(); it++) assert(*it != pV); 

	//remove any links to this voxel
	for (int i=0; i<6; i++){ //from X_POS to Z_NEG (0-5 enums)
		removeLink(xIndex, yIndex, zIndex, (CVX_Voxel::linkDirection)i); 
	}
}

void CVoxelyze::replaceVoxel(CVX_MaterialVoxel* newVoxelMaterial, int xIndex, int yIndex, int zIndex)
{
	collisionsStale = true; //new material requires new stiffnesses of contact bonds

	//replace the voxel materrial
	CVX_Voxel* pV=voxel(xIndex, yIndex, zIndex);
	pV->replaceMaterial(newVoxelMaterial);

	//reset all the links involving this voxel
	for (int i=0; i<6; i++){ //from X_POS to Z_NEG (0-5 enums)
		removeLink(xIndex, yIndex, zIndex, (CVX_Voxel::linkDirection)i);
		addLink(xIndex, yIndex, zIndex, (CVX_Voxel::linkDirection)i); //adds only if a voxel is found
	}
}

CVX_Link* CVoxelyze::link(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction) const
{
	return links[CVX_Voxel::toAxis(direction)](
		xIndex+xIndexLinkOffset(direction),
		yIndex+yIndexLinkOffset(direction),
		zIndex+zIndexLinkOffset(direction));
}

CVX_Link* CVoxelyze::addLink(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction)
{
	CVX_Link* pL = link(xIndex, yIndex, zIndex, direction);
	if (pL){return pL;} //if a link already exists... well, then it should be up to date.

	//ensure that there are voxels at both ends of the link
	CVX_Voxel* voxel1 = voxels(xIndex, yIndex, zIndex);
	CVX_Voxel* voxel2 = voxels(
		xIndex+xIndexVoxelOffset(direction),
		yIndex+yIndexVoxelOffset(direction),
		zIndex+zIndexVoxelOffset(direction));
	if (voxel1 == NULL || voxel2 == NULL) return NULL; //if no voxel at either position, don't make a link
	

	//make the link and add it to the array+list
	try {
		CVX_MaterialLink* mat = combinedMaterial(voxel1->material(), voxel2->material());
		pL = new CVX_Link(voxel1, voxel2, mat); //, direction);	//make the new link (change to both materials, etc.
		linksList.push_back(pL);							//add to the list
		links[CVX_Voxel::toAxis(direction)].addValue(
			xIndex + xIndexLinkOffset(direction),
			yIndex + yIndexLinkOffset(direction),
			zIndex + zIndexLinkOffset(direction), pL);
	}
	catch (std::bad_alloc&){
		return NULL;
	}
	//Add reference to this link to the relevant voxels
	voxel1->addLinkInfo(direction, pL);
	voxel2->addLinkInfo(CVX_Voxel::toOpposite(direction), pL);
	return pL;
}

void CVoxelyze::removeLink(int xIndex, int yIndex, int zIndex, CVX_Voxel::linkDirection direction)
{
	CVX_Link* pL = link(xIndex, yIndex, zIndex, direction);
	if (pL==NULL) return; //no link here to see!

	//remove the reference in the appropriate link 3d array
	links[CVX_Voxel::toAxis(direction)].removeValue( 
		xIndex + xIndexLinkOffset(direction),
		yIndex + yIndexLinkOffset(direction),
		zIndex + zIndexLinkOffset(direction)); 

	//remove the reference in the list
	for (std::vector<CVX_Link*>::iterator it = linksList.begin(); it!=linksList.end(); it++){ //remove from the list
		if (*it == pL){
			linksList.erase(it);
			break;
		}
	}
	
	//make sure no references are left in the list. This should be compiled away in release
	for (std::vector<CVX_Link*>::iterator it = linksList.begin(); it!=linksList.end(); it++) assert(*it != pL); 

	//remove the reference to this link from one voxel (if it exists)
	CVX_Voxel* voxel1 = voxels(xIndex, yIndex, zIndex);
	if (voxel1) voxel1->removeLinkInfo(direction);

	//remove the reference to this link from the other voxel (if it exists)
	CVX_Voxel* voxel2 = voxels(
		xIndex+xIndexVoxelOffset(direction),
		yIndex+yIndexVoxelOffset(direction),
		zIndex+zIndexVoxelOffset(direction));
	if (voxel2) voxel2->removeLinkInfo(CVX_Voxel::toOpposite(direction));

	delete pL;
}


bool CVoxelyze::exists(const CVX_MaterialVoxel* toCheck)
{
	std::vector<CVX_MaterialVoxel*>::iterator thisIt = std::find(voxelMats.begin(), voxelMats.end(), toCheck);
	return (thisIt == voxelMats.end()) ? false : true;

}

void CVoxelyze::setAmbientTemperature(float temperature, bool allVoxels)
{
	ambientTemp = temperature;
	//for now just set the temperature of each voxel (independent of future
	if (allVoxels){
		for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it != voxelsList.end(); it++){
			(*it)->setTemperature(temperature);
		}
	}
}

void CVoxelyze::setGravity(float g)
{
	grav = g;
	for (std::vector<CVX_MaterialVoxel*>::iterator it=voxelMats.begin(); it != voxelMats.end(); it++){
		(*it)->setGravityMultiplier(grav);
	}
}

void CVoxelyze::enableFloor(bool enabled)
{
	floor = enabled;
	for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it != voxelsList.end(); it++){
		(*it)->enableFloor(enabled);
	}
}

void CVoxelyze::enableCollisions(bool enabled)
{
	if (collisions == enabled) return; //if not changing state

	collisions = enabled;
	for (std::vector<CVX_Voxel*>::iterator it = voxelsList.begin(); it != voxelsList.end(); it++){
		(*it)->enableCollisions(enabled);
	}
	if (!collisions) clearCollisions();
	collisionsStale = true;
}



CVX_MaterialLink* CVoxelyze::combinedMaterial(CVX_MaterialVoxel* mat1, CVX_MaterialVoxel* mat2) 
{
	for (std::list<CVX_MaterialLink*>::iterator it = linkMats.begin(); it != linkMats.end(); it++){
		CVX_MaterialLink* thisMat = *it;
		if ((thisMat->vox1Mat == mat1 && thisMat->vox2Mat == mat2) || (thisMat->vox1Mat == mat2 && thisMat->vox2Mat == mat1))
			return thisMat;
	}

	CVX_MaterialLink* newMat = new CVX_MaterialLink(mat1, mat2);
	linkMats.push_back(newMat);
	mat1->dependentMaterials.push_back(newMat);
	mat2->dependentMaterials.push_back(newMat);

	return newMat;
}


void CVoxelyze::setVoxelSize(double voxelSize) //sets the voxel size.
{
	double scaleFactor = voxelSize/voxSize; //scaling factor
	voxSize = voxelSize;
	
	//update materials
	for (std::vector<CVX_MaterialVoxel*>::iterator it=voxelMats.begin(); it != voxelMats.end(); it++){
		(*it)->setNominalSize(voxelSize);
	}

	//update voxels
	for (std::vector<CVX_Voxel*>::iterator it=voxelsList.begin(); it != voxelsList.end(); it++){
		CVX_Voxel* pV = (*it);
		pV->pos *= scaleFactor;
		pV->haltMotion(); //stop motion to avoid weird huge kinetic energy disparities
		pV->setFloorStaticFriction(false);
	}

	//update links
	for (std::vector<CVX_Link*>::iterator it=linksList.begin(); it != linksList.end(); it++){
		CVX_Link* pL = (*it);
		pL->reset(); //updateProperties();
	}

	collisionsStale = true;
}

void CVoxelyze::updateCollisions()
{
	float watchRadiusVx = 2*boundingRadius+watchDistance; //outer radius to track all voxels within
	float watchRadiusMm = (float)(voxSize*watchRadiusVx); //outer radius to track all voxels within
	float recalcDist = (float)(voxSize*watchDistance/2); //if the voxel moves further than this radius, recalc! //1/2 the allowabl, accounting for 0.5x radius of the voxel iself

	//if voxels have been added/removed, regenerate everybody's nearby list
	if (nearbyStale){
		for (std::vector<CVX_Voxel*>::iterator it=voxelsList.begin(); it != voxelsList.end(); it++){
			(*it)->generateNearby(watchRadiusVx*2, false);
		}
		nearbyStale = false;
		collisionsStale = true;
	}

	//check if any voxels have moved far enough to make collisions stale
	int voxCount = voxelsList.size();

#ifdef USE_OMP
#pragma omp parallel for
#endif
	for (int i=0; i<voxCount; i++){
		CVX_Voxel* pV = voxelsList[i]; //(*it);
		if (pV->isSurface() && (pV->pos - *pV->lastColWatchPosition).Length2() > recalcDist*recalcDist){
			collisionsStale = true;
		}
	}

	if (collisionsStale) regenerateCollisions(watchRadiusMm*watchRadiusMm);

	//update the forces!

	int colCount = collisionsList.size();
#ifdef USE_OMP
#pragma omp parallel for
#endif
	for (int i=0; i<colCount; i++){
		collisionsList[i]->updateContactForce();
	}

}

void CVoxelyze::clearCollisions()
{
	for (std::vector<CVX_Collision*>::iterator it=collisionsList.begin(); it != collisionsList.end(); it++){
		delete (*it);
	}
	collisionsList.clear();

	for (std::vector<CVX_Voxel*>::iterator it=voxelsList.begin(); it != voxelsList.end(); it++){
		(*it)->colWatch->clear();
	}
}


void CVoxelyze::regenerateCollisions(float threshRadiusSq)
{
	clearCollisions();

	//check each combo of voxels and add a collision where necessary
	for (std::vector<CVX_Voxel*>::iterator it=voxelsList.begin(); it != voxelsList.end(); it++){
		CVX_Voxel* pV1 = *it;
		if (pV1->isInterior()) continue; //don't care about interior voxels here.
		*pV1->lastColWatchPosition = (Vec3D<float>)pV1->pos; //remember where collisions were last calculated at

		for (std::vector<CVX_Voxel*>::iterator jt=it+1; jt != voxelsList.end(); jt++){
			CVX_Voxel* pV2 = *jt;
			if (pV2->isInterior() || //don't care about interior voxels here.
				(pV1->pos-pV2->pos).Length2() > threshRadiusSq || //discard anything outside the watch radius
				std::find(pV1->nearby->begin(), pV1->nearby->end(), pV2) != pV1->nearby->end()) //discard if in the connected lattice array
				continue;

			CVX_Collision* pCol = new CVX_Collision(pV1, pV2);
			collisionsList.push_back(pCol);
			pV1->colWatch->push_back(pCol);
			pV2->colWatch->push_back(pCol);
		}
	}

	collisionsStale = false; //good to go!
}

float CVoxelyze::stateInfo(stateInfoType info, valueType type)
{
	float returnVal = 0;
	if (type == MAX) returnVal = -FLT_MAX;
	else if (type == MIN) returnVal = FLT_MAX;

	if (info == STRAIN_ENERGY || info==ENG_STRESS || info==ENG_STRAIN){ //Link properties
		if (linkCount() == 0) return 0.0;
		for (std::vector<CVX_Link*>::const_iterator it = linksList.begin(); it!=linksList.end(); it++){ //for each link
			float thisVal = 0;
			switch (info){
				case STRAIN_ENERGY: thisVal = (*it)->strainEnergy();break;
				case ENG_STRESS: thisVal = (*it)->axialStress(); break;
				case ENG_STRAIN: thisVal = (*it)->axialStrain(); break;
				default: thisVal=0;
			}
			switch (type){
				case MIN: if (thisVal < returnVal) returnVal = thisVal; break;
				case MAX: if (thisVal > returnVal) returnVal = thisVal; break;
				case TOTAL: case AVERAGE: returnVal += thisVal; 
			}
		}
		if (type == AVERAGE) returnVal /= linkCount();
	}
	else { //voxel properties: DISPLACEMENT, VELOCITY, KINETIC_ENERGY, ANGULAR_DISPLACEMENT, ANGULAR_VELOCITY, PRESSURE, MASS
		if (voxelCount() == 0) return 0.0;
		for (std::vector<CVX_Voxel*>::const_iterator it = voxelsList.begin(); it!=voxelsList.end(); it++){ //for each voxel
			float thisVal = 0;
			switch (info){
				case DISPLACEMENT: thisVal = (*it)->displacementMagnitude(); break;
				case VELOCITY: thisVal = (*it)->velocityMagnitude(); break;
				case KINETIC_ENERGY: thisVal = (*it)->kineticEnergy();break;
				case ANGULAR_DISPLACEMENT: thisVal = (*it)->angularDisplacementMagnitude(); break;
				case ANGULAR_VELOCITY: thisVal = (*it)->angularVelocityMagnitude(); break;
				case PRESSURE: thisVal = (*it)->pressure(); break;
				case MASS: thisVal = (*it)->material()->mass(); break;
				default: thisVal=0;
			}
			switch (type){
				case MIN: if (thisVal < returnVal) returnVal = thisVal; break;
				case MAX: if (thisVal > returnVal) returnVal = thisVal; break;
				case TOTAL: case AVERAGE: returnVal += thisVal; 
			}
		}
		if (type == AVERAGE) returnVal /= voxelCount();
	}

	return returnVal;
}


