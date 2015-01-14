/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_Voxel.h"
#include "VX_Material.h"
#include "VX_Link.h"
#include <algorithm> //for std::find


#include <assert.h>
#ifdef DEBUG
#include <iostream>
#endif

CVX_Voxel::CVX_Voxel(CVX_MaterialVoxel* material, short indexX, short indexY, short indexZ) 
{
	for (int i=0; i<6; i++) links[i]=NULL;
	mat = material;
	ix = indexX;
	iy = indexY;
	iz = indexZ;
	ext=NULL;
	boolStates = 0;
	lastColWatchPosition=NULL;
	colWatch=NULL;
	nearby=NULL;

	reset();
}

CVX_Voxel::~CVX_Voxel(void)
{
	if (lastColWatchPosition) delete lastColWatchPosition;
	if (colWatch) delete colWatch;
	if (nearby) delete nearby;
	if (ext) delete ext;
}

void CVX_Voxel::reset()
{
	pos = originalPosition();
	orient = Quat3D<double>();
	haltMotion(); //zeros linMom and angMom
	setFloorStaticFriction(true);
	temp=0.0f;
	previousDt=0.0f;
	poissonsStrainInvalid = true;
}

CVX_Voxel* CVX_Voxel::adjacentVoxel(linkDirection direction) const
{
	CVX_Link* pL = links[(int)direction];
	if (pL) return pL->voxel(true)==this ? pL->voxel(false) : pL->voxel(true);
	else return NULL;
}

void CVX_Voxel::addLinkInfo(linkDirection direction, CVX_Link* link)
{
	links[direction] = link;
	updateSurface();
}

void CVX_Voxel::removeLinkInfo(linkDirection direction)
{
	links[direction]=NULL;
	updateSurface();
}


void CVX_Voxel::replaceMaterial(CVX_MaterialVoxel* newMaterial)
{
	if (newMaterial != NULL){

		linMom *= newMaterial->_mass/mat->_mass; //adjust momentums to keep velocity constant across material change
		angMom *= newMaterial->_momentInertia/mat->_momentInertia;
		setFloorStaticFriction(false);
		poissonsStrainInvalid = true;

		mat = newMaterial;

	}
}

bool CVX_Voxel::isYielded() const
{
	for (int i=0; i<6; i++){
		if (links[i] && links[i]->isYielded()) return true;
	}
	return false;
}

bool CVX_Voxel::isFailed() const
{
	for (int i=0; i<6; i++){
		if (links[i] && links[i]->isFailed()) return true;
	}
	return false;
}

void CVX_Voxel::setTemperature(float temperature)
{
	temp = temperature;
	for (int i=0; i<6; i++){
		if (links[i] != NULL) links[i]->updateRestLength();
	}
} 


Vec3D<float> CVX_Voxel::externalForce()
{
	Vec3D<float> returnForce(ext->force());
	if (ext->isFixed(X_TRANSLATE) || ext->isFixed(Y_TRANSLATE) || ext->isFixed(Z_TRANSLATE)){
		Vec3D<float> thisForce = (Vec3D<float>) -force();
		if (ext->isFixed(X_TRANSLATE)) returnForce.x = thisForce.x;
		if (ext->isFixed(Y_TRANSLATE)) returnForce.y = thisForce.y;
		if (ext->isFixed(Z_TRANSLATE)) returnForce.z = thisForce.z;
	}
	return returnForce;
}

Vec3D<float> CVX_Voxel::externalMoment()
{
	Vec3D<float> returnMoment(ext->moment());
	if (ext->isFixed(X_ROTATE) || ext->isFixed(Y_ROTATE) || ext->isFixed(Z_ROTATE)){
		Vec3D<float> thisMoment = (Vec3D<float>) -moment();
		if (ext->isFixed(X_ROTATE)) returnMoment.x = thisMoment.x;
		if (ext->isFixed(Y_ROTATE)) returnMoment.y = thisMoment.y;
		if (ext->isFixed(Z_ROTATE)) returnMoment.z = thisMoment.z;
	}
	return returnMoment;
}

Vec3D<float> CVX_Voxel::cornerPosition(voxelCorner corner) const
{
	return (Vec3D<float>)pos + orient.RotateVec3D(cornerOffset(corner));
}

Vec3D<float> CVX_Voxel::cornerOffset(voxelCorner corner) const
{
	Vec3D<> strains;
	for (int i=0; i<3; i++){
		bool posLink = corner&(1<<(2-i))?true:false;
		CVX_Link* pL = links[2*i + (posLink?0:1)];
		if (pL && !pL->isFailed()){
			strains[i] = (1 + pL->axialStrain(posLink))*(posLink?1:-1);
		}
		else strains[i] = posLink?1.0:-1.0;
	}

	return (0.5*baseSize()).Scale(strains);
}

//http://klas-physics.googlecode.com/svn/trunk/src/general/Integrator.cpp (reference)
void CVX_Voxel::timeStep(float dt)
{
	previousDt = dt;
	if (dt == 0.0f) return;

	if (ext && ext->isFixedAll()){
		pos = originalPosition() + ext->translation();
		orient = ext->rotationQuat();
		haltMotion();
		return;
	}

	//Translation
	Vec3D<double> curForce = force();
	Vec3D<double> fricForce = curForce;

	if (isFloorEnabled()) floorForce(dt, &curForce); //floor force needs dt to calculate threshold to "stop" a slow voxel into static friction.

	fricForce = curForce - fricForce;

	assert(!(curForce.x != curForce.x) || !(curForce.y != curForce.y) || !(curForce.z != curForce.z)); //assert non QNAN
	linMom += curForce*dt;

	Vec3D<double> translate(linMom*(dt*mat->_massInverse)); //movement of the voxel this timestep

//	we need to check for friction conditions here (after calculating the translation) and stop things accordingly
	if (isFloorEnabled() && floorPenetration() >= 0){ //we must catch a slowing voxel here since it all boils down to needing access to the dt of this timestep.
		double work = fricForce.x*translate.x + fricForce.y*translate.y; //F dot disp
		double hKe = 0.5*mat->_massInverse*(linMom.x*linMom.x + linMom.y*linMom.y); //horizontal kinetic energy

		if(hKe + work <= 0) setFloorStaticFriction(true); //this checks for a change of direction according to the work-energy principle

		if (isFloorStaticFriction()){ //if we're in a state of static friction, zero out all horizontal motion
			linMom.x = linMom.y = 0;
			translate.x = translate.y = 0;
		}
	}
	else setFloorStaticFriction(false);


	pos += translate;

	//Rotation
	Vec3D<> curMoment = moment();
	angMom += curMoment*dt;

	orient = Quat3D<>(angMom*(dt*mat->_momentInertiaInverse))*orient; //update the orientation

	if (ext){
		double size = mat->nominalSize();
		if (ext->isFixed(X_TRANSLATE)) {pos.x = ix*size + ext->translation().x; linMom.x=0;}
		if (ext->isFixed(Y_TRANSLATE)) {pos.y = iy*size + ext->translation().y; linMom.y=0;}
		if (ext->isFixed(Z_TRANSLATE)) {pos.z = iz*size + ext->translation().z; linMom.z=0;}
		if (ext->isFixedAnyRotation()){ //if any rotation fixed, all are fixed
			if (ext->isFixedAllRotation()){
				orient = ext->rotationQuat();
				angMom = Vec3D<double>();
			}
			else { //partial fixes: slow!
				Vec3D<double> tmpRotVec = orient.ToRotationVector();
				if (ext->isFixed(X_ROTATE)){ tmpRotVec.x=0; angMom.x=0;}
				if (ext->isFixed(Y_ROTATE)){ tmpRotVec.y=0; angMom.y=0;}
				if (ext->isFixed(Z_ROTATE)){ tmpRotVec.z=0; angMom.z=0;}
				orient.FromRotationVector(tmpRotVec);
			}
		}
	}


	poissonsStrainInvalid = true;
}

Vec3D<double> CVX_Voxel::force()
{
	//forces from internal bonds
	Vec3D<double> totalForce(0,0,0);
	for (int i=0; i<6; i++){ 
		if (links[i]) totalForce += links[i]->force(isNegative((linkDirection)i)); //total force in LCS
	}
	totalForce = orient.RotateVec3D(totalForce); //from local to global coordinates
	assert(!(totalForce.x != totalForce.x) || !(totalForce.y != totalForce.y) || !(totalForce.z != totalForce.z)); //assert non QNAN

	//other forces
	if (externalExists()) totalForce += external()->force(); //external forces
	totalForce -= velocity()*mat->globalDampingTranslateC(); //global damping f-cv
	totalForce.z += mat->gravityForce(); //gravity, according to f=mg

	if (isCollisionsEnabled()){
		for (std::vector<CVX_Collision*>::iterator it=colWatch->begin(); it!=colWatch->end(); it++){
			totalForce -= (*it)->contactForce(this);
		}
	}

	return totalForce;
}

Vec3D<double> CVX_Voxel::moment()
{
	//moments from internal bonds
	Vec3D<double> totalMoment(0,0,0);
	for (int i=0; i<6; i++){ 
		if (links[i]) totalMoment += links[i]->moment(isNegative((linkDirection)i)); //total force in LCS
	}
	totalMoment = orient.RotateVec3D(totalMoment);
	
	//other moments
	if (externalExists()) totalMoment += external()->moment(); //external moments
	totalMoment -= angularVelocity()*mat->globalDampingRotateC(); //global damping
	return totalMoment;
}


void CVX_Voxel::floorForce(float dt, Vec3D<double>* pTotalForce)
{
	float CurPenetration = floorPenetration(); //for now use the average.

	if (CurPenetration>=0){ 
		Vec3D<double> vel = velocity();
		Vec3D<double> horizontalVel(vel.x, vel.y, 0);
		
		float normalForce = mat->penetrationStiffness()*CurPenetration;
		pTotalForce->z += normalForce - mat->collisionDampingTranslateC()*vel.z; //in the z direction: k*x-C*v - spring and damping

		if (isFloorStaticFriction()){ //If this voxel is currently in static friction mode (no lateral motion) 
			assert(horizontalVel.Length2() == 0);
			float surfaceForceSq = (float)(pTotalForce->x*pTotalForce->x + pTotalForce->y*pTotalForce->y); //use squares to avoid a square root
			float frictionForceSq = (mat->muStatic*normalForce)*(mat->muStatic*normalForce);
		
			if (surfaceForceSq > frictionForceSq) setFloorStaticFriction(false); //if we're breaking static friction, leave the forces as they currently have been calculated to initiate motion this time step
		}
		else { //even if we just transitioned don't process here or else with a complete lack of momentum it'll just go back to static friction
			*pTotalForce -=  mat->muKinetic*normalForce*horizontalVel.Normalized(); //add a friction force opposing velocity according to the normal force and the kinetic coefficient of friction
		}
	}
	else setFloorStaticFriction(false);

}

Vec3D<float> CVX_Voxel::strain(bool poissonsStrain) const
{
	//if no connections in the positive and negative directions of a particular axis, strain is zero
	//if one connection in positive or negative direction of a particular axis, strain is that strain - ?? and force or constraint?
	//if connections in both the positive and negative directions of a particular axis, strain is the average. 
	
	Vec3D<float> intStrRet(0,0,0); //intermediate strain return value. axes according to linkAxis enum
	int numBondAxis[3] = {0}; //number of bonds in this axis (0,1,2). axes according to linkAxis enum
	bool tension[3] = {false};
	for (int i=0; i<6; i++){ //cycle through link directions
		if (links[i]){
			int axis = toAxis((linkDirection)i);
			intStrRet[axis] += links[i]->axialStrain(isNegative((linkDirection)i));
			numBondAxis[axis]++;
		}
	}
	for (int i=0; i<3; i++){ //cycle through axes
		if (numBondAxis[i]==2) intStrRet[i] *= 0.5f; //average
		if (poissonsStrain){
			tension[i] = ((numBondAxis[i]==2) || (ext && (numBondAxis[i]==1 && (ext->isFixed((dofComponent)(1<<i)) || ext->force()[i] != 0)))); //if both sides pulling, or just one side and a fixed or forced voxel...
		}

	}

	if (poissonsStrain){
		if (!(tension[0] && tension[1] && tension[2])){ //if at least one isn't in tension
			float add = 0;
			for (int i=0; i<3; i++) if (tension[i]) add+=intStrRet[i];
			float value = pow( 1.0f + add, -mat->poissonsRatio()) - 1.0f;
			for (int i=0; i<3; i++) if (!tension[i]) intStrRet[i]=value;
		}
	}

	return intStrRet;
}

Vec3D<float> CVX_Voxel::poissonsStrain()
{
	if (poissonsStrainInvalid){
		pStrain = strain(true);
		poissonsStrainInvalid = false;
	}
	return pStrain;
}


float CVX_Voxel::transverseStrainSum(CVX_Link::linkAxis axis)
{
	if (mat->poissonsRatio() == 0) return 0;
	
	Vec3D<float> psVec = poissonsStrain();

	switch (axis){
	case CVX_Link::X_AXIS: return psVec.y+psVec.z;
	case CVX_Link::Y_AXIS: return psVec.x+psVec.z;
	case CVX_Link::Z_AXIS: return psVec.x+psVec.y;
	default: return 0.0f;
	}

}

float CVX_Voxel::transverseArea(CVX_Link::linkAxis axis)
{
	float size = (float)mat->nominalSize();
	if (mat->poissonsRatio() == 0) return size*size;

	Vec3D<> psVec = poissonsStrain();

	switch (axis){
	case CVX_Link::X_AXIS: return (float)(size*size*(1+psVec.y)*(1+psVec.z));
	case CVX_Link::Y_AXIS: return (float)(size*size*(1+psVec.x)*(1+psVec.z));
	case CVX_Link::Z_AXIS: return (float)(size*size*(1+psVec.x)*(1+psVec.y));
	default: return size*size;
	}
}

void CVX_Voxel::updateSurface()
{
	bool interior = true;
	for (int i=0; i<6; i++) if (!links[i]) interior = false;
	interior ? boolStates |= SURFACE : boolStates &= ~SURFACE;
}


void CVX_Voxel::enableCollisions(bool enabled, float watchRadius) {
	if (enabled){
		if (!lastColWatchPosition) lastColWatchPosition = new Vec3D<float>;
		if (!colWatch) colWatch = new std::vector<CVX_Collision*>;
		if (!nearby) nearby = new std::vector<CVX_Voxel*>;
	}

	enabled ? boolStates |= COLLISIONS_ENABLED : boolStates &= ~COLLISIONS_ENABLED;
}


void CVX_Voxel::generateNearby(int linkDepth, bool surfaceOnly){
	std::vector<CVX_Voxel*> allNearby;
	allNearby.push_back(this);
	
	int iCurrent = 0;
	for (int k=0; k<linkDepth; k++){
		int iPassEnd = allNearby.size();

		while (iCurrent != iPassEnd){
			CVX_Voxel* pV = allNearby[iCurrent++];
		
			for (int i=0; i<6; i++){
				CVX_Voxel* pV2 = pV->adjacentVoxel((linkDirection)i);
				if (pV2 && std::find(allNearby.begin(), allNearby.end(), pV2) == allNearby.end()) allNearby.push_back(pV2);
			}
		}
	}

	nearby->clear();
	for (std::vector<CVX_Voxel*>::iterator it = allNearby.begin(); it != allNearby.end(); it++){
		CVX_Voxel* pV = (*it);
		if (pV->isSurface() && pV != this) nearby->push_back(pV);		
	}
}
