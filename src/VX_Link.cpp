/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_Link.h"
#include "VX_Voxel.h"
#include "VX_MaterialLink.h"

#include <assert.h>
#ifdef DEBUG
#include <iostream>
#endif

static const float HYSTERESIS_FACTOR = 1.2f; //Amount for small angle bond calculations
static const float SA_BOND_BEND_RAD = 0.05f; //Amount for small angle bond calculations
static const float SA_BOND_EXT_PERC = 0.50f; //Amount for small angle bond calculations


CVX_Link::CVX_Link(CVX_Voxel* voxel1, CVX_Voxel* voxel2, CVX_MaterialLink* material/*, linkDirection direction*/)
{
	assert(voxel1 != NULL);
	assert(voxel2 != NULL);

	bool reverseOrder = false;
	if (voxel1->indexX() == voxel2->indexX() && voxel1->indexY() == voxel2->indexY()){
		if (voxel1->indexZ() == voxel2->indexZ()+1) reverseOrder = true;
		else if (voxel1->indexZ()+1 == voxel2->indexZ()) reverseOrder = false;
		else return; //non-adjacent
		axis = Z_AXIS;
	}
	else if (voxel1->indexX() == voxel2->indexX() && voxel1->indexZ() == voxel2->indexZ()){
		if (voxel1->indexY() == voxel2->indexY()+1) reverseOrder = true;
		else if (voxel1->indexY()+1 == voxel2->indexY()) reverseOrder = false;
		else return; //non-adjacent
		axis = Y_AXIS;
	}
	else if (voxel1->indexY() == voxel2->indexY() && voxel1->indexZ() == voxel2->indexZ()){
		if (voxel1->indexX() == voxel2->indexX()+1) reverseOrder = true;
		else if (voxel1->indexX()+1 == voxel2->indexX()) reverseOrder = false;
		else return; //non-adjacent
		axis = X_AXIS;
	}
	else return; //non-adjacent

	if (reverseOrder){ pVNeg=voxel2; pVPos=voxel1; }
	else { pVNeg=voxel1; pVPos=voxel2; }

	mat=material;

	boolStates=0;
	reset();
}

void CVX_Link::reset()
{
	pos2 = angle1v = angle2v = Vec3D<double>();
	angle1 = angle2 = Quat3D<double>();
	forceNeg = forcePos = momentNeg = momentPos = Vec3D<>();
	strain = maxStrain = strainOffset = _stress = 0.0f;
	strainRatio = pVPos->material()->E/pVNeg->material()->E;
	smallAngle = true;

	setBoolState(LOCAL_VELOCITY_VALID, false);

	updateRestLength();
	updateTransverseInfo();

}

Quat3D<double> CVX_Link::orientLink(/*double restLength*/) //updates pos2, angle1, angle2, and smallAngle
{
	pos2 = toAxisX(Vec3D<double>(pVPos->position() - pVNeg->position())); //digit truncation happens here...

	angle1 = toAxisX(pVNeg->orientation());
	angle2 = toAxisX(pVPos->orientation());

	Quat3D<double> totalRot = angle1.Conjugate(); //keep track of the total rotation of this bond (after toAxisX())
	pos2 = totalRot.RotateVec3D(pos2);
	angle2 = totalRot*angle2;
	angle1 = Quat3D<>(); //zero for now...

	//small angle approximation?
	float SmallTurn = (float)((abs(pos2.z)+abs(pos2.y))/pos2.x);
	float ExtendPerc = (float)(abs(1-pos2.x/currentRestLength));
	if (!smallAngle /*&& angle2.IsSmallAngle()*/ && SmallTurn < SA_BOND_BEND_RAD && ExtendPerc < SA_BOND_EXT_PERC){
		smallAngle = true;
		setBoolState(LOCAL_VELOCITY_VALID, false);
	}
	else if (smallAngle && (/*!angle2.IsSmallishAngle() || */SmallTurn > HYSTERESIS_FACTOR*SA_BOND_BEND_RAD || ExtendPerc > HYSTERESIS_FACTOR*SA_BOND_EXT_PERC)){
		smallAngle = false;
		setBoolState(LOCAL_VELOCITY_VALID, false);
	}

	if (smallAngle)	{ //Align so Angle1 is all zeros
		pos2.x -= currentRestLength; //only valid for small angles
	}
	else { //Large angle. Align so that Pos2.y, Pos2.z are zero.
		angle1.FromAngleToPosX(pos2); //get the angle to align Pos2 with the X axis
		totalRot = angle1 * totalRot; //update our total rotation to reflect this
		angle2 = angle1 * angle2; //rotate angle2
		pos2 = Vec3D<>(pos2.Length() - currentRestLength, 0, 0); 
	}

	angle1v = angle1.ToRotationVector();
	angle2v = angle2.ToRotationVector();

	assert(!(angle1v.x != angle1v.x) || !(angle1v.y != angle1v.y) || !(angle1v.z != angle1v.z)); //assert non QNAN
	assert(!(angle2v.x != angle2v.x) || !(angle2v.y != angle2v.y) || !(angle2v.z != angle2v.z)); //assert non QNAN


	return totalRot;
}

float CVX_Link::axialStrain(bool positiveEnd) const
{
	return positiveEnd ? 2.0f*strain*strainRatio/(1.0f+strainRatio) : 2.0f*strain/(1.0f+strainRatio);
}


bool CVX_Link::isYielded() const
{
	return mat->isYielded(maxStrain);
}

bool CVX_Link::isFailed() const
{
	return mat->isFailed(maxStrain);
}

void CVX_Link::updateRestLength()
{
	currentRestLength = 0.5*(pVNeg->baseSize(axis) + pVPos->baseSize(axis));
}

void CVX_Link::updateTransverseInfo()
{
	currentTransverseArea = 0.5f*(pVNeg->transverseArea(axis)+pVPos->transverseArea(axis));
	currentTransverseStrainSum = 0.5f*(pVNeg->transverseStrainSum(axis)+pVPos->transverseStrainSum(axis));

}

void CVX_Link::updateForces()
{
	Vec3D<double> oldPos2 = pos2, oldAngle1v = angle1v, oldAngle2v = angle2v; //remember the positions/angles from last timestep to calculate velocity

	orientLink(/*restLength*/); //sets pos2, angle1, angle2

	Vec3D<double> dPos2 = 0.5*(pos2-oldPos2); //deltas for local damping. velocity at center is half the total velocity
	Vec3D<double> dAngle1 = 0.5*(angle1v-oldAngle1v);
	Vec3D<double> dAngle2 = 0.5*(angle2v-oldAngle2v);
	
	//if volume effects...
	if (!mat->isXyzIndependent() || currentTransverseStrainSum != 0) updateTransverseInfo(); //currentTransverseStrainSum != 0 catches when we disable poissons mid-simulation

	_stress = updateStrain((float)(pos2.x/currentRestLength));
	if (isFailed()){forceNeg = forcePos = momentNeg = momentPos = Vec3D<double>(0,0,0); return;}

	float b1=mat->_b1, b2=mat->_b2, b3=mat->_b3, a2=mat->_a2; //local copies
	//Beam equations. All relevant terms are here, even though some are zero for small angle and others are zero for large angle (profiled as negligible performance penalty)
	forceNeg = Vec3D<double> (	_stress*currentTransverseArea, //currentA1*pos2.x,
								b1*pos2.y - b2*(angle1v.z + angle2v.z),
								b1*pos2.z + b2*(angle1v.y + angle2v.y)); //Use Curstress instead of -a1*Pos2.x to account for non-linear deformation 
	forcePos = -forceNeg;

	momentNeg = Vec3D<double> (	a2*(angle2v.x - angle1v.x),
								-b2*pos2.z - b3*(2*angle1v.y + angle2v.y),
								b2*pos2.y - b3*(2*angle1v.z + angle2v.z));
	momentPos = Vec3D<double> (	a2*(angle1v.x - angle2v.x),
								-b2*pos2.z - b3*(angle1v.y + 2*angle2v.y),
								b2*pos2.y - b3*(angle1v.z + 2*angle2v.z));


	//local damping:
	if (isLocalVelocityValid()){ //if we don't have the basis for a good damping calculation, don't do any damping.
		float sqA1=mat->_sqA1, sqA2xIp=mat->_sqA2xIp,sqB1=mat->_sqB1, sqB2xFMp=mat->_sqB2xFMp, sqB3xIp=mat->_sqB3xIp;
		Vec3D<double> posCalc(	sqA1*dPos2.x,
								sqB1*dPos2.y - sqB2xFMp*(dAngle1.z+dAngle2.z),
								sqB1*dPos2.z + sqB2xFMp*(dAngle1.y+dAngle2.y));

		forceNeg += pVNeg->dampingMultiplier()*posCalc;
		forcePos -= pVPos->dampingMultiplier()*posCalc;

		momentNeg -= 0.5*pVNeg->dampingMultiplier()*Vec3D<>(	-sqA2xIp*(dAngle2.x - dAngle1.x),
																sqB2xFMp*dPos2.z + sqB3xIp*(2*dAngle1.y + dAngle2.y),
																-sqB2xFMp*dPos2.y + sqB3xIp*(2*dAngle1.z + dAngle2.z));
		momentPos -= 0.5*pVPos->dampingMultiplier()*Vec3D<>(	sqA2xIp*(dAngle2.x - dAngle1.x),
																sqB2xFMp*dPos2.z + sqB3xIp*(dAngle1.y + 2*dAngle2.y),
																-sqB2xFMp*dPos2.y + sqB3xIp*(dAngle1.z + 2*dAngle2.z));

	}
	else setBoolState(LOCAL_VELOCITY_VALID, true); //we're good for next go-around unless something changes

	//	transform forces and moments to local voxel coordinates
	if (!smallAngle){
		forceNeg = angle1.RotateVec3DInv(forceNeg);
		momentNeg = angle1.RotateVec3DInv(momentNeg);
	}
	forcePos = angle2.RotateVec3DInv(forcePos);
	momentPos = angle2.RotateVec3DInv(momentPos);

	toAxisOriginal(&forceNeg);
	toAxisOriginal(&forcePos);
	toAxisOriginal(&momentNeg);
	toAxisOriginal(&momentPos);

	assert(!(forceNeg.x != forceNeg.x) || !(forceNeg.y != forceNeg.y) || !(forceNeg.z != forceNeg.z)); //assert non QNAN
	assert(!(forcePos.x != forcePos.x) || !(forcePos.y != forcePos.y) || !(forcePos.z != forcePos.z)); //assert non QNAN


}


float CVX_Link::updateStrain(float axialStrain)
{
	strain = axialStrain; //redundant?

	if (mat->linear){
		if (axialStrain > maxStrain) maxStrain = axialStrain; //remember this maximum for easy reference
		return mat->stress(axialStrain, currentTransverseStrainSum);
	}
	else {
		float returnStress;

		if (axialStrain > maxStrain){ //if new territory on the stress/strain curve
			maxStrain = axialStrain; //remember this maximum for easy reference
			returnStress = mat->stress(axialStrain, currentTransverseStrainSum);
			
			if (mat->nu != 0.0f) strainOffset = maxStrain-mat->stress(axialStrain)/(mat->_eHat*(1-mat->nu)); //precalculate strain offset for when we back off
			else strainOffset = maxStrain-returnStress/mat->E; //precalculate strain offset for when we back off

		}
		else { //backed off a non-linear material, therefore in linear region.
			float relativeStrain = axialStrain-strainOffset; // treat the material as linear with a strain offset according to the maximum plastic deformation
			
			if (mat->nu != 0.0f) returnStress = mat->stress(relativeStrain, currentTransverseStrainSum, true);
			else returnStress = mat->E*relativeStrain;
		}
		return returnStress;

	}

}

float CVX_Link::strainEnergy() const
{
	return	forceNeg.x*forceNeg.x/(2.0f*mat->_a1) + //Tensile strain
			momentNeg.x*momentNeg.x/(2.0*mat->_a2) + //Torsion strain
			(momentNeg.z*momentNeg.z - momentNeg.z*momentPos.z +momentPos.z*momentPos.z)/(3.0*mat->_b3) + //Bending Z
			(momentNeg.y*momentNeg.y - momentNeg.y*momentPos.y +momentPos.y*momentPos.y)/(3.0*mat->_b3); //Bending Y
}

float CVX_Link::axialStiffness() {
	if (mat->isXyzIndependent()) return mat->_a1;
	else {
		updateRestLength();
		updateTransverseInfo();

		return (float)(mat->_eHat*currentTransverseArea/((strain+1)*currentRestLength)); // _a1;
	}
} 

float CVX_Link::a1() const {return mat->_a1;}
float CVX_Link::a2() const {return mat->_a2;}
float CVX_Link::b1() const {return mat->_b1;}
float CVX_Link::b2() const {return mat->_b2;}
float CVX_Link::b3() const {return mat->_b3;}

