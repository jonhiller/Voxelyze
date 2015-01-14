/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_Collision.h"
#include "VX_Voxel.h"

float CVX_Collision::envelopeRadius = 0.625f;

CVX_Collision::CVX_Collision(CVX_Voxel* v1, CVX_Voxel* v2)
{
	pV1 = v1;
	pV2 = v2;
	penetrationStiff = 2.0f/(1.0f/v1->material()->penetrationStiffness()+1.0f/pV2->material()->penetrationStiffness());
	dampingC = 0.5f*(v1->material()->collisionDampingTranslateC() + v2->material()->collisionDampingTranslateC()); //average
}

CVX_Collision& CVX_Collision::operator=(const CVX_Collision& col)
{
	pV1 = col.pV1;
	pV2 = col.pV2;
	penetrationStiff = col.penetrationStiff;
	force = col.force;
	return *this;
}

Vec3D<float> const CVX_Collision::contactForce(CVX_Voxel* pVoxel)
{
	if (pVoxel == pV1) return force;
	else if (pVoxel == pV2) return -force;
	else return Vec3D<float>(0,0,0);
}


void CVX_Collision::updateContactForce() 
{
	//just basic sphere envelope, repel with the stiffness of the material... (assumes UpdateConstants has been called)
	Vec3D<float> offset = (Vec3D<float>)(pV2->position() - pV1->position());
	float NomDist = (float)((pV1->baseSizeAverage() + pV2->baseSizeAverage())*envelopeRadius); //effective diameter of 1.5 voxels... (todo: remove length2!!
	float RelDist = NomDist -offset.Length(); //negative for overlap!

	if (RelDist > 0){
		Vec3D<float> unit = offset.Normalized(); //unit vector from voxel 1 in the direction of voxel 2
		float relativeVelocity = pV1->velocity().Dot((Vec3D<double>)unit) - pV2->velocity().Dot((Vec3D<double>)unit); //negative is moving towards each other

		force = unit * (penetrationStiff * RelDist + dampingC*relativeVelocity); //kx + cV - if we're overlapping
	}
	else force = Vec3D<float>(0,0,0);
}
