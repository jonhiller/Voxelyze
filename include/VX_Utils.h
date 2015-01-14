/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_UTILS_H
#define VX_UTILS_H

#include "Voxelyze.h"

//static methods and utilites for Voxelyze

void blurMaterials(CVoxelyze* pVx, Vec3D<float> mixRadius){ //blurs the specified voxel object according to the specified radii
	if (mixRadius == Vec3D<float>(0,0,0)) return; //no mixing needed
	
	double voxSize = pVx->voxelSize();
	int xLook = mixRadius.x==0 ? 0 : mixRadius.x/voxSize+1;
	int yLook = mixRadius.y==0 ? 0 : mixRadius.y/voxSize+1;
	int zLook = mixRadius.z==0 ? 0 : mixRadius.z/voxSize+1; //number of voxels away to look
	Vec3D<float> mixRadVoxInvSq(mixRadius.x==0?0:voxSize*voxSize/(mixRadius.x*mixRadius.x), mixRadius.y==0?0:voxSize*voxSize/(mixRadius.y*mixRadius.y), mixRadius.z==0?0:voxSize*voxSize/(mixRadius.z*mixRadius.z)); //in voxel units

	CVoxelyze* pRef = new CVoxelyze(*pVx); //make a copy
	pVx->clear(); //clear to add voxels back in

	for (int i=0; i<pRef->voxelCount(); i++){ //for each voxel
		CVX_Voxel* pV = pRef->voxel(i);
		int x=pV->indexX(), y=pV->indexY(), z=pV->indexZ();
		double accYoungsMod = 0, accDensity = 0, totalWeight = 0; //accumulators and the total "weight" (i.e. how much to divide he accumulators by to get the weighted average)
		double accR=0, accG=0, accB=0; //color channels

		for (int ix = x-xLook; ix<=x+xLook; ix++){
			for (int jy = y-yLook; jy<=y+yLook; jy++){
				for (int kz = z-zLook; kz<=z+zLook; kz++){
					CVX_Voxel* pV2 = pRef->voxel(ix, jy, kz);
					if (pV2){ //if theres a voxel here:
						double thisWeight = 0;
						if (ix == x && jy==y && kz == z){
							thisWeight = 1; // std::max(1+voxSize/amr.x, std::max(1+voxSize/amr.y, 1+voxSize/amr.z));
						}
						else {
							Vec3D<double> thisDistVec = Vec3D<double>(ix-x, jy-y, kz-z).Abs(); //in voxel units
							Vec3D<double> minDistVec(thisDistVec.x>1?thisDistVec.x-1:0, thisDistVec.y>1?thisDistVec.y-1:0, thisDistVec.z>1?thisDistVec.z-1:0);

							double SumMin = 0;
							SumMin += minDistVec.x*minDistVec.x*mixRadVoxInvSq.x;
							SumMin += minDistVec.y*minDistVec.y*mixRadVoxInvSq.y;
							SumMin += minDistVec.z*minDistVec.z*mixRadVoxInvSq.z;

							double SumMax = 0;
							SumMax += thisDistVec.x*thisDistVec.x*mixRadVoxInvSq.x;
							SumMax += thisDistVec.y*thisDistVec.y*mixRadVoxInvSq.y;
							SumMax += thisDistVec.z*thisDistVec.z*mixRadVoxInvSq.z;

							if ((SumMin==0 && SumMax==0) || SumMin > 1) thisWeight = 0;
							else if (SumMax > 1) thisWeight = (1-SumMin)/(2*(SumMax-SumMin)); //just the area between 0 and 1.
							else thisWeight = 1-(SumMax+SumMin)/2; //both less than (or equal) to 1 - average!

						}
	
						//straight weighted average (regardless of stiffness)
						CVX_Material* pMat2 = pV2->material();
						accYoungsMod += thisWeight*pMat2->youngsModulus();
						accDensity += thisWeight*pMat2->density();
						accR += thisWeight*pMat2->red();
						accG += thisWeight*pMat2->green();
						accB += thisWeight*pMat2->blue();
						totalWeight += thisWeight;

					}
				}
			}
		}

		accYoungsMod /= totalWeight;
		accDensity /= totalWeight;
		accR /= totalWeight;
		accG /= totalWeight;
		accB /= totalWeight;

		CVX_Material* pThisMaterial = NULL;
		for (int j=0; j<pVx->materialCount(); j++){ //if existing material, find it.
			if (pVx->material(j)->youngsModulus() == (float)accYoungsMod && pVx->material(j)->density() == (float)accDensity){
				pThisMaterial = pVx->material(j);
				break;
			}
		}
		if (!pThisMaterial){
			pThisMaterial = pVx->addMaterial((float)accYoungsMod, (float)accDensity); //add material if equivalent does not yet exist
			pThisMaterial->setColor((int)accR, (int)accG, (int)accB);
		}

		CVX_Voxel* thisVox = pVx->setVoxel(pThisMaterial, x, y, z);

		if (pRef->voxel(x,y,z)->externalExists()){ //copy over any externals
			*thisVox->external() = *pRef->voxel(x,y,z)->external();
		}
	}
}

#endif //VX_UTILS_H