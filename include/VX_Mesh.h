/*******************************************************************************
Copyright (c) 2010, Jonathan Hiller (Cornell University)
If used in publication cite "J. Hiller and H. Lipson "Dynamic Simulation of Soft Heterogeneous Objects" In press. (2011)"

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_MESH_H
#define VX_MESH_H

#include <vector>
#include "Voxelyze.h"

//Voxelyze mesh visualizer
class CVX_MeshRender
{
public:
	enum viewColor {MATERIAL, KINETIC_ENERGY, STRAIN_ENERGY, ENG_STRAIN, ENG_STRESS, DISPLACEMENT, STATE, PRESSURE};

	CVX_MeshRender(CVoxelyze* voxelyzeInstance);
	void generateMesh(); //generates from the linked voxelyze object. must be called whenever voxels change in the simulation
	void updateMesh(viewColor color = MATERIAL);

	void glDraw();

private:
	CVoxelyze* vx;

	std::vector<float> vertices; //vx1, vy1, vz1, vx2, vy2, vz2, vx3, ...
	std::vector<CVX_Voxel*> vertexLinks; //vx1NNN, vx1NNP, [CVX_Voxel::voxelCorner enum order], ... vx2NNN, vx2NNp, ... (null if no link) 
//	std::vector<float> offsets; //ox1, oy1, oz1, ox2, oy2, oz2, ox3, ... (for mesh deformation)

	std::vector<int> quads; //q1v1, q1v2, q1v3, q1v4, q2v1, q2v2, ... (ccw order)
	std::vector<float> quadColors; //q1R, q1G, q1B, q2R, q2G, q2B, ... 
//	std::vector<CVX_Voxel*> quadLinks; //q1v, q2v, q3v
	std::vector<int> quadVoxIndices; //q1n, q2n, q3n, ... 
	std::vector<float> quadNormals; //q1Nx, q1Ny, q1Nz, q2Nx, q2Ny, q2Nz, ... (needs updating with mesh deformation)

	std::vector<int> lines; //l1v1, l1v2, l2v1, l2v2, ...

	float jetMapR(float val) {if (val<0.5f) return 0.0f; else if (val>0.75f) return 1.0f; else return val*4-2;}
	float jetMapG(float val) {if (val<0.25f) return val*4; else if (val>0.75f) return 4-val*4; else return 1.0f;}
	float jetMapB(float val) {if (val>0.5f) return 0.0f; else if (val<0.25f) return 1.0f; else return 2-val*4;}

	float maxColorValue(viewColor color);
	float minColorValue(viewColor color);
	float linkMaxColorValue(CVX_Voxel* pV, viewColor color); //for link properties, the max
};
#endif
