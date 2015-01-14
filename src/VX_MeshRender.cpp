/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_MeshRender.h"
#include "VX_Voxel.h"

//for file output
#include <iostream>
#include <fstream>

#ifdef USE_OPEN_GL
	#ifdef QT_GUI_LIB
		#include <qgl.h>
	#else
		#ifdef __APPLE__
			#include <OpenGL/gl.h>
			#include <OpenGL/glu.h>
		#else
			#include <GL/gl.h>
			#include <GL/glu.h>
		#endif
	#endif
#endif

//link direction to clockwise vertex lookup info:
CVX_Voxel::voxelCorner CwLookup[6][4] = {
	{CVX_Voxel::PNN, CVX_Voxel::PPN, CVX_Voxel::PPP, CVX_Voxel::PNP}, //linkDirection::X_POS
	{CVX_Voxel::NNN, CVX_Voxel::NNP, CVX_Voxel::NPP, CVX_Voxel::NPN}, //linkDirection::X_NEG
	{CVX_Voxel::NPN, CVX_Voxel::NPP, CVX_Voxel::PPP, CVX_Voxel::PPN}, //linkDirection::Y_POS
	{CVX_Voxel::NNN, CVX_Voxel::PNN, CVX_Voxel::PNP, CVX_Voxel::NNP}, //linkDirection::Y_NEG
	{CVX_Voxel::NNP, CVX_Voxel::PNP, CVX_Voxel::PPP, CVX_Voxel::NPP}, //linkDirection::Z_POS
	{CVX_Voxel::NNN, CVX_Voxel::NPN, CVX_Voxel::PPN, CVX_Voxel::PNN}  //linkDirection::Z_NEG
};

CVX_MeshRender::CVX_MeshRender(CVoxelyze* voxelyzeInstance)
{
	vx = voxelyzeInstance;
	generateMesh();
}

void CVX_MeshRender::generateMesh()
{
	vertices.clear();
	vertexLinks.clear();

	tris.clear();
	triColors.clear();
	triVoxIndices.clear();
	triNormals.clear();

	lines.clear();

	int minX = vx->indexMinX();
	int sizeX = vx->indexMaxX()-minX+1;
	int minY = vx->indexMinY();
	int sizeY = vx->indexMaxY()-minY+1;
	int minZ = vx->indexMinZ();
	int sizeZ = vx->indexMaxZ()-minZ+1;

	CArray3D<int> vIndMap; //maps a 3d location to a vertex index
	vIndMap.setDefaultValue(-1); //-1 indicates no vertex created (yet)
	vIndMap.resize(sizeX+1, sizeY+1, sizeZ+1, minX, minY, minZ);
	int vertexCounter = 0;
	
	//for each possible voxel location: (fill in vertices)
	int vCount = vx->voxelCount();
	for (int k=0; k<vCount; k++){
		CVX_Voxel* pV = vx->voxel(k);
		int x=pV->indexX(), y=pV->indexY(), z=pV->indexZ();

		Index3D thisVox(x, y, z);
		for (int i=0; i<6; i++){ //for each direction that a quad face could exist
			if (pV->adjacentVoxel((CVX_Voxel::linkDirection)i)) continue;
			int theseInds[4]; //ccw order of voxel endices for this quad
			for (int j=0; j<4; j++){ //for each corner of the (exposed) face in this direction
				CVX_Voxel::voxelCorner thisCorner = CwLookup[i][j];
				Index3D thisVertInd3D = thisVox + Index3D(thisCorner&(1<<2)?1:0, thisCorner&(1<<1)?1:0, thisCorner&(1<<0)?1:0);
				int thisInd = vIndMap[thisVertInd3D];

				//if this vertec needs to be added, do it now!
				if (thisInd == -1){ 
					vIndMap[thisVertInd3D] = thisInd = vertexCounter++;
					for (int i=0; i<3; i++) vertices.push_back(0); //will be set on first updateMesh()
				}

				theseInds[j] = thisInd;

				//quads.push_back(thisInd); //add this vertices' contribution to the quad
			}
			//triangle 1
			tris.push_back(theseInds[0]);
			tris.push_back(theseInds[1]);
			tris.push_back(theseInds[2]);
			triVoxIndices.push_back(k);

			//triangle 2
			tris.push_back(theseInds[0]);
			tris.push_back(theseInds[2]);
			tris.push_back(theseInds[3]);
			triVoxIndices.push_back(k);
		}
	}

	//vertex links: do here to make it the right size all at once and avoid lots of expensive allocations
	vertexLinks.resize(vertexCounter*8, NULL);
	for (int z=minZ; z<minZ+sizeZ+1; z++){ //for each in vIndMap, now.
		for (int y=minY; y<minY+sizeY+1; y++){
			for (int x=minX; x<minX+sizeX+1; x++){
				int thisInd = vIndMap[Index3D(x,y,z)];
				if (thisInd == -1) continue;

				//backwards links
				for (int i=0; i<8; i++){ //check all 8 possible voxels that could be connected...
					CVX_Voxel* pV = vx->voxel(x-(i&(1<<2)?1:0), y-(i&(1<<1)?1:0), z-(i&(1<<0)?1:0));
					if (pV) vertexLinks[8*thisInd + i] = pV;
				}

				//lines
				for (int i=0; i<3; i++){ //look in positive x, y, and z directions
					int isX = (i==0?1:0), isY = (i==1?1:0), isZ = (i==2?1:0);
					int p2Ind = vIndMap[Index3D(x+isX, y+isY, z+isZ)];
					if (p2Ind != -1){ //for x: voxel(x,y,z) (x,y-1,z) (x,y-1,z-1) (x,y,z-1) -- y: voxel(x,y,z) (x-1,y,z) (x-1,y,z-1) (x,y,z-1) -- z: voxel(x,y,z) (x,y-1,z) (x-1,y-1,z) (x-1,y,z)
						if (vx->voxel(x,			y,			z) ||
							vx->voxel(x-isY,		y-isX-isZ,	z) ||
							vx->voxel(x-isY-isZ,	y-isX-isZ,	z-isX-isY) ||
							vx->voxel(x-isZ,		y,			z-isX-isY)) {
							
							lines.push_back(thisInd); lines.push_back(p2Ind);
						}
					}
				}
			}
		}
	}

	//the rest... allocate space, but updateMesh will fill them in.
	int triCount = tris.size()/3;
	triColors.resize(triCount*3);
	triNormals.resize(triCount*3);

	updateMesh();
}

//updates all the modal properties: offsets, quadColors, quadNormals.
void CVX_MeshRender::updateMesh(viewColoring colorScheme, CVoxelyze::stateInfoType stateType)
{
	//location
	int vCount = vertices.size()/3;
	if (vCount == 0) return;
	for (int i=0; i<vCount; i++){ //for each vertex...
		Vec3D<float> avgPos;
		int avgCount = 0;
		for (int j=0; j<8; j++){
			CVX_Voxel* pV = vertexLinks[8*i+j];
			if (pV){
				avgPos += pV->cornerPosition((CVX_Voxel::voxelCorner)j);
				avgCount++;
			}
		}
		avgPos /= avgCount;
		vertices[3*i] = avgPos.x;
		vertices[3*i+1] = avgPos.y;
		vertices[3*i+2] = avgPos.z;
	}

	//Find a maximum if necessary:
	float minVal = 0, maxVal = 0;
	if (colorScheme == STATE_INFO){
		maxVal = vx->stateInfo(stateType, CVoxelyze::MAX);
		minVal = vx->stateInfo(stateType, CVoxelyze::MIN);
		if (stateType == CVoxelyze::PRESSURE){ //pressure max and min are equal pos/neg
			maxVal = maxVal>-minVal ? maxVal : -minVal;
			minVal = -maxVal;
		}

	}

	//color + normals 
	int tCount = tris.size()/3;
	if (tCount == 0) return;
	for (int i=0; i<tCount; i++){
		Vec3D<float> v[3];
		for (int j=0; j<3; j++) v[j] = Vec3D<float>(vertices[3*tris[3*i+j]], vertices[3*tris[3*i+j]+1], vertices[3*tris[3*i+j]+2]);
		Vec3D<float> n = ((v[1]-v[0]).Cross(v[2]-v[0]));
		n.Normalize(); //necessary? try glEnable(GL_NORMALIZE)
		triNormals[i*3] = n.x;
		triNormals[i*3+1] = n.y;
		triNormals[i*3+2] = n.z;


		float r=1.0f, g=1.0f, b=1.0f;
		float jetValue = -1.0f;
		switch (colorScheme){
			case MATERIAL:
				r = ((float)vx->voxel(triVoxIndices[i])->material()->red())/255.0f;
				g = ((float)vx->voxel(triVoxIndices[i])->material()->green())/255.0f;
				b = ((float)vx->voxel(triVoxIndices[i])->material()->blue())/255.0f;
				break;
			case FAILURE:
				if (vx->voxel(triVoxIndices[i])->isFailed()){g=0.0f; b=0.0f;}
				else if (vx->voxel(triVoxIndices[i])->isYielded()){b=0.0f;}
				break;
			case STATE_INFO:
				switch (stateType) {
				case CVoxelyze::KINETIC_ENERGY: jetValue = vx->voxel(triVoxIndices[i])->kineticEnergy()/maxVal; break;
				case CVoxelyze::STRAIN_ENERGY: case CVoxelyze::ENG_STRAIN: case CVoxelyze::ENG_STRESS: jetValue = linkMaxColorValue(vx->voxel(triVoxIndices[i]), stateType) / maxVal; break;
				case CVoxelyze::DISPLACEMENT: jetValue = vx->voxel(triVoxIndices[i])->displacementMagnitude()/maxVal; break;
				case CVoxelyze::PRESSURE: jetValue = 0.5-vx->voxel(triVoxIndices[i])->pressure()/(2*maxVal); break;
				default: jetValue = 0;
				}
			break;
		}

		if (jetValue != -1.0f){
			r = jetMapR(jetValue);
			g = jetMapG(jetValue);
			b = jetMapB(jetValue);
		}

		triColors[i*3] = r;
		triColors[i*3+1] = g;
		triColors[i*3+2] = b;
	}
}
float CVX_MeshRender::linkMaxColorValue(CVX_Voxel* pV, CVoxelyze::stateInfoType coloring)
{
	float voxMax = -FLT_MAX;
	for (int i=0; i<6; i++){
		float thisVal = -FLT_MAX;
		CVX_Link* pL = pV->link((CVX_Voxel::linkDirection)i);
		if (pL){
			switch (coloring){
				case CVoxelyze::STRAIN_ENERGY: thisVal = pL->strainEnergy(); break;
				case CVoxelyze::ENG_STRESS: thisVal = pL->axialStress(); break;
				case CVoxelyze::ENG_STRAIN: thisVal = pL->axialStrain(); break;
				default: thisVal=0;
			}
		}
		
		if(thisVal>voxMax) voxMax=thisVal;
	}
	return voxMax;
}

void CVX_MeshRender::saveObj(const char* filePath)
{
	std::ofstream ofile(filePath);
	ofile << "# OBJ file generated by Voxelyze\n";
	for (int i=0; i<(int)(vertices.size()/3); i++){
		ofile << "v " << vertices[3*i] << " " << vertices[3*i+1] << " " << vertices[3*i+2] << "\n";
	}

	for (int i=0; i<(int)(tris.size()/4); i++){
		ofile << "f " << tris[3*i]+1 << " " << tris[3*i+1]+1 << " " << tris[3*i+2]+1 << "\n";
	}
	ofile.close();
}

void CVX_MeshRender::glDraw()
{
#ifdef USE_OPEN_GL

	//triangles
	int tCount = tris.size()/3;
	for (int i=0; i<tCount; i++) {
		glNormal3d(triNormals[i*3], triNormals[i*3+1], triNormals[i*3+2]);
		glColor3d(triColors[i*3], triColors[i*3+1], triColors[i*3+2]);
		glLoadName(triVoxIndices[i]); //to enable picking

		glBegin(GL_TRIANGLES);
		glVertex3d(vertices[3*tris[3*i]],   vertices[3*tris[3*i]+1],   vertices[3*tris[3*i]+2]);
		glVertex3d(vertices[3*tris[3*i+1]], vertices[3*tris[3*i+1]+1], vertices[3*tris[3*i+1]+2]);
		glVertex3d(vertices[3*tris[3*i+2]], vertices[3*tris[3*i+2]+1], vertices[3*tris[3*i+2]+2]);
		glEnd();

	}

	//lines
	glLineWidth(1.0);
	glBegin(GL_LINES);
	glColor3d(0, 0, 0); //black lines...

	int lCount = lines.size()/2;
	for (int i=0; i<lCount; i++) {
		glVertex3d(vertices[3*lines[2*i]], vertices[3*lines[2*i]+1], vertices[3*lines[2*i]+2]);
		glVertex3d(vertices[3*lines[2*i+1]], vertices[3*lines[2*i+1]+1], vertices[3*lines[2*i+1]+2]);
	}
	glEnd();


#endif
}

