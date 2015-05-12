/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include <string>
#include "Mesh3D.h"
#include "MeshPolygonize.h"
#include <unordered_map>

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

#define STL_LABEL_SIZE 80

CMesh3D::CMesh3D(const char* filePath)
{
	load(filePath);
}

CMesh3D::CMesh3D(CArray3D<float>& values, float threshold, float scale, float (*density)(Vec3D<float>&, Vec3D<float>*))
{
	meshFrom3dArrayMC(this, values, threshold, scale, density);
}

CMesh3D::CMesh3D(CArray3D<float>& values, CArray3D<Vec3D<float>>& normals, float threshold, float scale, float (*density)(Vec3D<float>&, Vec3D<float>*))
{
	meshFrom3dArrayDC(this, values, normals, threshold, scale, density);
}


void CMesh3D::clear()
{
	vertices.clear();
	vertexColors.clear();
	vertexNormals.clear();
	triangles.clear();
	triangleColors.clear();
	triangleNormals.clear();
	lines.clear();

	boundsMin = Vec3D<float>(0,0,0);
	boundsMax = Vec3D<float>(0,0,0);

	normalsByVertex = false;
	colorsByVertex = false;
	//...etc.

	meshChanged(); //take care of stale flags

}

void CMesh3D::meshChanged()
{
	vertexNormalsStale = true;
	vertexMergesStale = true;
	faceNormalsStale = true;
	boundsStale = true;
	slicerStale = true;

	TriLayer.clear();
	TriLine.clear();
	TriInts.clear();

	_lastZ = -FLT_MAX;
	_lastY = -FLT_MAX;
	_lastPad = 0;
}


bool CMesh3D::load(const char* filePath)
{
	std::string fPath(filePath);
	std::string fileExt = fPath.substr(fPath.find_last_of(".") + 1);
	if (fileExt == "stl" || fileExt == "STL" || fileExt == "Stl") return loadSTL(fPath);
	else return false; //unsupported file type
}

bool CMesh3D::save(const char* filePath)
{
	std::string fPath(filePath);

	if (faceNormalsStale) calcFaceNormals();

	std::string fileExt = fPath.substr(fPath.find_last_of(".") + 1);
	if (fileExt == "stl" || fileExt == "STL" || fileExt == "Stl") return saveSTL(fPath);
	if (fileExt == "obj" || fileExt == "OBJ" || fileExt == "Obj"){
		if (vertexNormalsStale) calcVertNormals();
		return saveOBJ(fPath);
	}
	else return false; //unsupported file type
}

int CMesh3D::addVertex(Vec3D<float>& location){
	for (int i=0; i<3; i++){
		vertices.push_back(location[i]);
		if (vertexColors.size() != 0) vertexColors.push_back(0);
	}
	vertexNormalsStale = true;

	return vertexCount()-1;
}

void CMesh3D::addTriangle(int vIndex1, int vIndex2, int vIndex3)
{
	triangles.push_back(vIndex1);
	triangles.push_back(vIndex2);
	triangles.push_back(vIndex3);
	if (triangleColors.size() != 0) for (int i=0; i<3; i++) vertexColors.push_back(0);

	meshChanged();
}


void CMesh3D::addTriangle(Vec3D<float>& p0, Vec3D<float>& p1, Vec3D<float>& p2)
{
	int vIndexStart = (int)(vertices.size());

	for (int i=0; i<3; i++) vertices.push_back(p0[i]);
	for (int i=0; i<3; i++) vertices.push_back(p1[i]);
	for (int i=0; i<3; i++) vertices.push_back(p2[i]);

	if (vertexColors.size() != 0) for (int i=0; i<9; i++) vertexColors.push_back(0);
	vertexNormalsStale = true;

	for (int i=0; i<3; i++) triangles.push_back(vIndexStart/3+i);
	if (triangleColors.size() != 0) for (int i=0; i<3; i++) vertexColors.push_back(0);

//	Vec3D<float> N = ((p1-p0).Cross(p2-p0)).Normalized();
//	for (int i=0; i<3; i++) triangleNormals.push_back(N[i]); //don't need faceNormalsStale!

	//stale out other stuff here...
	meshChanged();
//	boundsStale = true; //we could update here to ease update workload
//	faceNormalsStale = true; //we could update here to ease update workload
//	vertexMergesStale = true;
}

void CMesh3D::useFaceNormals()
{
	if (faceNormalsStale) calcFaceNormals();
	normalsByVertex = false;
}

void CMesh3D::useVertexNormals(float seamAngleDegrees)
{
	if (vertexNormalsStale || seamAngleDegrees != currentSeamAngle) calcVertNormals(seamAngleDegrees);
	normalsByVertex = true;
	currentSeamAngle = seamAngleDegrees;
}

void CMesh3D::calcFaceNormals() //called to update the face normals...
{
	int triCount = (int)(triangles.size()/3);
	triangleNormals.clear();
	triangleNormals.reserve(triCount*3);

	for (int i=0; i<triCount; i++){
		Vec3D<float> V0(&vertices[3*triangles[3*i]]), V1(&vertices[3*triangles[3*i+1]]), V2(&vertices[3*triangles[3*i+2]]);
		Vec3D<float> N = ((V1-V0).Cross(V2-V0)).Normalized();
		for (int j=0; j<3; j++) triangleNormals.push_back(N[j]);
	}

	faceNormalsStale = false;
}

//assumes triangle normals are accurate! (and that coincident vertices are merged)
void CMesh3D::splitVerticesByAngle(float seamAngleDegrees)
{
	float seamAngleRad = seamAngleDegrees*3.1415926/180.0f;
	useFaceNormals();

	std::vector<int> oldTriangles = triangles;

	int vCount = vertexCount();
	int tCount = triangleCount();

	int lTri[256], lVert[256], lTriGroup[256], lTriGroupToVertIndex[256]; //assume maximum 256 triangles fanning out from this vertex
	for (int i=0; i<vCount; i++){ //for each vertex (although, not duplicate ones that are added in process
		int lTriCount = 0;
		for (int j=0; j<tCount; j++){ //for each vertex
			//get the local triangles
			if (oldTriangles[3*j] == i || oldTriangles[3*j+1] == i || oldTriangles[3*j+2] == i) lTri[lTriCount++] = j;
		}

		for (int j=0; j<lTriCount; j++) lTriGroup[j] = j; //each in its own smoothing group to begin

		for (int j=0; j<lTriCount; j++){
			for (int k=j+1; k<lTriCount; k++){
				bool commonEdge = false;
				for (int mj=0; mj<3; mj++){
					int thisJVertInd = oldTriangles[3*lTri[j]+mj];
				//	if (thisJVertInd == i) continue;
				//	if (commonEdge) break;
					for (int mk=0; mk<3; mk++){
						if (thisJVertInd == oldTriangles[3*lTri[k]+mk]){ commonEdge = true;} // break; }
					}
				}


				if (commonEdge){ //if adjacent triangles
					//angles and smoothing group calc
					Vec3Df Nj = Vec3Df(&triangleNormals[3*lTri[j]]);
					Vec3Df Nk = Vec3Df(&triangleNormals[3*lTri[k]]);

					float thisAngle = acosf(Nj.Dot(Nk));
					if (thisAngle < seamAngleRad){
						int jSmoothGroup = lTriGroup[j];
						int kSmoothGroup = lTriGroup[k];
						for (int m=0; m<lTriCount; m++){
							if (lTriGroup[m] == kSmoothGroup) lTriGroup[m] = jSmoothGroup;
						}
					}
				}
			}
		}

		//map smoothing groups to whichever vertex is for their smoothing group
		bool usedOriginalVertex = false;
		for (int m=0; m<lTriCount; m++) lTriGroupToVertIndex[m] = -1;
					
		for (int m=0; m<lTriCount; m++){
			int thisGroupIndex = lTriGroup[m];
			if (lTriGroupToVertIndex[thisGroupIndex] == -1){ //unassigned
				if (!usedOriginalVertex){
					lTriGroupToVertIndex[thisGroupIndex] = i;
					usedOriginalVertex = true;
				}
				else { //must duplicate vertex
					int newIndex = vertexCount();
					for (int n=0; n<3; n++) vertices.push_back(vertices[3*i+n]); //add the new vertex
					lTriGroupToVertIndex[thisGroupIndex] = newIndex;
				}
			}
		}		

		//replace references to vertex "i" in triangles with new vertices as necessary
		for (int m=0; m<lTriCount; m++){
			for (int n=0; n<3; n++){
				if (oldTriangles[3*lTri[m]+n] == i){
					triangles[3*lTri[m]+n] = lTriGroupToVertIndex[lTriGroup[m]];
					continue;
				}
			}

		}


	}
}

void CMesh3D::calcVertNormals(float seamAngleDegrees) //called to update the vertex normals (without welding vertices this will be of limited use...)
{ 
	if (seamAngleDegrees == 0){
		if (faceNormalsStale) calcFaceNormals();
	}
	else {
		if (faceNormalsStale) calcFaceNormals();
		splitVerticesByAngle(seamAngleDegrees);
	}



	int triCount = (int)(triangles.size()/3);
	int vertCount = (int)(vertices.size()/3);
	vertexNormals.clear();
	vertexNormals.reserve(vertCount*3);

	std::vector<Vec3D<float>> newVNormals(vertices.size()/3);

	for (int i=0; i<triCount; i++){
		Vec3D<float> thisFNormal(&triangleNormals[i*3]);
		float thisTriArea = GetTriArea(i);
		for (int j=0; j<3; j++)	newVNormals[triangles[3*i+j]] += thisTriArea*thisFNormal;
	}

	for (int i=0; i<vertCount; i++){
		newVNormals[i].Normalize();
		for (int j=0; j<3; j++) vertexNormals.push_back(newVNormals[i][j]);
	}

	vertexNormalsStale = false;
}

void CMesh3D::mergeVertices(float threshold)
{
	//	throws out colors and normals
	vertexColors.clear();
	vertexNormals.clear();

	int vCount = vertexCount();
	std::vector<int> oldToNew(vCount, -1); //size of # old vertices, values of # new vertices
	int newVCount = 0; //keep track of how many new vertices added
	Vec3D<float> size = meshSize();

	bool useStandard = true;
	if (useStandard){
		//calculate epsilon
		float epsSq = threshold*threshold; //epsilon*epsilon;

		//map old to new, shifting down as we go
		for (int i=0; i<vCount; i++){
			if (oldToNew[i] == -1){ //this vertex not already merged
				oldToNew[i] = newVCount;

				for (int j=i+1; j<vCount; j++){ //the rest of the vertices
					float dx = vertices[3*i] - vertices[3*j];
					float dy = vertices[3*i+1] - vertices[3*j+1];
					float dz = vertices[3*i+2] - vertices[3*j+2];
					if (dx*dx+dy*dy+dz*dz < epsSq)
						oldToNew[j] = newVCount; //if within epsilon
				}

				vertices[3*newVCount] = vertices[3*i];
				vertices[3*newVCount+1] = vertices[3*i+1];
				vertices[3*newVCount+2] = vertices[3*i+2];
				newVCount++;
			}
		}
	}
	else { //experimental
	//	assumes 1024 divisions in the maximum mesh dimension is enough resolution to be ok with merging "close" vertices
		int precision = 10;

		if (precision > 10) precision = 10;
		else if (precision < 2) precision = 2;
		int precNum = (1 << precision) -1;


		Vec3D<float> min = meshMin();
	//	float maxDim = std::max(size.x, std::max(size.y, size.z));

		std::unordered_map<unsigned int, int> map; //key is a hash of the three float values. value is the new vertex index

		for (int i=0; i<vCount; i++){
			//hash for this vertex: a 32-bit uint
			unsigned int ix = (unsigned int)(precNum*((vertices[3*i]-min.x)/size.x)); //each range from 0-1024
			unsigned int iy = (unsigned int)(precNum*((vertices[3*i+1]-min.y)/size.y));
			unsigned int iz = (unsigned int)(precNum*((vertices[3*i+2]-min.z)/size.z));
			unsigned int key = ix | (iy << 10) | (iz << 20);

			if (!map.count(key)){
				oldToNew[i] = newVCount;
				map[key] = newVCount;
				for (int j=0; j<3; j++) vertices[3*newVCount+j] = vertices[3*i+j];
				newVCount++; //added a new vertex
			}
			else oldToNew[i] = map[key];  //key exists
		}
	}

	vertices.resize(3*newVCount); //chop off any unused vertices
	
	int tSize = (int)triangles.size();
	for (int i=0; i<tSize; i++)
		triangles[i] = oldToNew[triangles[i]];

	faceNormalsStale = true;
	vertexNormalsStale = true;
	vertexMergesStale = false;
}

void CMesh3D::updateBounds(void)
{
	if (vertices.size() == 0) {boundsMin = Vec3D<float>(); boundsMax = Vec3D<float>(); return;} 

	boundsMin = boundsMax = Vec3D<float>(vertices[0], vertices[1], vertices[2]); 
	
	for (int i=0; i<(int)vertices.size(); i++) {
		int axis = i%3;
		if (vertices[i]<boundsMin[axis]) boundsMin[axis] = vertices[i];
		if (vertices[i]>boundsMax[axis]) boundsMax[axis] = vertices[i];
	}

	boundsStale = false;
}

bool CMesh3D::loadSTL(std::string& filePath)
{
	clear();
	FILE *fp;

#ifdef WIN32
	fopen_s(&fp, filePath.c_str(), "r"); //secure version. preferred on windows platforms...
#else
	fp = fopen(filePath.c_str(), "r");
#endif

	if(fp == NULL) return false;

	/* Check for binary or ASCII file */
	bool binary=false;
	fseek(fp, 0, SEEK_END);
	unsigned int facenum, file_size = ftell(fp);
	fseek(fp, STL_LABEL_SIZE, SEEK_SET);
	if (fread(&facenum, sizeof(int), 1, fp) == 0) return false;
	if(file_size == (STL_LABEL_SIZE + 4 + (sizeof(short)+12*sizeof(float) )*facenum) ) binary = true; //expected file size
	unsigned char tmpbuf[128];
	if (fread(tmpbuf,sizeof(tmpbuf),1,fp) == 0) return false;
	for(unsigned int i = 0; i < sizeof(tmpbuf); i++){ if(tmpbuf[i] > 127) binary=true; }
	
	//rewind(fp);


	//now we know binary vs ascii. reset file pointer to beginning of data
	float N[3], P[9];

	if (binary){ //if this is a binary stl
		fclose(fp);
		#ifdef WIN32
			fopen_s(&fp, filePath.c_str(), "rb"); //secure version. preferred on windows platforms...
		#else
			fp = fopen(filePath.c_str(), "rb");
		#endif

		fseek(fp, STL_LABEL_SIZE + sizeof(int), SEEK_SET); //STL_LABEL_SIZE + facenum
		short attr;

		for(int i=0;i<(int)facenum;++i) { // For each triangle read the normal, the three coords and a short set to zero
			if (fread(&N,3*sizeof(float),1,fp) == 0) return false; //We end up throwing this out and recalculating because... we don't trust it!!!
			if (fread(&P,3*sizeof(float),3,fp) == 0) return false;
			if (fread(&attr,sizeof(short),1,fp) == 0) return false;

			int prevVCount = (int)vertices.size()/3;
			for (int j=0; j<9; j++) vertices.push_back(P[j]);
			triangles.push_back(prevVCount);
			triangles.push_back(prevVCount+1);
			triangles.push_back(prevVCount+2);
		}

	}
	else { //if this is an ascii stl
		fseek(fp,0,SEEK_SET);
		
		while(getc(fp) != '\n'){}	// Skip the first line of the file 
		int /*cnt=0, */lineCnt=0, ret;
		
		while(!feof(fp)){ // Read a single facet from an ASCII .STL file 
			ret=fscanf(fp, "%*s %*s %f %f %f\n", &N[0], &N[1], &N[2]); // --> "facet normal 0 0 0" (We throw this out and recalculate based on vertices)
			if(ret!=3){	// we could be in the case of a multiple solid object, where after a endfacet instead of another facet we have to skip two lines:
				lineCnt++;			//  |     endloop
				continue;			//  |	 endfacet
									//  |endsolid     <- continue on ret==0 will skip this line
									//  |solid ascii  <- and this one.
									//  |   facet normal 0.000000e+000 7.700727e-001 -6.379562e-001
			}
			ret=fscanf(fp, "%*s %*s"); // --> "outer loop"
			ret=fscanf(fp, "%*s %f %f %f\n", &P[0],  &P[1],  &P[2]); // --> "vertex x y z"
			if(ret!=3) return false;
			ret=fscanf(fp, "%*s %f %f %f\n", &P[3],  &P[4],  &P[5]); // --> "vertex x y z"
			if(ret!=3) return false;
			ret=fscanf(fp, "%*s %f %f %f\n", &P[6],  &P[7],  &P[8]); // --> "vertex x y z"
			if(ret!=3) return false;
			ret=fscanf(fp, "%*s"); // --> "endloop"
			ret=fscanf(fp, "%*s"); // --> "endfacet"
			lineCnt+=7;
			if(feof(fp)) break;

			int prevVCount = (int)vertices.size()/3;
			for (int j=0; j<9; j++) vertices.push_back(P[j]);
			triangles.push_back(prevVCount);
			triangles.push_back(prevVCount+1);
			triangles.push_back(prevVCount+2);

		}
	}
	fclose(fp);

	meshChanged();
	useFaceNormals();

	return true;
}




bool CMesh3D::saveSTL(std::string& filePath, bool binary) const { //writes ascii stl file...

	FILE *fp;

#ifdef WIN32
	if (binary) fopen_s (&fp, filePath.c_str(),"wb"); //secure version. preferred on windows platforms...
	else fopen_s(&fp, filePath.c_str(),"w");
#else
	if (binary) fp = fopen(filePath.c_str(),"wb");
	else fp = fopen(filePath.c_str(),"w");
#endif

	if(fp==0) return false;
	int tCount = (int)triangles.size()/3; //triangle count

	if(binary){
		std::string tmp = "DefaultSTL_Voxelyze_jdh                                                                                       "; 
		fwrite(tmp.c_str(),80,1,fp); // Write header
		fwrite(&tCount,1,sizeof(int),fp); // write number of facets (triangles)
		unsigned short attributes=0;

		float N[3], P[9];
		for(int i=0; i<tCount; i++){
			for (int j=0; j<3; j++){
				N[j] = triangleNormals[3*i+j];
				for (int k=0; k<3; k++){
					P[3*j+k] = vertices[3*triangles[3*i+j]+k];
				}
			}

			fwrite(&N,3,sizeof(float),fp); //write normal
 			for(int k=0;k<3;k++){fwrite(&P[3*k],3,sizeof(float),fp);} //write coordinates
			fwrite(&attributes,1,sizeof(short),fp); //write a short set to zero
		}
	}
	else {
		fprintf(fp,"solid jdh\n");
		for(int i=0; i<tCount; i++){
		  	// For each triangle write the normal, the three coords and a short set to zero
			fprintf(fp,"  facet normal %13e %13e %13e\n", triangleNormals[3*i], triangleNormals[3*i+1], triangleNormals[3*i+2]);
			fprintf(fp,"    outer loop\n");
			for(int k=0; k<3; k++) fprintf(fp,"      vertex  %13e %13e %13e\n", vertices[3*triangles[3*i+k]], vertices[3*triangles[3*i+k]+1], vertices[3*triangles[3*i+k]+2]);			
			fprintf(fp,"    endloop\n");
			fprintf(fp,"  endfacet\n");
		}
		fprintf(fp,"endsolid vcg\n");
	}
	fclose(fp);

	return 0;
}


bool CMesh3D::saveOBJ(std::string& filePath) const
{
	std::ofstream ofile(filePath.c_str());
	ofile << "# OBJ file generated by Voxelyze\n";
	for (int i=0; i<(int)(vertices.size()/3); i++){
		ofile << "v " << vertices[3*i] << " " << vertices[3*i+1] << " " << vertices[3*i+2] << "\n";
	}
	if (normalsByVertex){
		for (int i=0; i<(int)(vertices.size()/3); i++){
			ofile << "vn " << vertexNormals[3*i] << " " << vertexNormals[3*i+1] << " " << vertexNormals[3*i+2] << "\n";
		}
	}
	else { //flat faces
		for (int i=0; i<(int)(triangles.size()/3); i++){
			ofile << "vn " << triangleNormals[3*i] << " " << triangleNormals[3*i+1] << " " << triangleNormals[3*i+2] << "\n";
		}
	}

	for (int i=0; i<(int)(triangles.size()/3); i++){
		if (normalsByVertex)
			ofile << "f " << triangles[3*i]+1 << "//" << triangles[3*i]+1 << " " << triangles[3*i+1]+1 << "//" << triangles[3*i+1]+1 << " " << triangles[3*i+2]+1 << "//" << triangles[3*i+2]+1 << "\n";
		else 
			ofile << "f " << triangles[3*i]+1 << "//" << i+1 << " " << triangles[3*i+1]+1 << "//" << i+1 << " " << triangles[3*i+2]+1 << "//" << i+1 << "\n";
	}
	ofile.close();
	return true;
}


void CMesh3D::translate(Vec3D<float> d)
{
	for (int i=0; i<(int)vertices.size(); i++) vertices[i] += d[i%3];
	meshChanged();
}

void CMesh3D::scale(Vec3D<float> s)
{
	for (int i=0; i<(int)vertices.size(); i++) vertices[i] *= s[i%3];
	meshChanged();
	
}

void CMesh3D::rotate(Vec3D<float> ax, float a)
{
	bool hasVertNorms = (vertexNormals.size() != 0);
	for (int i=0; i<(int)vertices.size(); i+= 3) {
		Vec3D<float> vert(vertices[i], vertices[i+1], vertices[i+2]);
		vert = vert.Rot(ax, a);
		for (int j=0; j<3; j++) vertices[i+j] = vert[j];
		if (hasVertNorms){
			Vec3D<float> norm(vertexNormals[i], vertexNormals[i+1], vertexNormals[i+2]);
			norm = norm.Rot(ax, a);
			for (int j=0; j<3; j++) vertexNormals[i+j] = norm[j];
		}
	}
	if (triangleNormals.size() != 0){
		for (int i=0; i<(int)triangles.size(); i+=3) {
			Vec3D<float> norm(triangleNormals[i], triangleNormals[i+1], triangleNormals[i+2]);
			norm = norm.Rot(ax, a);
			for (int j=0; j<3; j++) triangleNormals[i+j] = norm[j];
		}
	}
	meshChanged();
	
}

float CMesh3D::distanceFromSurface(Vec3D<float>* point, float maxDistance, Vec3D<float>* pNormalOut)
{
//	if (vertexMergesStale) mergeVertices(10);
	if (pNormalOut){
		if (faceNormalsStale) calcFaceNormals();
		*pNormalOut = Vec3D<float>(0,0,0);
	}

	if (!FillCheckTriInts(point->y, point->z, maxDistance)) return FLT_MAX; //returns very fast if previously used z or y layers...

	int triCount = (int)TriLine.size();
	float minDist2 = FLT_MAX;
//	Vec3D<float> avgNormal;
//	float avgNormalSum = 0;
//	float distOut;
//	float areaSum;
	float minPointX = point->x-maxDistance, maxPointX = point->x+maxDistance;

	for (int i=0; i<triCount; i++){ //all triangles within maxDistance
		int thisTriIndex = TriLine[i];
		bool allAbove = true, allBelow = true;
		for (int j=0; j<3; j++){
			float thisX = vertices[3*triangles[3*thisTriIndex+j]];
			if (thisX < maxPointX) allAbove = false;
			if (thisX > minPointX) allBelow = false;
		}
		if (!allAbove && !allBelow){ //within range to consider further
			float u, v;
			Vec3D<float> toIntersect;
			bool result = GetTriDist(thisTriIndex, point, u, v, toIntersect);
			//float area = GetTriArea(TriLine[i]);
			float dist2 = toIntersect.Length2();
			//if (dist2a < 0)
			//	dist2a = 0; //-dist2; //this should never happen, but seems possible.

			if (dist2 < minDist2){
				minDist2 = dist2;
				if (pNormalOut)	{ 
					if (dist2 < FLT_EPSILON) *pNormalOut = Vec3D<float>(&triangleNormals[3*thisTriIndex]); //if coincident to triangle, use that triangle's normal
					else *pNormalOut = toIntersect;
				}
			}

			//if (pNormalOut) {
			//	//float distWeight = 1/(dist2*dist2);

			//	avgNormal += distWeight*Vec3D<float>(&triangleNormals[3*TriLine[i]]); //currently greedy - takes normal from first equidistant triangle.
			//	avgNormalSum += distWeight;
			//}

//			if (dist2 <= maxDistance*maxDistance){
//				distOut += sqrt(dist2); //todo here!
//			}
		}
	}

	bool inside = isInside(point);
	if (pNormalOut){
		if(!inside) *pNormalOut = - *pNormalOut;
		pNormalOut->Normalize();
	}

	return inside ? -sqrt(minDist2) : sqrt(minDist2);
}

bool CMesh3D::isInside(Vec3D<float>* point)
{
//	if (vertexMergesStale) mergeVertices(10);

	if (!FillCheckTriInts(point->y, point->z)) return false; //returns very fast if previously used z or y layers...

	int intCount = 0;
	for (int i=0; i<(int)TriInts.size(); i++) if (TriInts[i] < point->x) intCount++;

	return (intCount%2 == 1);
}


bool CMesh3D::FillCheckTriInts(float y, float z, float pad)
{
	float maxYZ = meshSize().y > meshSize().z ? meshSize().y : meshSize().z;
	float maxEpsilonChange = 20*maxYZ*FLT_EPSILON; //the most the x or y can change in seeking a non-edge intesecting ray

	y += maxYZ*FLT_EPSILON; //these help avoid the majority of costly edge intersections at round numbers.
	z += maxYZ*FLT_EPSILON;

	pad = pad > maxEpsilonChange ? pad : maxEpsilonChange; //always keep enough padding around to do epsilon changes in the ray position
	bool yStale = abs(y-_lastY) > maxEpsilonChange; //account for small epsilon changes from before
	bool zStale = abs(z-_lastZ) > maxEpsilonChange; //account for small epsilon changes from before
	bool padStale = pad > _lastPad; 

	if (yStale || zStale || padStale) {  //recalculate
		if (padStale || zStale) FillTriLayer(z, pad);
		if (padStale || yStale || zStale) FillTriLine(y, z, pad);
		_lastY = y;
		_lastZ = z;
		_lastPad = pad;



		int tryCount = 1;
		while (!FillTriInts(y, z, pad)){ //while we don't hit any edges (or vertices) and get an even number of hits for the line
			if (tryCount%2 == 0) y += tryCount*maxYZ*FLT_EPSILON;
			else z += tryCount*maxYZ*FLT_EPSILON;
		
			if (tryCount++ > 5) return false;
		}
	}

	return true;
}

bool CMesh3D::FillTriInts(float y, float z, float padding)
{
	TriInts.clear();

	int triCount = (int)TriLine.size();
	for (int i=0; i<triCount; i++){
		float hitPointX = 0;
		IntersectionType intersect = IntersectXRay(TriLine[i], y, z, hitPointX);

		for (int j=0; j<(int)TriInts.size(); j++) {if (TriInts[j] == hitPointX)
			return false;}
		if (intersect == IT_INSIDE)
			TriInts.push_back(hitPointX);
		else if (intersect == IT_EDGE)
			return false; //try again...
	}
	if (TriInts.size() %2 ==1)
		return false; //should never be an odd number of intersections
	return true;
}


void CMesh3D::FillTriLine(float y, float z, float padding)
{
	int triCount = (int)TriLayer.size();
	TriLine.clear(); //clear previous list

	//add any Facets whose Z coordinates are not all above or all below this Z plane
	for (int i=0; i<triCount; i++){
		bool allAbove = true, allBelow = true;
		for (int j=0; j<3; j++){
			float thisY = vertices[3*triangles[3*TriLayer[i]+j]+1];
			if (thisY < y+padding) allAbove = false;
			if (thisY > y-padding) allBelow = false;
		}
		if (!allAbove && !allBelow) TriLine.push_back(TriLayer[i]);
	}
}

void CMesh3D::FillTriLayer(float z, float padding)
{
	int triCount = (int)triangles.size()/3;
	TriLayer.clear(); //clear previous list

	//add any Facets whose Z coordinates are not all above or all below this Z plane
	for (int i=0; i<triCount; i++){
		bool allAbove = true, allBelow = true;
		for (int j=0; j<3; j++){
			float thisZ = vertices[3*triangles[3*i+j]+2];
			if (thisZ < z+padding) allAbove = false;
			if (thisZ > z-padding) allBelow = false;
		}
		if (!allAbove && !allBelow) TriLayer.push_back(i);
	}
}

CMesh3D::IntersectionType CMesh3D::IntersectXRay(const int TriangleIndex, const float y, const float z, float& XIntersect) const
{
	//http://www.blackpawn.com/texts/pointinpoly/default.html

//	Vec3D<float> vA = *GetpFacetVertex(FacetIndex, 0);
//	Vec3D<float> vB = *GetpFacetVertex(FacetIndex, 1);
//	Vec3D<float> vC = *GetpFacetVertex(FacetIndex, 2);
	Vec3D<float> vA(&vertices[3*triangles[3*TriangleIndex]]); // = vertices[pFacet->vi[0]];
	Vec3D<float> vB(&vertices[3*triangles[3*TriangleIndex+1]]);
	Vec3D<float> vC(&vertices[3*triangles[3*TriangleIndex+2]]);

	//necessary if buffer has padding and some triangles are fully outside
	if ((vA.y>y && vB.y>y && vC.y>y) || (vA.y<y && vB.y<y && vC.y<y)) return IT_OUTSIDE;
	if ((vA.z>z && vB.z>z && vC.z>z) || (vA.z<z && vB.z<z && vC.z<z)) return IT_OUTSIDE;

	float v0y = vC.y-vA.y; //u
	float v0z = vC.z-vA.z;
	float v1y = vB.y-vA.y; //v
	float v1z = vB.z-vA.z;
	float v2y = y-vA.y;
	float v2z = z-vA.z;

	float dot00=v0y*v0y+v0z*v0z;
	float dot01=v0y*v1y+v0z*v1z;
	float dot02=v0y*v2y+v0z*v2z;
	float dot11=v1y*v1y+v1z*v1z;
	float dot12=v1y*v2y+v1z*v2z;

	float invDenom = 1.0f/(dot00*dot11-dot01*dot01);
	float u=(dot11*dot02-dot01*dot12)*invDenom;
	float v=(dot00*dot12-dot01*dot02)*invDenom;

	if (u<0 || v<0 || u+v > 1)
		return IT_OUTSIDE;
	else if (u>0 && v>0 && u+v<1.0){
		XIntersect = vA.x+u*(vC.x-vA.x)+v*(vB.x-vA.x);
		return IT_INSIDE;
	}
	else 
		return IT_EDGE;
}

bool CMesh3D::GetTriDist(const int TriangleIndex, const Vec3D<float>* pPointIn, float& UOut, float& VOut, Vec3D<float>& ToSurfPointOut) const //gets distance of provided point to closest UV coordinate of within the triangle. returns true if sensible distance, otherwise false
{
	//http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf

	//Regions:
	//    E1 (t)
	// \ 2|
	//   \|
	//	  |\
	//    |  \    1
	//  3 |    \
	//    | 0    \
	// ___|________\____   E0 (s)
	//    |          \ 6
	// 4  |    5       \

//	Vec3D<float> B = *GetpFacetVertex(FacetIndex, 0);
//	Vec3D<float> E0 = *GetpFacetVertex(FacetIndex, 1) - B; //(s)
//	Vec3D<float> E1 = *GetpFacetVertex(FacetIndex, 2) - B; //(t)
	Vec3D<float> B(&vertices[3*triangles[3*TriangleIndex]]);
	Vec3D<float> E0 = Vec3D<float>(&vertices[3*triangles[3*TriangleIndex+1]]) - B; //(s)
	Vec3D<float> E1 = Vec3D<float>(&vertices[3*triangles[3*TriangleIndex+2]]) - B; //(t)
	Vec3D<float> D = B - *pPointIn;

	float a = E0.Dot(E0);
	float b = E0.Dot(E1);
	float c = E1.Dot(E1);
	float d = E0.Dot(D);
	float e = E1.Dot(D);
	float f = D.Dot(D);

	float det=a*c-b*b;
	float s = b*e-c*d;
	float t = b*d-a*e;

	//determine which region...
	//Q(s,t) = as^2+2bst+ct^2+2ds+2et+f
	if (s+t <= det){
		if (s<0){
			if (t<0){ //Region 4
				// Grad(Q) = 2(as+bt+d,bs+ct+e) :: (derivative w respect to s, t)
				// (0,1)*Grad(Q(0,0)) = (0,1)*(d,e) = e :: is grad from tip of region 2 in directon of leg toward origin (0,1)...
				// (1,0)*Grad(Q(0,0)) = (1,0)*(d,e) = d
				// min on edge t=0 if (0,1)*Grad(Q(0,0)) < 0 )
				// min on edge s=0 otherwise
				if (e<0){ // minimum on edge t=0
					t=0;
					s=(d >=0 ? 0 :(-d >=a ? 1:-d/a)); //fixed 2-7! t changed to s
				}
				else { // minimum on edge s=0
					s=0;
					t=(e >=0 ? 0 :(-e >=c ? 1:-e/c));
				}
			}
			else { //Region 3
				// F(t) = Q(0,t) = ct^2 + 2et + f
				// F’(t)/2 = ct+e
				// F’(T) = 0 when T = -e/c
				s=0;
				t=(e >=0 ? 0 :(-e >=c ? 1:-e/c));
			}
		}
		else if (t<0){ //Region 5 (like region 3)
			// F(s) = Q(s,0) = as^2 + 2ds + f
			// F’(s)/2 = as+d
			// F’(S) = 0 when S = -d/a
			t=0;
			s=(d >=0 ? 0 :(-d >=a ? 1:-d/a)); //fixed 2-7! t changed to s
		}
		else { //Region 0
			float invdet = 1/det;
			s *= invdet;
			t *= invdet;
		}
	}
	else {
		if (s<0){ //Region 2
			// Grad(Q) = 2(as+bt+d,bs+ct+e) :: (derivative w respect to s, t)
			// (0,-1)*Grad(Q(0,1)) = (0,-1)*(b+d,c+e) = -(c+e) :: is grad from tip of region 2 in directon of leg toward origin (0,1)...
			// (1,-1)*Grad(Q(0,1)) = (1,-1)*(b+d,c+e) = (b+d)-(c+e)
			// min on edge s+t=1 if (1,-1)*Grad(Q(0,1)) < 0 )
			// min on edge s=0 otherwise
			float tmp0 = b+d;
			float tmp1 = c+e;
			if ( tmp1 > tmp0 ){ // minimum on edge s+t=1
				float numer = tmp1 - tmp0;
				float denom = a-2*b+c;
				s = ( numer >= denom ? 1 : numer/denom );
				t = 1-s;
			}
			else { // minimum on edge s=0
				s = 0;
				t = ( tmp1 <= 0 ? 1 : ( e >= 0 ? 0 : -e/c ) );
			}
		}
		else if (t<0){ //Region 6 
			// Grad(Q) = 2(as+bt+d,bs+ct+e) :: (derivative w respect to s, t)
			// (-1,0)*Grad(Q(1,0)) = (-1,0)*(a+d,b+e) = -(a+d) :: is grad from tip of region 2 in directon of leg toward origin (0,1)...
			// (-1,1)*Grad(Q(1,0)) = (-1,1)*(a+d,b+e) = (b+e)-(a+d)
			// min on edge s+t=1 if (-1,1)*Grad(Q(0,1)) < 0 )
			// min on edge t=0 otherwise
			float tmp0 = b+e;
			float tmp1 = a+d;
			if ( tmp1 > tmp0 ){ // minimum on edge s+t=1
				float numer = tmp1 - tmp0;
				float denom = a-2*b+c;
				t = ( numer >= denom ? 1 : numer/denom );
				s = 1-t;
			}
			else { // minimum on edge t=0
				t = 0;
				s = ( tmp1 <= 0 ? 1 : ( d >= 0 ? 0 : -d/a ) );
			}
		}
		else { //Region 1
			// F(s) = Q(s,1-s) = (a-2b+c)s^2 + 2(b-c+d-e)s + (c+2e+f)
			// F’(s)/2 = (a-2b+c)s + (b-c+d-e)
			// F’(S) = 0 when S = (c+e-b-d)/(a-2b+c)
			// a-2b+c = |E0-E1|^2 > 0, so only sign of c+e-b-d need be considered
			float numer = c+e-b-d;
			if (numer <= 0) s=0;
			else {
				float denom = a-2*b+c;
				s = (numer >= denom ? 1 : numer/denom);
			}
			t=1-s;
		}
	}

	//check one or the other? should be same!
	Vec3D<float> ClosestPoint = B+s*E0+t*E1;
	ToSurfPointOut = ClosestPoint-*pPointIn; //OUTPUT!

//	Dist2Out = a*s*s+2*b*s*t+c*t*t+2*d*s+2*e*t+f;

	UOut = s;
	VOut = t;
//	Dist2Out = D22;
	return true;
}

float CMesh3D::GetTriArea(const int TriangleIndex)
{
	Vec3D<float> B(&vertices[3*triangles[3*TriangleIndex]]);
	Vec3D<float> E0 = Vec3D<float>(&vertices[3*triangles[3*TriangleIndex+1]]) - B; //(s)
	Vec3D<float> E1 = Vec3D<float>(&vertices[3*triangles[3*TriangleIndex+2]]) - B; //(t)

	return 0.5f*E0.Cross(E1).Length();
}


//
////---------------------------------------------------------------------------
//int CMesh::GetXIntersections(vfloat z, vfloat y, vfloat* pIntersections, int NumtoCheck, int* pToCheck)
////---------------------------------------------------------------------------
//{ //returns the number of intersections, stored in pIntersections. pToCheck is a vector of facet indices that are in this Z plane...
//	Vec3D<> p;
//	vfloat pu, pv, V1y, V2y, V3y;
//	int NumFound = 0;
//
//	for (int i=0; i<NumtoCheck; i++){ //for each facet we wish to check...
//		V1y = Vertices[Facets[pToCheck[i]].vi[0]].v.y;
//		V2y = Vertices[Facets[pToCheck[i]].vi[1]].v.y;
//		V3y = Vertices[Facets[pToCheck[i]].vi[2]].v.y;
//		//trivial checks (should get most of them...)
//		if (V1y < y && V2y < y && V3y < y)
//			continue;
//		if (V1y > y && V2y > y && V3y > y)
//			continue;
//
//		if(IntersectXRay(&Facets[pToCheck[i]], y, z, p, pu, pv)) { //if it intersects
//			if (InsideTri(p, Vertices[Facets[pToCheck[i]].vi[0]].v, Vertices[Facets[pToCheck[i]].vi[1]].v, Vertices[Facets[pToCheck[i]].vi[2]].v)){
//				pIntersections[NumFound++] = p.x; //(1.0 - pu - pv)*Vertices[Facets[pToCheck[i]].vi[0]].v.x + pu*Vertices[Facets[pToCheck[i]].vi[1]].v.x + pv*Vertices[Facets[pToCheck[i]].vi[2]].v.x;
//			}
//		}
//	}
//	
////	if (NumFound%2 != 0) std::cout << "Uh-oh! Found an odd number of intersections!";
//	
//	//sort intersections... (bubble sort = slow, but these should be super small...
//	vfloat tmp;
//	for (int i=0; i<NumFound; i++){
//		for (int j=0; j<NumFound - i - 1; j++){ //each iteration gets the largest element to the end...
//			if(pIntersections[j] > pIntersections[j+1]){
//				tmp = pIntersections[j+1];
//				pIntersections[j+1] = pIntersections[j];
//				pIntersections[j] = tmp;
//			}
//		}
//	}
//
//	return NumFound;
//}
//
////---------------------------------------------------------------------------
//bool CMesh::InsideTri(Vec3D<>& p, Vec3D<>& v0, Vec3D<>& v1, Vec3D<>& v2)
////---------------------------------------------------------------------------
//{// True if point p projects to within triangle (v0;v1;v2)
//
//	Vec3D<> xax = (v1-v0).Normalized();
//	Vec3D<> zax = ((v2-v0).Cross(xax)).Normalized();
//	Vec3D<> yax = zax.Cross(xax).Normalized();
//
//	Vec3D<> p0(0,0,1);
//	Vec3D<> p1((v1-v0).Dot(xax),(v1-v0).Dot(yax),1);
//	Vec3D<> p2((v2-v0).Dot(xax),(v2-v0).Dot(yax),1);
//	Vec3D<> pt((p-v0).Dot(xax),(p-v0).Dot(yax),1);
//
//	vfloat d0 = Det(p0,p1,pt);
//	vfloat d1 = Det(p1,p2,pt);
//	vfloat d2 = Det(p2,p0,pt);
//
//	if (d0<=0 && d1<=0 && d2<=0)
//		return true;
//	if (d0>=0 && d1>=0 && d2>=0)
//		return true;
//
//	return false;
//
//}
//
////---------------------------------------------------------------------------
//vfloat CMesh::Det(Vec3D<>& v0, Vec3D<>& v1, Vec3D<>& v2)
////---------------------------------------------------------------------------
//{ // Compute determinant of 3x3 matrix v0,v1,v2
//
//	return 
//
//		v0.x*(v1.y*v2.z-v1.z*v2.y) +
//		v0.y*(v1.z*v2.x-v1.x*v2.z) +
//		v0.z*(v1.x*v2.y-v1.y*v2.x);
//
//}
//
////---------------------------------------------------------------------------
//bool CMesh3D::IntersectXRay(int triIndex, float y, float z, Vec3D<float>& p, float& pu, float& pv)
////---------------------------------------------------------------------------
//{
//	// compute intersection point P of triangle plane with ray from origin O in direction D
//	// D assumed to be normalized
//	// if no interstion, return false
//	// u and v are barycentric coordinates of the intersection point P = (1 - u - v)A + uB + vC 
//	// see http://www.devmaster.net/wiki/Ray-triangle_intersection
//
//
////	Vec3D<> a = Vertices[pFacet->vi[0]].v;
////	Vec3D<> b = Vertices[pFacet->vi[1]].v;
////	Vec3D<> c = Vertices[pFacet->vi[2]].v;
//
//	Vec3D<float> v[3];
//	for (int i=0; i<3; i++){
//		for (int j=0; j<3; j++){
//			v[i][j] = vertices[3*triangles[3*triIndex+i]+j];
//		}
//	}
////	Vec3D<float> a(vertices[triangles[3*triIndex;
////	Vec3D<float> b = Vertices[pFacet->vi[1]].v;
////	Vec3D<float> c = Vertices[pFacet->vi[2]].v;
//	
//	float MinX = v[0].x < v[1].x ? (v[0].x < v[2].x ? v[0].x : v[2].x) : (v[1].x < v[2].x ? v[1].x : v[2].x) - 1.0;
//	Vec3D<float> d(1,0,0);
//	Vec3D<float> o(MinX, y, z);
//
//	//Vec3D n = pFacet->n; //((b-a).Cross(c-a)).Normalized();
//	Vec3D<float> n = ((v[1]-v[0]).Cross(v[2]-v[0])).Normalized();
//	//if (n.x > 0){ //flip vertices...
//	//	Vec3D tmp = a;
//	//	a = b;
//	//	b = tmp;
//	//	n = ((b-a).Cross(c-a)).Normalized();
//	//}
//
//	float dn = d.Dot(n);
//	if (fabs(dn)<1E-5)
//		return false; //parallel
//
//	float dist = -(o-v[0]).Dot(n)/dn;
//	Vec3D<> sD = d*dist;
//	p = o+sD;
//
//	float V1, V2, V3;
//	V1 = (v[1]-v[0]).Cross(p-v[0]).Dot(n);
//	V2 = (v[2]-v[1]).Cross(p-v[1]).Dot(n);
//	V3 = (v[0]-v[2]).Cross(p-v[2]).Dot(n);
//	
//	if (V1 >=0 && V2 >=0 && V2 >=0) return true;
//	//if (V1 <=0 && V2 <=0 && V2 <=0) return true;
//	else return false;
//
//}
//
////
//void CMesh3D::RotX(vfloat a)
//{
//	for (int i=0; i<(int)Vertices.size(); i++) {
//		Vertices[i].v.RotX(a);
//		Vertices[i].n.RotX(a);
//	}
//	for (int i=0; i<(int)Facets.size(); i++) {
//		Facets[i].n.RotX(a);
//	}
//	meshChanged();
//	
//}
//
//
//void CMesh3D::RotY(vfloat a)
//{
//	for (int i=0; i<(int)Vertices.size(); i++) {
//		Vertices[i].v.RotY(a);
//		Vertices[i].n.RotY(a);
//	}
//	for (int i=0; i<(int)Facets.size(); i++) {
//		Facets[i].n.RotY(a);
//	}
//	meshChanged();
//
//}
//
//
//void CMesh3D::RotZ(vfloat a)
//{
//	for (int i=0; i<(int)Vertices.size(); i++) {
//		Vertices[i].v.RotZ(a);
//		Vertices[i].n.RotZ(a);
//	}
//	for (int i=0; i<(int)Facets.size(); i++) {
//		Facets[i].n.RotZ(a);
//	}
//	meshChanged();
//
//}


//#ifdef WIN32
//#include <Math.h>
//#else
//#include <math.h>
//#endif
//
//
//#include <algorithm>
//
//#ifdef USE_OPEN_GL
//#ifdef QT_GUI_LIB
//#include <qgl.h>
//#else
//#include "OpenGLInclude.h" //If not using QT's openGL system, make a header file "OpenGLInclude.h" that includes openGL library functions 
//#endif
////#include "GL_Utils.h"
//#endif
//
//#define STL_LABEL_SIZE 80
//
//CMesh::CMesh(void)
//{
//	DrawSmooth = true;
//	_CurBBMin = Vec3D<>(0,0,0);
//	_CurBBMax = Vec3D<>(0,0,0);
//	MeshChanged();
//
//}
//
//CMesh::~CMesh(void)
//{
//}
//
////copy constructure
//CMesh::CMesh(CMesh& s) {
//	*this = s;
//}
//
////overload =
//CMesh& CMesh::operator=(const CMesh& s) {
//
//	Facets.resize(s.Facets.size());
//	for (int i = 0; i<(int)Facets.size(); i++)
//		Facets[i] = s.Facets[i];
//
//	Vertices.resize(s.Vertices.size());
//	for (int i = 0; i<(int)Vertices.size(); i++)
//		Vertices[i] = s.Vertices[i];
//
//	Lines.resize(s.Lines.size());
//	for (int i = 0; i<(int)Lines.size(); i++)
//		Lines[i] = s.Lines[i];
//	
//	DrawSmooth = s.DrawSmooth;
//	_CurBBMin = s._CurBBMin;
//	_CurBBMax = s._CurBBMax;
//	
//	MeshChanged();
//
//	return *this;
//}
//
//
//void CMesh::WriteXML(CXML_Rip* pXML, bool MeshOnly)
//{
//	pXML->DownLevel("CMesh");
//		pXML->Element("DrawSmooth", DrawSmooth);
//		pXML->DownLevel("Vertices");
//		std::vector<CVertex>::iterator VIt;
//		for(VIt=Vertices.begin(); VIt != Vertices.end(); VIt++){
//			pXML->DownLevel("Vertex");
//				pXML->Element("Vx", VIt->v.x);
//				pXML->Element("Vy", VIt->v.y);
//				pXML->Element("Vz", VIt->v.z);
//				if (!MeshOnly){
//					if (VIt->n != Vec3D<>(0,0,0)){
//						pXML->Element("Nx", VIt->n.x);
//						pXML->Element("Ny", VIt->n.y);
//						pXML->Element("Nz", VIt->n.z);
//					}
//					pXML->Element("R", VIt->VColor.r);
//					pXML->Element("G", VIt->VColor.g);
//					pXML->Element("B", VIt->VColor.b);
//					pXML->Element("A", VIt->VColor.a);
//					if (VIt->DrawOffset != Vec3D<>(0,0,0)){
//						pXML->Element("DOx", VIt->DrawOffset.x);
//						pXML->Element("DOy", VIt->DrawOffset.y);
//						pXML->Element("DOz", VIt->DrawOffset.z);
//					}
//				}
//			pXML->UpLevel();
//		}
//		pXML->UpLevel();
//
//		pXML->DownLevel("Facets");
//		std::vector<CFacet>::iterator FIt;
//		for(FIt=Facets.begin(); FIt != Facets.end(); FIt++){
//			pXML->DownLevel("Facet");
//				pXML->Element("V0", FIt->vi[0]);
//				pXML->Element("V1", FIt->vi[1]);
//				pXML->Element("V2", FIt->vi[2]);
//				if (!MeshOnly){
//					if (FIt->n != Vec3D<>(0,0,0)){
//						pXML->Element("Nx", FIt->n.x);
//						pXML->Element("Ny", FIt->n.y);
//						pXML->Element("Nz", FIt->n.z);
//					}
//					pXML->Element("R", FIt->FColor.r);
//					pXML->Element("G", FIt->FColor.g);
//					pXML->Element("B", FIt->FColor.b);
//					pXML->Element("A", FIt->FColor.a);
//					pXML->Element("Name", FIt->Name);
//				}
//			pXML->UpLevel();
//		}
//		pXML->UpLevel();
//
//		pXML->DownLevel("Lines");
//		std::vector<CLine>::iterator LIt;
//		for(LIt=Lines.begin(); LIt != Lines.end(); LIt++){
//			pXML->DownLevel("Line");
//				pXML->Element("V0", LIt->vi[0]);
//				pXML->Element("V1", LIt->vi[1]);
//			pXML->UpLevel();
//		}
//		pXML->UpLevel();
//	pXML->UpLevel();
//}
//
//bool CMesh::ReadXML(CXML_Rip* pXML)
//{
//	Clear();
//
//	if (!pXML->FindLoadElement("DrawSmooth", &DrawSmooth)) DrawSmooth = false;
//
//	CVertex tmp;
//	if (pXML->FindElement("Vertices")){
//		while (pXML->FindElement("Vertex")){
//			if (!pXML->FindLoadElement("Vx", &tmp.v.x)) tmp.v.x = 0.0;
//			if (!pXML->FindLoadElement("Vy", &tmp.v.y)) tmp.v.y = 0.0;
//			if (!pXML->FindLoadElement("Vz", &tmp.v.z)) tmp.v.z = 0.0;
//			if (!pXML->FindLoadElement("Nx", &tmp.n.x)) tmp.n.x = 0.0;
//			if (!pXML->FindLoadElement("Ny", &tmp.n.y)) tmp.n.y = 0.0;
//			if (!pXML->FindLoadElement("Nz", &tmp.n.z)) tmp.n.z = 0.0;
//			if (!pXML->FindLoadElement("R", &tmp.VColor.r)) tmp.VColor.r = 1.0;
//			if (!pXML->FindLoadElement("G", &tmp.VColor.g)) tmp.VColor.g = 1.0;
//			if (!pXML->FindLoadElement("B", &tmp.VColor.b)) tmp.VColor.b = 1.0;
//			if (!pXML->FindLoadElement("A", &tmp.VColor.a)) tmp.VColor.a = 1.0;
//			if (!pXML->FindLoadElement("DOx", &tmp.DrawOffset.x)) tmp.DrawOffset.x = 0.0;
//			if (!pXML->FindLoadElement("DOy", &tmp.DrawOffset.y)) tmp.DrawOffset.y = 0.0;
//			if (!pXML->FindLoadElement("DOz", &tmp.DrawOffset.z)) tmp.DrawOffset.z = 0.0;
//			Vertices.push_back(tmp);
//		}
//		pXML->UpLevel();
//	}
//
//	CFacet Ftmp;
//	if (pXML->FindElement("Facets")){
//		while (pXML->FindElement("Facet")){
//			if (!pXML->FindLoadElement("V0", &Ftmp.vi[0])) Ftmp.vi[0] = 0;
//			if (!pXML->FindLoadElement("V1", &Ftmp.vi[1])) Ftmp.vi[1] = 0;
//			if (!pXML->FindLoadElement("V2", &Ftmp.vi[2])) Ftmp.vi[2] = 0;
//			if (!pXML->FindLoadElement("Nx", &Ftmp.n.x)) Ftmp.n.x = 0.0;
//			if (!pXML->FindLoadElement("Ny", &Ftmp.n.y)) Ftmp.n.y = 0.0;
//			if (!pXML->FindLoadElement("Nz", &Ftmp.n.z)) Ftmp.n.z = 0.0;
//			if (!pXML->FindLoadElement("R", &Ftmp.FColor.r)) Ftmp.FColor.r = 1.0;
//			if (!pXML->FindLoadElement("G", &Ftmp.FColor.g)) Ftmp.FColor.g = 1.0;
//			if (!pXML->FindLoadElement("B", &Ftmp.FColor.b)) Ftmp.FColor.b = 1.0;
//			if (!pXML->FindLoadElement("A", &Ftmp.FColor.a)) Ftmp.FColor.a = 1.0;
//			if (!pXML->FindLoadElement("Name", &Ftmp.Name)) Ftmp.Name = -1;
//
//			Facets.push_back(Ftmp);
//		}
//		pXML->UpLevel();
//	}
//
//	CLine Ltmp;
//	if (pXML->FindElement("Lines")){
//		while (pXML->FindElement("Line")){
//			if (!pXML->FindLoadElement("V0", &Ltmp.vi[0])) Ltmp.vi[0] = 0;
//			if (!pXML->FindLoadElement("V1", &Ltmp.vi[1])) Ltmp.vi[1] = 0;
//			Lines.push_back(Ltmp);
//		}
//		pXML->UpLevel();
//	}
//
//	UpdateBoundingBox();
//
//	CalcFaceNormals();
//	CalcVertNormals();
//	/*
//	int PrimType = -1;
//	float fX, fY, fZ; //to retrieve to Vec3D
//	if (!pXML->FindLoadElement("PrimType", &PrimType)) return false;
//
//	if (PrimType == PRIM_BOX)
//		CreateBoxRegion(Vec3D(0,0,0), Vec3D(0,0,0)); //creates box and sets pointer
//	else if (PrimType == PRIM_CYLINDER)
//		CreateCylRegion(Vec3D(0,0,0), Vec3D(0,0,0), 0);  //creates cylinder and sets pointer
//	else if (PrimType == PRIM_SPHERE)
//		CreateSphRegion(Vec3D(0,0,0), 0);  //creates Sphere and sets pointer
//	else if (PrimType == PRIM_MESH)
//		CreateMeshRegion(Vec3D(0,0,0), Vec3D(0,0,0));  //creates Sphere and sets pointer
//	
//
//	if (!pXML->FindLoadElement("X", &pRegion->X)) pRegion->X = 0;
//	if (!pXML->FindLoadElement("Y", &pRegion->Y)) pRegion->Y = 0;
//	if (!pXML->FindLoadElement("Z", &pRegion->Z)) pRegion->Z = 0;
//	if (!pXML->FindLoadElement("dX", &pRegion->dX)) pRegion->dX = 0;
//	if (!pXML->FindLoadElement("dY", &pRegion->dY)) pRegion->dY = 0;
//	if (!pXML->FindLoadElement("dZ", &pRegion->dZ)) pRegion->dZ = 0;
//	if (!pXML->FindLoadElement("Radius", &pRegion->Radius)) pRegion->Radius = 0;
//	if (!pXML->FindLoadElement("R", &pRegion->R)) pRegion->R = 0;
//	if (!pXML->FindLoadElement("G", &pRegion->G)) pRegion->G = 0;
//	if (!pXML->FindLoadElement("B", &pRegion->B)) pRegion->B = 0;
//	if (!pXML->FindLoadElement("alpha", &pRegion->alpha)) pRegion->alpha = 0;
//	if (!pXML->FindLoadElement("Fixed", &Fixed)) Fixed = 0;
//	if (!pXML->FindLoadElement("ForceX", &fX)) fX = 0;
//	if (!pXML->FindLoadElement("ForceY", &fY)) fY = 0;
//	if (!pXML->FindLoadElement("ForceZ", &fZ)) fZ = 0;
//
//	Force = Vec3D(fX, fY, fZ);
//	pRegion->UpdateAspect();
//	*/
//	return true;
//}
//
//
//#ifdef USE_OPEN_GL
////---------------------------------------------------------------------------
//void CMesh::Draw(bool bModelhNormals, bool bShaded, bool bIngoreColors, bool bIgnoreNames)
////---------------------------------------------------------------------------
//{
//	if (bShaded) {
//		for (int i=0; i<(int)Facets.size(); i++) {
//			if (!bIgnoreNames) glLoadName(Facets[i].Name);
//			glBegin(GL_TRIANGLES);
//
//			if (!DrawSmooth){ //if setting things per triangle, can do normal and color here...
//				glNormal3d(Facets[i].n.x, Facets[i].n.y, Facets[i].n.z);
//				if (!bIngoreColors) glColor3d(Facets[i].FColor.r, Facets[i].FColor.g, Facets[i].FColor.b);
//			}
//			for (int j=0; j<3; j++) {
//				CVertex& CurVert = Vertices[Facets[i].vi[j]]; //just a local reference for readability
//
//				if (DrawSmooth){ //if we want to draw smoothed normals/colors per vertex
//					glNormal3d(CurVert.n.x, CurVert.n.y, CurVert.n.z);
//					if (!bIngoreColors) glColor3d(CurVert.VColor.r, CurVert.VColor.g, CurVert.VColor.b);
//				}
//
//				glVertex3d(CurVert.v.x + CurVert.DrawOffset.x, CurVert.v.y + CurVert.DrawOffset.y, CurVert.v.z + CurVert.DrawOffset.z);
//
//			}
//			glEnd();
//
//		}
//
//		glLineWidth(1.0);
//
//		glBegin(GL_LINES);
//		glColor3d(0, 0, 0); //black only for now...
//
//		for (int i=0; i<(int)Lines.size(); i++) {
//			for (int j=0; j<2; j++) {
//				CVertex& CurVert = Vertices[Lines[i].vi[j]]; //just a local reference for readability
//				glVertex3d(CurVert.v.x + CurVert.DrawOffset.x, CurVert.v.y + CurVert.DrawOffset.y, CurVert.v.z + CurVert.DrawOffset.z);
//			}
//		}
//		glEnd();
//
//	}
//	else { // wireframe
//		for (int i=0; i<(int)Facets.size(); i++) {
//			glBegin(GL_LINE_LOOP);
//			glNormal3d(Facets[i].n.x, Facets[i].n.y, Facets[i].n.z);
//			for (int j=0; j<3; j++) {
//				CVertex& CurVert = Vertices[Facets[i].vi[j]]; //just a local reference for readability
//				glColor3d(CurVert.VColor.r, CurVert.VColor.g, CurVert.VColor.b);
//				glVertex3d(CurVert.v.x + CurVert.DrawOffset.x , CurVert.v.y + CurVert.DrawOffset.y, CurVert.v.z + CurVert.DrawOffset.z);
//			}
//			glEnd();
//		}
//	}
//
//	if (bModelhNormals) {
//		glColor3d(1,1,0);
//		glBegin(GL_LINES);
//		for (int i=0; i<(int)Facets.size(); i++) {
//			Vec3D<> c = (Vertices[Facets[i].vi[0]].v + Vertices[Facets[i].vi[1]].v + Vertices[Facets[i].vi[2]].v)/3;
//			Vec3D<> c2 = c - Facets[i].n*3;
//			glVertex3d(c.x, c.y, c.z);
//			glVertex3d(c2.x, c2.y, c2.z);
//		}
//		glEnd();
//	}
//
//}
//#endif
//
////---------------------------------------------------------------------------
//void CMesh::CalcFaceNormals() //called to update the face normals...
////---------------------------------------------------------------------------
//{
//	for (int i=0; i<(int)Facets.size(); i++){
//		Facets[i].n = ((Vertices[Facets[i].vi[1]].OffPos()-Vertices[Facets[i].vi[0]].OffPos()).Cross(Vertices[Facets[i].vi[2]].OffPos()-Vertices[Facets[i].vi[0]].OffPos())).Normalized();
//	}
//}
//
//
////---------------------------------------------------------------------------
//void CMesh::CalcVertNormals()
////---------------------------------------------------------------------------
//{ //called once for each new geometry
//	//fills in Vertices.n
//	for (int i=0; i<(int)Vertices.size(); i++){
//		Vertices[i].n = Vec3D<>(0,0,0);
//	}
//
//	for (int i=0; i<(int)Facets.size(); i++){
//
//		for (int j=0; j<3; j++){
//			Vertices[Facets[i].vi[j]].n += Facets[i].n;
//		}
//	}
//
//	for (int i=0; i<(int)Vertices.size(); i++){
//		Vertices[i].n.Normalize();
//	}
//}
//
//void CMesh::AddFacet(const Vec3D<>& v1, const Vec3D<>& v2, const Vec3D<>& v3, bool QuickAdd) //adds a facet, checks vertex list for existing vertices...
//{
//	AddFacet(v1, v2, v3, CColor(0.5, 0.5, 0.5, 1.0), CColor(0.5, 0.5, 0.5, 1.0), CColor(0.5, 0.5, 0.5, 1.0));
//}
//
////---------------------------------------------------------------------------
//void CMesh::AddFacet(const Vec3D<>& v1, const Vec3D<>& v2, const Vec3D<>& v3, const CColor& Col1, const CColor& Col2, const CColor& Col3, bool QuickAdd) //adds a facet... with color info
////---------------------------------------------------------------------------
//{
//	vfloat WeldThresh = 1e-10f; //This needs to be around the precision of a float.
//
//	Vec3D<> Points[3]; //make a local array for easy referencing
//	Points[0] = v1;
//	Points[1] = v2;
//	Points[2] = v3;
//	CColor Colors[3];
//	Colors[0] = Col1;
//	Colors[1] = Col2;
//	Colors[2] = Col3;
//
//
//	int FoundIndex[3]; //each index for a triangle...
//
//	for (int j=0; j<3; j++){ //each point in this facet
//		FoundIndex[j] = -1;
//
//		if (!QuickAdd){
//			for (int k=Vertices.size()-1; k>=0; k--){ //DO THIS BACKWARDS!!!! (more likely to have just added one next to us...)
//				if (abs(Points[j].x - Vertices[k].v.x) < WeldThresh  &&  abs(Points[j].y - Vertices[k].v.y) < WeldThresh  &&  abs(Points[j].z - Vertices[k].v.z) < WeldThresh){ //if points are identical...
//					FoundIndex[j] = k;
//					break; //kicks out of for loop, because we've found!
//				}
//			}
//		}
//
//		if (FoundIndex[j] == -1){ //if we didn't find one...
//			CVertex ThisPoint;
//			ThisPoint.v.x = Points[j].x;
//			ThisPoint.v.y = Points[j].y;
//			ThisPoint.v.z = Points[j].z;
//			ThisPoint.VColor = Colors[j];
//
//			Vertices.push_back(ThisPoint);
//			FoundIndex[j] = (int)Vertices.size() - 1; //-1 because zero-index based.
//		}
//
//	}
//
////	CFacet ThisFacet;
////	for (int m=0; m<3; m++) ThisFacet.vi[m] = FoundIndex[m];
//
//	Facets.push_back(CFacet(FoundIndex[0], FoundIndex[1], FoundIndex[2])); //TODO... select whether to create new object or add to existing...
//	MeshChanged();
//
//}
//
////---------------------------------------------------------------------------
//void CMesh::ComputeBoundingBox(Vec3D<> &pmin, Vec3D<> &pmax)
////---------------------------------------------------------------------------
//{
//	UpdateBoundingBox();
//	pmin = _CurBBMin;
//	pmax = _CurBBMax;
//
//}
//
////---------------------------------------------------------------------------
//void CMesh::UpdateBoundingBox(void)
////---------------------------------------------------------------------------
//{
//	if (Vertices.size() == 0)
//		return;
//
//	_CurBBMin = _CurBBMax = Vertices[0].v;
//	
//	for (int i=0; i<(int)Vertices.size(); i++) {
//		_CurBBMin.x = _CurBBMin.x < Vertices[i].v.x ? _CurBBMin.x : Vertices[i].v.x;
//		_CurBBMin.y = _CurBBMin.y < Vertices[i].v.y ? _CurBBMin.y : Vertices[i].v.y;
//		_CurBBMin.z = _CurBBMin.z < Vertices[i].v.z ? _CurBBMin.z : Vertices[i].v.z;
//		_CurBBMax.x = _CurBBMax.x > Vertices[i].v.x ? _CurBBMax.x : Vertices[i].v.x;
//		_CurBBMax.y = _CurBBMax.y > Vertices[i].v.y ? _CurBBMax.y : Vertices[i].v.y;
//		_CurBBMax.z = _CurBBMax.z > Vertices[i].v.z ? _CurBBMax.z : Vertices[i].v.z;
//	}
//}
//
//
//
//
////---------------------------------------------------------------------------
//bool CMesh::IsInside(Vec3D<>* Point)
////---------------------------------------------------------------------------
//{
//	FillTriLine(Point->y, Point->z); //returns very fast if previously used z or y layers...
//	//so, assume TriLine is filled correctly
//
//	std::vector<vfloat>::iterator LIter;
//	int count = 0;
//	for (LIter = TriLine.begin(); LIter != TriLine.end(); LIter++){
//		if (Point->x < *LIter) break;
//		count ++;
//
//	}
//	if (count%2 == 1) return true; //if we've passed an off number of facets...
//	else return false;
//}
//
//
////---------------------------------------------------------------------------
//int CMesh::GetXIntersections(vfloat z, vfloat y, vfloat* pIntersections, int NumtoCheck, int* pToCheck)
////---------------------------------------------------------------------------
//{ //returns the number of intersections, stored in pIntersections. pToCheck is a vector of facet indices that are in this Z plane...
//	Vec3D<> p;
//	vfloat pu, pv, V1y, V2y, V3y;
//	int NumFound = 0;
//
//	for (int i=0; i<NumtoCheck; i++){ //for each facet we wish to check...
//		V1y = Vertices[Facets[pToCheck[i]].vi[0]].v.y;
//		V2y = Vertices[Facets[pToCheck[i]].vi[1]].v.y;
//		V3y = Vertices[Facets[pToCheck[i]].vi[2]].v.y;
//		//trivial checks (should get most of them...)
//		if (V1y < y && V2y < y && V3y < y)
//			continue;
//		if (V1y > y && V2y > y && V3y > y)
//			continue;
//
//		if(IntersectXRay(&Facets[pToCheck[i]], y, z, p, pu, pv)) { //if it intersects
//			if (InsideTri(p, Vertices[Facets[pToCheck[i]].vi[0]].v, Vertices[Facets[pToCheck[i]].vi[1]].v, Vertices[Facets[pToCheck[i]].vi[2]].v)){
//				pIntersections[NumFound++] = p.x; //(1.0 - pu - pv)*Vertices[Facets[pToCheck[i]].vi[0]].v.x + pu*Vertices[Facets[pToCheck[i]].vi[1]].v.x + pv*Vertices[Facets[pToCheck[i]].vi[2]].v.x;
//			}
//		}
//	}
//	
////	if (NumFound%2 != 0) std::cout << "Uh-oh! Found an odd number of intersections!";
//	
//	//sort intersections... (bubble sort = slow, but these should be super small...
//	vfloat tmp;
//	for (int i=0; i<NumFound; i++){
//		for (int j=0; j<NumFound - i - 1; j++){ //each iteration gets the largest element to the end...
//			if(pIntersections[j] > pIntersections[j+1]){
//				tmp = pIntersections[j+1];
//				pIntersections[j+1] = pIntersections[j];
//				pIntersections[j] = tmp;
//			}
//		}
//	}
//
//	return NumFound;
//}
//
////---------------------------------------------------------------------------
//bool CMesh::InsideTri(Vec3D<>& p, Vec3D<>& v0, Vec3D<>& v1, Vec3D<>& v2)
////---------------------------------------------------------------------------
//{// True if point p projects to within triangle (v0;v1;v2)
//
//	Vec3D<> xax = (v1-v0).Normalized();
//	Vec3D<> zax = ((v2-v0).Cross(xax)).Normalized();
//	Vec3D<> yax = zax.Cross(xax).Normalized();
//
//	Vec3D<> p0(0,0,1);
//	Vec3D<> p1((v1-v0).Dot(xax),(v1-v0).Dot(yax),1);
//	Vec3D<> p2((v2-v0).Dot(xax),(v2-v0).Dot(yax),1);
//	Vec3D<> pt((p-v0).Dot(xax),(p-v0).Dot(yax),1);
//
//	vfloat d0 = Det(p0,p1,pt);
//	vfloat d1 = Det(p1,p2,pt);
//	vfloat d2 = Det(p2,p0,pt);
//
//	if (d0<=0 && d1<=0 && d2<=0)
//		return true;
//	if (d0>=0 && d1>=0 && d2>=0)
//		return true;
//
//	return false;
//
//}
//
////---------------------------------------------------------------------------
//vfloat CMesh::Det(Vec3D<>& v0, Vec3D<>& v1, Vec3D<>& v2)
////---------------------------------------------------------------------------
//{ // Compute determinant of 3x3 matrix v0,v1,v2
//
//	return 
//
//		v0.x*(v1.y*v2.z-v1.z*v2.y) +
//		v0.y*(v1.z*v2.x-v1.x*v2.z) +
//		v0.z*(v1.x*v2.y-v1.y*v2.x);
//
//}
//
////---------------------------------------------------------------------------
//bool CMesh::IntersectXRay(CFacet* pFacet, vfloat y, vfloat z, Vec3D<>& p, vfloat& pu, vfloat& pv)
////---------------------------------------------------------------------------
//{
//	// compute intersection point P of triangle plane with ray from origin O in direction D
//	// D assumed to be normalized
//	// if no interstion, return false
//	// u and v are barycentric coordinates of the intersection point P = (1 - u - v)A + uB + vC 
//	// see http://www.devmaster.net/wiki/Ray-triangle_intersection
//
//
//	Vec3D<> a = Vertices[pFacet->vi[0]].v;
//	Vec3D<> b = Vertices[pFacet->vi[1]].v;
//	Vec3D<> c = Vertices[pFacet->vi[2]].v;
//	
//	vfloat MinX = a.x < b.x ? (a.x < c.x ? a.x : c.x) : (b.x < c.x ? b.x : c.x) - 1.0;
//	Vec3D<> d(1,0,0);
//	Vec3D<> o(MinX, y, z);
//
//	//Vec3D n = pFacet->n; //((b-a).Cross(c-a)).Normalized();
//	Vec3D<> n = ((b-a).Cross(c-a)).Normalized();
//	//if (n.x > 0){ //flip vertices...
//	//	Vec3D tmp = a;
//	//	a = b;
//	//	b = tmp;
//	//	n = ((b-a).Cross(c-a)).Normalized();
//	//}
//
//	vfloat dn = d.Dot(n);
//	if (fabs(dn)<1E-5)
//		return false; //parallel
//
//	vfloat dist = -(o-a).Dot(n)/dn;
//	Vec3D<> sD = d*dist;
//	p = o+sD;
//
//	vfloat V1, V2, V3;
//	V1 = (b-a).Cross(p-a).Dot(n);
//	V2 = (c-b).Cross(p-b).Dot(n);
//	V3 = (a-c).Cross(p-c).Dot(n);
//	
//	if (V1 >=0 && V2 >=0 && V2 >=0) return true;
//	//if (V1 <=0 && V2 <=0 && V2 <=0) return true;
//	else return false;
//
//}
//
//void CMesh::WeldClose(float Distance)
//{
//
//	int* NumVertHere = new int[Vertices.size()]; //keeps track of how many vertices have been averaged to get here...
//	int* ConsolidateMap = new int[Vertices.size()]; //maps the whole vertex list to the welded vertex list (IE has holes)
//	int* OldNewMap = new int [Vertices.size()]; //maps the old, larger vertex list to the new, smaller one.
//	for (int i=0; i<(int)Vertices.size(); i++){
//		NumVertHere[i] = 1;
//		ConsolidateMap[i] = i;
//		OldNewMap[i] = -1;
//	}
//
//	for (int i=0; i<(int)Facets.size(); i++){ //look through facets so we don't have to do exhaustive On2 search of all vertex combos
//		for (int j=0; j<3; j++){ //look at all three combinations of vertices...
//			int Vi1 = Facets[i].vi[j];
//			int np = -1; while (np != Vi1){ np = Vi1; Vi1 = ConsolidateMap[Vi1]; } //iterates NewMap to get the final value...
//
//			int Vi2 = Facets[i].vi[(j+1)%3];
//			np = -1; while (np != Vi2){ np = Vi2; Vi2 = ConsolidateMap[Vi2]; } //iterates NewMap to get the final value...
//
//			if (Vi1 != Vi2 && (Vertices[Vi1].v-Vertices[Vi2].v).Length() < Distance){ //if they are close enough but not already the same...
//				Vertices[Vi1].v = (Vertices[Vi1].v*NumVertHere[Vi1] + Vertices[Vi2].v*NumVertHere[Vi2]) / (NumVertHere[Vi1]+NumVertHere[Vi2]); //Vertex 1 is the weighted average
//				NumVertHere[Vi1] = NumVertHere[Vi1] + NumVertHere[Vi2]; //count how many vertices make up this point now...
//				
//				ConsolidateMap[Vi2] = Vi1; //effectively deletes Vi2... (points to Vi1)
//			}
//		}
//	}
//
//	std::vector<CFacet> NewFacets;
//	std::vector<CVertex> NewVertices;
//
//	for (int i=0; i<(int)Vertices.size(); i++){
//		if (ConsolidateMap[i] == i) { //if this vertex ended up being part of the welded part
//			NewVertices.push_back(Vertices[i]); //add to the new vertex list
//			OldNewMap[i] = NewVertices.size()-1;
//		}
//	}
//
//	//update the vertex indices
//	for (int i=0; i<(int)Facets.size(); i++){ //look through facets so we don't have to do exhaustive On2 search of all vertex combos
//		for (int j=0; j<3; j++){ //look at all three combinations of vertices...
//			int n = Facets[i].vi[j];
//			int np = -1; while (np != n){ np = n; n = ConsolidateMap[n]; } //iterates NewMap to get the final value...
//
//			Facets[i].vi[j] = OldNewMap[n];
//		}
//		if (!(Facets[i].vi[0] == Facets[i].vi[1] || Facets[i].vi[0] == Facets[i].vi[2] || Facets[i].vi[2] == Facets[i].vi[1])) //if there aren't any the same...
//			NewFacets.push_back(Facets[i]);
//	}
//
//	Facets = NewFacets;
//	Vertices = NewVertices;
//
//	delete [] NumVertHere;
//	NumVertHere = NULL;
//	delete [] ConsolidateMap;
//	ConsolidateMap = NULL;
//	delete [] OldNewMap;
//	OldNewMap = NULL;
//
//	CalcVertNormals(); //re-calculate normals!
//	MeshChanged();
//	
//}
//
//
//void CMesh::RemoveDupLines(void)
//{
//	//first order lines so lower index is first:
//	int tmpHold;
//	for (int i=0; i<(int)Lines.size(); i++){
//		if(Lines[i].vi[0] > Lines[i].vi[1]){
//			tmpHold = Lines[i].vi[0];
//			Lines[i].vi[0] = Lines[i].vi[1];
//			Lines[i].vi[1] = tmpHold;
//		}
//	}
//
//	//now sort them...
//	std::sort(Lines.begin(), Lines.end());
//
//	//iterate up, checking for duplicates and removing...
//	for (int i=1; i<(int)Lines.size(); i++){ //size changes, but that's ok!
//		if (Lines[i] == Lines[i-1]){
//			Lines.erase(Lines.begin()+i);
//			i--;
//		}
//	}
//
//}
//
//
//void CMesh::MeshChanged(void) //invalidates all cached voxelizing info!
//{
//	_TriLayerZ = -1;
//	TriLayer.clear();
//	_TriLineY = -1;
//	TriLine.clear();
//}
//
//void CMesh::FillTriLayer(vfloat z) //fills in TriHeight with all triangles that bridge this plane
//{
//	if(z == _TriLayerZ) return; //if we've already done this...
//	_TriLayerZ = z;
//
//	TriLayer.clear(); //clear previous list
//
//	//add any Facets whose Z coordinates are not all above or all below this Z plane
//	bool IsAbove, IsBelow;
//	std::vector<CFacet>::iterator FIter;
//	int m=0;
//	for (FIter = Facets.begin(); FIter != Facets.end(); FIter++){
//		IsAbove = true; IsBelow = true;
//
//		for (int n=0; n<3; n++){
//			if(Vertices[FIter->vi[n]].v.z > z) IsBelow = false;
//			if(Vertices[FIter->vi[n]].v.z < z) IsAbove = false;
//		}
//		if (!IsAbove && !IsBelow) TriLayer.push_back(m); //if this facet is not fully above or fully below our ZPlane
//		
//		m++;
//	}
//}
//
//void CMesh::FillTriLine(vfloat y, vfloat z) //fills in TriHeight with all triangles that bridge this plane
//{
//	if(y == _TriLineY && z == _TriLayerZ) return; //if we've already done this...
//	FillTriLayer(z); //exits immediately if z is the same as cached...
//	_TriLineY = y;
//
//	TriLine.clear(); //clear previous list
//
//	Vec3D<> p;
//	vfloat pu, pv, V1y, V2y, V3y;
//
//	//add any Facets whose Y coordinates are not all above or all below this y plane (from within those in the Z plane)
////	bool IsAbove, IsBelow;
//	std::vector<int>::iterator ZFIter;
//	for (ZFIter = TriLayer.begin(); ZFIter != TriLayer.end(); ZFIter++){
//		V1y = Vertices[Facets[*ZFIter].vi[0]].v.y;
//		V2y = Vertices[Facets[*ZFIter].vi[1]].v.y;
//		V3y = Vertices[Facets[*ZFIter].vi[2]].v.y;
//		//trivial checks (should get most of them...)
//		if (V1y < y && V2y < y && V3y < y)
//			continue;
//		if (V1y > y && V2y > y && V3y > y)
//			continue;
//
//		if(IntersectXRay(&Facets[*ZFIter], y, z, p, pu, pv)) { //if it intersects
//			if (InsideTri(p, Vertices[Facets[*ZFIter].vi[0]].v, Vertices[Facets[*ZFIter].vi[1]].v, Vertices[Facets[*ZFIter].vi[2]].v)){
//				TriLine.push_back(p.x); 
//			}
//		}
//
//
////		IsAbove = true; IsBelow = true;
//
////		for (int n=0; n<3; n++){
////			if(Vertices[Facets[*ZFIter].vi[n]].v.y > y) IsBelow = false;
////			if(Vertices[Facets[*ZFIter].vi[n]].v.y < y) IsAbove = false;
////		}
////		if (!IsAbove && !IsBelow) TriLine.push_back(*ZFIter); //if this facet is not fully above or fully below our ZPlane
//		
//	}
//
//	std::sort(TriLine.begin(), TriLine.end());
//
//}
//
