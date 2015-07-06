/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef MESH3D_H
#define MESH3D_H

#include "Array3Df.h"
#include "Vec3D.h"

class CMesh3D
{
public:
	CMesh3D(void) {clear();}
	CMesh3D(const char* filePath); //only stl supported currently
	CMesh3D(CArray3Df& values, float threshold, float (*density)(Vec3D<float>&, Vec3D<float>*) = 0, bool useDC = false);
	//CMesh3D(CArray3Df& values, CArray3D<Vec3D<float>>& normals, float threshold, float (*density)(Vec3D<float>&, Vec3D<float>*) = 0);
	//CMesh3D(float (*density)(Vec3D<float>&), Vec3D<float>& min, Vec3D<float>& max, int maxDivs, float threshold); //maxDivs = number of voxels in maximum dimension 
	//virtual ~CMesh(){};

	void clear();
	bool load(const char* filePath); //stl
	bool save(const char* filePath); //stl or obj (color) //!< Save the current deformed mesh as an obj file to the path specified. Coloring is not supported yet. @param[in] filePath File path to save the obj file as. Creates or overwrites.

	int addVertex(Vec3D<float>& location); 
	void addTriangle(int vIndex1, int vIndex2, int vIndex3); //assummed ccw (from outside) order
	void addTriangle(Vec3D<float>& p1, Vec3D<float>& p2, Vec3D<float>& p3); //assummed ccw (from outside) order

	int triangleCount() {return (int)(triangles.size()/3);}
	int vertexCount() {return (int)(vertices.size()/3);}
	Vec3Df vertex(int vIndex) {return Vec3Df(&vertices[3*vIndex]);} //returns location of this vertex
	void mergeVertices(float precision); //precision is distance to consider points coincident

	bool isInside(Vec3D<float>* point); //true if inside mesh, false
	float distanceFromSurface(Vec3D<float>* point, float maxDistance, Vec3D<float>* pNormalOut = 0); //returns "blended" distance if within maxDistance. positive for outside, negative for inside.

	void useFaceNormals(); // calc ? 
	void useVertexNormals(float seamAngleDegrees = 0); // seams between triangles less than seamAngleDegrees show as smooth

	Vec3D<float> meshMin() {if (boundsStale) updateBounds(); return boundsMin;}
	Vec3D<float> meshMax() {if (boundsStale) updateBounds(); return boundsMax;}
	Vec3D<float> meshSize() {if (boundsStale) updateBounds(); return boundsMax-boundsMin;}


	void translate(Vec3D<float> d); // translate geometry
	void scale(Vec3D<float> s); // scale geometry
	void rotate(Vec3D<float> ax, float a);
	//void rotX(float a);
	//void rotY(float a);
	//void rotZ(float a);

	void decimate();

	void glDraw(); //!< Executes openGL drawing commands to draw this mesh in an Open GL window if USE_OPEN_GL is defined.

protected:
//	Vec3Df vInterpLow(float iso, Vec3D<float> p1, Vec3D<float> p2, float valp1, float valp2, float (*density)(Vec3D<float>&, Vec3D<float>*), Vec3Df* pNormalOut, float maxError, int maxIter = 5);



	std::vector<float> vertices; //vx1, vy1, vz1, vx2, vy2, vz2, vx3, ...
	std::vector<float> vertexColors; //v1R, v1G, v1B, v2R, v2G, v2B, ... if size != 0, use all vertex colors. Otherwise triangle colors
	std::vector<float> vertexNormals; //t1Nx, t1Ny, t1Nz, t2Nx, t2Ny, t2Nz, ... if size != 0, use all vertex normals. Otherwise triangle normals

	std::vector<int> triangles; //t1v1, t1v2, t1v3, t2v1, t2v2, ... (ccw order)
	std::vector<float> triangleColors; //t1R, t1G, t1B, t2R, t2G, t2B, ... 
	std::vector<float> triangleNormals; //t1Nx, t1Ny, t1Nz, t2Nx, t2Ny, t2Nz, ... (needs updating with mesh deformation)

	std::vector<int> lines; //l1v1, l1v2, l2v1, l2v2, ...

	void meshChanged(); //call whenever geometry has changed (facets added or removed)

private:
	bool normalsByVertex, colorsByVertex; //normals / colors drawn from vertices (vs by facet)
	float currentSeamAngle;

	bool vertexNormalsStale, vertexMergesStale, faceNormalsStale, boundsStale, slicerStale;

	void splitVerticesByAngle(float seamAngleDegrees);

	Vec3D<float> boundsMin, boundsMax; //bounds
	void updateBounds(void);
	
//	// file i/o
	bool loadSTL(std::string& filePath);

	bool saveSTL(std::string& filePath, bool binary = true) const;
	bool saveOBJ(std::string& filePath) const; //!< Save the current deformed mesh as an obj file to the path specified. Coloring is not supported yet. @param[in] filePath File path to save the obj file as. Creates or overwrites.

//
	bool loadBinarySTL(std::string filePath);
	bool loadAsciiSTL(std::string filePath);

	void calcFaceNormals(); //called to update the face normals...
	void calcVertNormals(float seamAngleDegrees = 0); //called once for each new geometry (or after deformed...) (depends on face normals!!!)

	std::vector<int> TriLayer, TriLine; //array of all triangle indices that cross the current z layer, or y line
	std::vector<float> TriInts; //array of all intersection points at the current z height at the curernt Y value
	float _lastZ, _lastY, _lastPad; //height we calculated TriHeight, TriLine at
	void FillTriLayer(float z, float pad = 0.0f); //fills in TriLayer with all triangles that bridge this plane. returns true if re-calculated.
	void FillTriLine(float y, float z, float pad = 0.0); //fills in TriLine with all triangle the bridge the line in the layer. returns true if re-calculated.
	bool FillTriInts(float y, float z, float pad = 0.0f); //fills in TriLine with all triangle the bridge the line in the layer. returns true if re-calculated.
	bool FillCheckTriInts(float y, float z, float pad = 0.0f); //Keeps trying FillTriInts until valid results found


	enum IntersectionType {IT_INSIDE, IT_OUTSIDE, IT_EDGE};
	IntersectionType IntersectXRay(const int TriangleIndex, const float y, const float z, float& XIntersect) const;
	bool GetTriDist(const int TriangleIndex, const Vec3D<float>* pPointIn, float& UOut, float& VOut, Vec3D<float>& ToSurfPointOut) const; //gets distance of provided point to closest UV coordinate of within the triangle. returns true if sensible distance, otherwise false
	float GetTriArea(const int TriangleIndex);



};
#endif //MESH3D_H
