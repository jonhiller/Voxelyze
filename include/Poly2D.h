/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef POLY3D_H
#define POLY3D_H

#include "Vec2D.h"
#include <vector>

class CPoly2D
{
public:
	CPoly2D(void) {clear();}
	CPoly2D(std::vector<float>& coords); //make from list of x1, y1, x2, y2, ... coordinates
	CPoly2D& operator=(const CPoly2D& p)	{vertices = p.vertices; boundsMin = p.boundsMin; boundsMax = p.boundsMax; boundsStale = p.boundsStale; return *this; } //!< overload equals.
	CPoly2D(const CPoly2D& p) {*this = p;} //!< Copy constructor.

	void clear();

	int addVertex(Vec2Df& location); 
	int vertexCount() const {return (int)vertices.size();}
	Vec2Df& vertex(int vertexIndex){return vertices[vertexIndex];}

	bool isInside(Vec2Df* point); //true if inside mesh, false
	bool isInsideConst(Vec2Df* point) const; //true if inside mesh, false
	float distanceFromEdge(Vec2Df* point, Vec2Df* pNormalOut = 0) const; //returns disance from closest point on perimeter of polygon. positive for outside, negative for inside.

	Vec2Df polyMin() {if (boundsStale) updateBounds(); return boundsMin;}
	Vec2Df polyMax() {if (boundsStale) updateBounds(); return boundsMax;}
	Vec2Df polySize() {if (boundsStale) updateBounds(); return boundsMax-boundsMin;}

	void translate(Vec2Df& d); // translate polygon
	void scale(Vec2Df& s); // scale polygon
	void rotate(float a); //rotation angle in radians

	void updateAll(); //Updates all internal state in preparation for calls to isInsideConst();

protected:

	std::vector<Vec2Df> vertices; //v1, v2, ... vN. Don't repeat V1.


private:
	Vec2Df boundsMin, boundsMax; //bounds
	bool boundsStale;
	void updateBounds(void);

	float minDistanceSegment(Vec2Df v1, Vec2Df v2, Vec2Df p, Vec2Df* pNormalOut = 0) const; // Returns minimum distance between point p and a line segment from v1 to v2.

};
#endif //POLY2D_H
