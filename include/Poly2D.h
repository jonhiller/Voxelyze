/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef POLY2D_H
#define POLY2D_H

#include "Vec2D.h"
#include <vector>

class CPoly2D
{
public:
	CPoly2D(void) {clear();}
	CPoly2D(std::vector<float>& coords); //make from list of x1, y1, x2, y2, ... coordinates
    CPoly2D& operator=(const CPoly2D& p); //!< overload equals.
	CPoly2D(const CPoly2D& p) {*this = p;} //!< Copy constructor.

	void clear();

	int addVertex(Vec2Df& location); 
	int vertexCount() const {return (int)vertices.size();}
	const Vec2Df& vertex(int vertexIndex) const {return vertices[vertexIndex];}

	bool isInside(const Vec2Df &point) const; //true if inside mesh, false
	float distanceFromEdge(const Vec2Df &point, Vec2Df* pNormalOut = 0) const; //returns distance from closest point on perimeter of polygon. positive for outside, negative for inside.

	Vec2Df polyMin() const {if (boundsStale) updateBounds(); return boundsMin;}
	Vec2Df polyMax() const {if (boundsStale) updateBounds(); return boundsMax;}
	Vec2Df polySize() const {if (boundsStale) updateBounds(); return boundsMax-boundsMin;}

	void translate(Vec2Df& d); // translate polygon
	void scale(Vec2Df& s); // scale polygon
	void rotate(float a); //rotation angle in radians

    std::vector<bool> ignoreSegmentsForClosestPoint; //index is for the segment between the correspongind vertex and the next. (same length as vertices, since the last degment is implied returning to the first point in the vertices vector.)

protected:

	std::vector<Vec2Df> vertices; //v1, v2, ... vN. Don't repeat V1.

private:
	mutable Vec2Df boundsMin, boundsMax; //bounds. mutable to make polyMin(), polyMax() const as they should be from external perspective while still enabling lazy bounds computation.
	mutable bool boundsStale;
	void updateBounds(void) const;

	float minDistance2Segment(const Vec2Df &v1, const Vec2Df &v2, const Vec2Df &p, Vec2Df* pEdgePointOut = 0) const; // Returns minimum distance between point p and a line segment from v1 to v2.

};
#endif //POLY2D_H
