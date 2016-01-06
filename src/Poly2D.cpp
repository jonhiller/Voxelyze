/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "Poly2D.h"

CPoly2D::CPoly2D(std::vector<float>& coords)
{
	clear();
	if (coords.size()%2 == 1) return; //invalid if odd number of numbers
	for (int i=0; i<(int)coords.size()/2; i++){
		vertices.push_back(Vec2Df(coords[2*i], coords[2*i+1]));
	}

    boundsStale = true;
}

void CPoly2D::clear()
{
	vertices.clear();

	boundsMin = Vec2Df(0,0);
	boundsMax = Vec2Df(0,0);
	boundsStale = true;
}

int CPoly2D::addVertex(Vec2Df& location){
	vertices.push_back(location);
	boundsStale = true;

	return vertexCount()-1; //returns index
}

void CPoly2D::updateBounds(void) const
{
	boundsStale = false;
	if (vertices.size() == 0) {
		boundsMin = Vec2Df();
		boundsMax = Vec2Df();
	} 
	else {
		boundsMin = boundsMax = vertices[0];
	
		for (int i=0; i<(int)vertices.size(); i++) {
			boundsMin = boundsMin.Min(vertices[i]);
			boundsMax = boundsMax.Max(vertices[i]);
		}
	}
}


void CPoly2D::translate(Vec2Df& d)
{
	boundsStale = true;
	int vCount = vertexCount();
	for (int i=0; i<vCount; i++){
		vertices[i]+=d;
	}
}

void CPoly2D::scale(Vec2Df& s)
{
	boundsStale = true;
	int vCount = vertexCount();
	for (int i=0; i<vCount; i++){
		vertices[i].Scale(s);
	}

}

void CPoly2D::rotate(float a)
{
	boundsStale = true;
	int vCount = vertexCount();
	for (int i=0; i<vCount; i++){
		vertices[i].Rot(a);
	}

}


bool CPoly2D::isInside(const Vec2Df &point) const
{
    if (boundsStale) updateBounds();

	if (point.x < boundsMin.x || point.x > boundsMax.x || point.y < boundsMin.y || point.y > boundsMax.y) return false;

	//adapted from http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
	int vCount = vertexCount();

	int i, j, c = 0;
	for (i = 0, j = vCount-1; i < vCount; j = i++) {
		if ( ((vertices[i].y > point.y) != (vertices[j].y > point.y)) && (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x))
			c = !c;
	}
    return c % 2 == 1;
}

float CPoly2D::distanceFromEdge(const Vec2Df &point, const bool ignoreYAxisSegments, Vec2Df* pEdgePointOut) const
{
    if (boundsStale) updateBounds();

	float minDist2 = FLT_MAX;
	Vec2Df minGrad;
    Vec2Df closestPoint;
	int vCount = vertexCount();
	for (int p1=0; p1<vCount; p1++){
		int p2 = p1+1;
		if (p2 == vCount) p2 = 0;

        if (ignoreYAxisSegments && vertices[p1].x == 0 && vertices[p2].x == 0)
            continue;

        Vec2Df thisClosestPoint;
		const float thisMinDist2 = minDistance2Segment(vertices[p1], vertices[p2], point, &thisClosestPoint);
		if (thisMinDist2 < minDist2){
			closestPoint = thisClosestPoint;
			minDist2 = thisMinDist2;
		}
	}

    if (pEdgePointOut) *pEdgePointOut = closestPoint;

	return isInside(point) ? -sqrt(minDist2) : sqrt(minDist2);
}


//adapted from http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment?page=1&tab=votes#tab-top
float CPoly2D::minDistance2Segment(const Vec2Df &v1, const Vec2Df &v2, const Vec2Df &p, Vec2Df* pEdgePointOut) const
{
	Vec2Df closestPoint;

	const float d2 = v1.Dist2(v2);
	if (d2 == 0.0f) closestPoint = v1;
	else {
		// Consider the line extending the segment, parameterized as v1 + t (v2 - v1).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v1) . (v2-v1)] / |v2-v1|^2
		const float t = (p-v1).Dot(v2-v1) / d2;
		if (t < 0.0) closestPoint = v1; // Beyond the 'v1' end of the segment
		else if (t > 1.0) closestPoint = v2;  // Beyond the 'v2' end of the segment
		else closestPoint = v1 + t * (v2 - v1);  // Projection falls on the segment
	}

    if (pEdgePointOut) *pEdgePointOut = closestPoint;

	return p.Dist2(closestPoint);
}

