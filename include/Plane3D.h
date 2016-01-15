/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef PLANE3D_H
#define PLANE3D_H

#include "Vec3D.h"

class Plane3D
{
public:
    Plane3D() { a = 0; b = 0; c = 0; d = 0; }
    Plane3D(const Vec3Df &normal, const Vec3Df &pointThrough = Vec3Df(0, 0, 0)) {
        const Vec3Df normalNormalized(normal.Normalized());
        a = normalNormalized.x;
        b = normalNormalized.y;
        c = normalNormalized.z;
        d = -(a*pointThrough.x + b*pointThrough.y + c*pointThrough.z);
    }
    Plane3D& operator=(const Plane3D& copyFrom) { a = copyFrom.a; b = copyFrom.b; c = copyFrom.c; d = copyFrom.d; return *this; } //!< overload equals.
    Plane3D(const Plane3D& copyFrom) { *this = copyFrom; } //!< Copy constructor.

    inline float distanceFromPlane(const Vec3Df &point) const { return a*point.x + b*point.y + c*point.z + d; }

    void mirror(Vec3Df *point) const { //point mirored to other side of plane
        const float dFromPlane = a*point->x + b*point->y + c*point->z + d;
        *point -= 2 * dFromPlane*Vec3Df(a, b, c); //a, b, c must be normalized: a^2+b^2+c^2=1
    }

    float mirrorToPositive(Vec3Df *point) const { //if point is on positive side of plane, don't change it. Otherwise mirror it to positive side. returns the pre-flipping distance.
        const float dFromPlane = a*point->x + b*point->y + c*point->z + d;
        if (dFromPlane < 0) { *point -= 2 * dFromPlane*Vec3Df(a, b, c); } //a, b, c must be normalized: a^2+b^2+c^2=1
        return dFromPlane;
    }


    void mirrorVector(Vec3Df *vector) const { //direction vector.
        const float dFromPlaneOrigin = a*vector->x + b*vector->y + c*vector->z; //plane at this angle through origin (leave off d)
        *vector -= 2 * dFromPlaneOrigin*Vec3Df(a, b, c); //a, b, c must be normalized: a^2+b^2+c^2=1
    }

    float a, b, c, d; //Ax + By + Cz + D = 0;
};


#endif //PLANE3D_H
