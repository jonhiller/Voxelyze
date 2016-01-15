/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef LINE2D_H
#define LINE2D_H

#include "Vec2D.h"

class Line2D
{
public:
    Line2D() { a = 0; b = 0; c = 0; }
    Line2D(const Vec2Df &normal, const Vec2Df &pointThrough = Vec2Df(0,0)) {
        const Vec2Df normalNormalized(normal.Normalized());
        a = normalNormalized.x;
        b = normalNormalized.y;
        c = -(a*pointThrough.x + b*pointThrough.y);
    }

    Line2D& operator=(const Line2D& copyFrom) { a = copyFrom.a; b = copyFrom.b; c = copyFrom.c; return *this; } //!< overload equals.
    Line2D(const Line2D& copyFrom) { *this = copyFrom; } //!< Copy constructor.
    inline float distanceFromLine(const Vec2Df &point) const { return a*point.x + b*point.y + c; }
    void mirror(Vec2Df *point) const { //point mirored to other side of line
        const float dFromLine = a*point->x + b*point->y + c;
        *point -= 2 * dFromLine*Vec2Df(a, b); //a, b must be normalized: a^2+b^2=1
    }
    float mirrorToPositive(Vec2Df *point) const { //if point is on positive side of line, don't change it. Otherwise mirror it to positive side. returns the pre-flipping distance.
        const float dFromLine = a*point->x + b*point->y + c;
        if (dFromLine < 0) { *point -= 2 * dFromLine*Vec2Df(a, b); }//a, b must be normalized: a^2+b^2=1
        return dFromLine;
    }

    float a, b, c; //Ax + By + C = 0;
};

#endif //LINE2D_H
