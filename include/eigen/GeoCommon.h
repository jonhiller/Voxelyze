/*

  Structure definitions for points and triangles.

  Copyright (C) 2011  Tao Ju

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public License
  (LGPL) as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef GEOCOMMON_H
#define GEOCOMMON_H

#define UCHAR unsigned char
#define USHORT unsigned short

#define USE_MINIMIZER

// 3d point with integer coordinates
typedef struct
{
	int x, y, z;
} Point3i;

typedef struct
{
	Point3i begin;
	Point3i end;
} BoundingBox;

// triangle that points to three vertices
typedef struct 
{
	float vt[3][3] ;
} Triangle;

struct TriangleList
{
	float vt[3][3] ;
	TriangleList* next ;
};

struct VertexList
{
	float vt[3] ;
	VertexList* next ;
};

struct IndexedTriangleList
{
	int vt[3] ;
	IndexedTriangleList* next ;
};

// 3d point with float coordinates
typedef struct
{
	float x, y, z;
} Point3f;

typedef struct
{
	Point3f begin;
	Point3f end;
} BoundingBoxf;


#endif