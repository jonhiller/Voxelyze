/*

  Numerical functions for computing minimizers of a least-squares system
  of equations.

  Copyright (C) 2011 Scott Schaefer

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


#ifndef EIGEN_H
#define EIGEN_H

#define TOLERANCE 0.0001f

#include "GeoCommon.h"



/**
 * Uses a jacobi method to return the eigenvectors and eigenvalues 
 * of a 3x3 symmetric matrix.  Note: "a" will be destroyed in this
 * process.  "d" will contain the eigenvalues sorted in order of 
 * decreasing modulus and v will contain the corresponding eigenvectors.
 *
 * @param a the 3x3 symmetric matrix to calculate the eigensystem for
 * @param d the variable to hold the eigenvalues
 * @param v the variables to hold the eigenvectors
 */
void jacobi ( float a[][3], float d[], float v[][3] );

/**
 * Inverts a 3x3 symmetric matrix by computing the pseudo-inverse.
 *
 * @param mat the matrix to invert
 * @param midpoint the point to minimize towards
 * @param rvalue the variable to store the pseudo-inverse in
 * @param w the place to store the inverse of the eigenvalues
 * @param u the place to store the eigenvectors
 */
void matInverse ( float mat[][3], float midpoint[], float rvalue[][3], float w[], float u[][3] );

/**
 * Calculates the L2 norm of the residual (the error)
 * (Transpose[A].A).x = Transpose[A].B
 * 
 * @param a the matrix Transpose[A].A
 * @param b the matrix Transpose[A].B
 * @param btb the value Transpose[B].B
 * @param point the minimizer found
 *
 * @return the error of the minimizer
 */
float calcError ( float a[][3], float b[], float btb, float point[] );

/**
 * Calculates the normal.  This function is not called and doesn't do
 * anything right now.  It was originally meant to orient the normals 
 * correctly that came from the principle eigenvector, but we're flat
 * shading now.
 */
float *calcNormal ( float halfA[], float norm[], float expectedNorm[] );

/**
 * Calculates the minimizer of the given system and returns its error.
 *
 * @param halfA the compressed form of the symmetric matrix Transpose[A].A
 * @param b the matrix Transpose[A].B
 * @param btb the value Transpose[B].B
 * @param midpoint the point to minimize towards
 * @param rvalue the place to store the minimizer
 * @param box the volume bounding the voxel this QEF is for
 *
 * @return the error of the minimizer
 */
float calcPoint ( float halfA[], float b[], float btb, float midpoint[], float rvalue[], BoundingBoxf *box, float *mat );


void qr ( float eqs[][4], int num, float *rvalue );
void qr ( float *mat1, float *mat2, float *rvalue );
void qr ( float eqs[][4], int num = 4, float tol = 0.000001f );

int estimateRank ( float *a );

#endif