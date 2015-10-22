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


#include "../../include/eigen/eigen.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);a[k][l]=h+s*(g-h*tau);

int method = 3;

// for reducing two upper triangular systems of equations into 1
void qr ( float *mat1, float *mat2, float *rvalue )
{
	int i, j;
	float temp1 [ 8 ] [ 4 ];

	for ( i = 0; i < 4; i++ )
	{
		for ( j = 0; j < i; j++ )
		{
			temp1 [ i ] [ j ] = 0;
			temp1 [ i + 4 ] [ j ] = 0;
		}
		for ( j = i; j < 4; j++ )
		{
			temp1 [ i ] [ j ] = mat1 [ ( 7 * i - i * i ) / 2 + j ];
			temp1 [ i + 4 ] [ j ] = mat2 [ ( 7 * i - i * i ) / 2 + j ];
		}
	}

	qr ( temp1, 8, rvalue );
}

// WARNING: destroys eqs in the process
void qr ( float eqs[][4], int num, float *rvalue )
{
	int i, j, k;

	qr ( eqs, num, 0.000001f );
	for ( i = 0; i < 10; i++ )
	{
		rvalue [ i ] = 0;
	}

	k = 0;
	for ( i = 0; i < num && i < 4; i++ )
	{
		for ( j = i; j < 4; j++ )
		{
			rvalue [ k++ ] = eqs [ i ] [ j ];
		}
	}
}

void qr ( float eqs[][4], int num, float tol )
{
	int i, j, k;
	float a, b, mag, temp;

	for ( i = 0; i < 4 && i < num; i++ )
	{
		for ( j = i + 1; j < num; j++ )
		{
			a = eqs [ i ] [ i ];
			b = eqs [ j ] [ i ];

			if ( fabs ( a ) > 0.000001f || fabs ( b ) > 0.000001f )
			{
				mag = (float)sqrt ( a * a + b * b );
				a /= mag;
				b /= mag;

				for ( k = 0; k < 4; k++ )
				{
					temp = a * eqs [ i ] [ k ] + b * eqs [ j ] [ k ];
					eqs [ j ] [ k ] = b * eqs [ i ] [ k ] - a * eqs [ j ] [ k ];
					eqs [ i ] [ k ] = temp;
				}
			}
		}
		for ( j = i - 1; j >= 0; j-- )
		{
			if ( eqs [ j ] [ j ] < 0.000001f && eqs [ j ] [ j ] > -0.000001f )
			{
				a = eqs [ i ] [ i ];
				b = eqs [ j ] [ i ];

				if ( fabs ( a ) > 0.000001f || fabs ( b ) > 0.000001f )
				{
					mag = (float)sqrt ( a * a + b * b );
					a /= mag;
					b /= mag;

					for ( k = 0; k < 4; k++ )
					{
						temp = a * eqs [ i ] [ k ] + b * eqs [ j ] [ k ];
						eqs [ j ] [ k ] = b * eqs [ i ] [ k ] - a * eqs [ j ] [ k ];
						eqs [ i ] [ k ] = temp;
					}
				}
			}
		}
	}

}

void jacobi ( float u[][3], float d[], float v[][3] )
{
	int j, iq, ip, i;
	float tresh, theta, tau, t, sm, s, h, g, c, b [ 3 ], z [ 3 ];
	float a [ 3 ] [ 3 ];

	a [ 0 ] [ 0 ] = u [ 0 ] [ 0 ];
	a [ 0 ] [ 1 ] = u [ 0 ] [ 1 ];
	a [ 0 ] [ 2 ] = u [ 0 ] [ 2 ];
	a [ 1 ] [ 0 ] = u [ 1 ] [ 0 ];
	a [ 1 ] [ 1 ] = u [ 1 ] [ 1 ];
	a [ 1 ] [ 2 ] = u [ 1 ] [ 2 ];
	a [ 2 ] [ 0 ] = u [ 2 ] [ 0 ];
	a [ 2 ] [ 1 ] = u [ 2 ] [ 1 ];
	a [ 2 ] [ 2 ] = u [ 2 ] [ 2 ];

	for ( ip = 0; ip < 3; ip++ ) 
	{
		for ( iq = 0; iq < 3; iq++ )
		{
			v [ ip ] [ iq ] = 0.0f;
		}
		v [ ip ] [ ip ] = 1.0f;
	}

	for ( ip = 0; ip < 3; ip++ )
	{
		b [ ip ] = a [ ip ] [ ip ];
		d [ ip ] = b [ ip ];
		z [ ip ] = 0.0f;
	}

	for ( i = 1; i <= 50; i++ )
	{
		sm = 0.0f;
		for ( ip = 0; ip < 2; ip++ )
		{
			for ( iq = ip + 1; iq < 3; iq++ )
			{
				sm += (float)fabs ( a [ ip ] [ iq ] );
			}
		}

		if ( sm == 0.0f )
		{
			// sort the stupid things and transpose
			a [ 0 ] [ 0 ] = v [ 0 ] [ 0 ];
			a [ 0 ] [ 1 ] = v [ 1 ] [ 0 ];
			a [ 0 ] [ 2 ] = v [ 2 ] [ 0 ];
			a [ 1 ] [ 0 ] = v [ 0 ] [ 1 ];
			a [ 1 ] [ 1 ] = v [ 1 ] [ 1 ];
			a [ 1 ] [ 2 ] = v [ 2 ] [ 1 ];
			a [ 2 ] [ 0 ] = v [ 0 ] [ 2 ];
			a [ 2 ] [ 1 ] = v [ 1 ] [ 2 ];
			a [ 2 ] [ 2 ] = v [ 2 ] [ 2 ];

			if ( fabs ( d [ 0 ] ) < fabs ( d [ 1 ] ) )
			{
				sm = d [ 0 ];
				d [ 0 ] = d [ 1 ];
				d [ 1 ] = sm;

				sm = a [ 0 ] [ 0 ];
				a [ 0 ] [ 0 ] = a [ 1 ] [ 0 ];
				a [ 1 ] [ 0 ] = sm;
				sm = a [ 0 ] [ 1 ];
				a [ 0 ] [ 1 ] = a [ 1 ] [ 1 ];
				a [ 1 ] [ 1 ] = sm;
				sm = a [ 0 ] [ 2 ];
				a [ 0 ] [ 2 ] = a [ 1 ] [ 2 ];
				a [ 1 ] [ 2 ] = sm;
			}
			if ( fabs ( d [ 1 ] ) < fabs ( d [ 2 ] ) )
			{
				sm = d [ 1 ];
				d [ 1 ] = d [ 2 ];
				d [ 2 ] = sm;

				sm = a [ 1 ] [ 0 ];
				a [ 1] [ 0 ] = a [ 2 ] [ 0 ];
				a [ 2 ] [ 0 ] = sm;
				sm = a [ 1 ] [ 1 ];
				a [ 1 ] [ 1 ] = a [ 2 ] [ 1 ];
				a [ 2 ] [ 1 ] = sm;
				sm = a [ 1 ] [ 2 ];
				a [ 1 ] [ 2 ] = a [ 2 ] [ 2 ];
				a [ 2 ] [ 2 ] = sm;
			}
			if ( fabs ( d [ 0 ] ) < fabs ( d [ 1 ] ) )
			{
				sm = d [ 0 ];
				d [ 0 ] = d [ 1 ];
				d [ 1 ] = sm;

				sm = a [ 0 ] [ 0 ];
				a [ 0 ] [ 0 ] = a [ 1 ] [ 0 ];
				a [ 1 ] [ 0 ] = sm;
				sm = a [ 0 ] [ 1 ];
				a [ 0 ] [ 1 ] = a [ 1 ] [ 1 ];
				a [ 1 ] [ 1 ] = sm;
				sm = a [ 0 ] [ 2 ];
				a [ 0 ] [ 2 ] = a [ 1 ] [ 2 ];
				a [ 1 ] [ 2 ] = sm;
			}

			v [ 0 ] [ 0 ] = a [ 0 ] [ 0 ];
			v [ 0 ] [ 1 ] = a [ 0 ] [ 1 ];
			v [ 0 ] [ 2 ] = a [ 0 ] [ 2 ];
			v [ 1 ] [ 0 ] = a [ 1 ] [ 0 ];
			v [ 1 ] [ 1 ] = a [ 1 ] [ 1 ];
			v [ 1 ] [ 2 ] = a [ 1 ] [ 2 ];
			v [ 2 ] [ 0 ] = a [ 2 ] [ 0 ];
			v [ 2 ] [ 1 ] = a [ 2 ] [ 1 ];
			v [ 2 ] [ 2 ] = a [ 2 ] [ 2 ];

			return;
		}

		if ( i < 4 )
		{
			tresh = 0.2f * sm / 9;
		}
		else
		{
			tresh = 0.0f;
		}

		for ( ip = 0; ip < 2; ip++ )
		{
			for ( iq = ip + 1; iq < 3; iq++ ) 
			{
				g = 100.0f * (float)fabs ( a [ ip ] [ iq ] );
				if ( i > 4 && (float)( fabs ( d [ ip ] ) + g ) == (float)fabs ( d [ ip ] )
					&& (float)( fabs ( d [ iq ] ) + g ) == (float)fabs ( d [ iq ] ) )
				{
					a [ ip ] [ iq ] = 0.0f;
				}
				else
				{
					if ( fabs ( a [ ip ] [ iq ] ) > tresh )
					{
						h = d [ iq ] - d [ ip ];
						if ( (float)( fabs ( h ) + g ) == (float)fabs ( h ) )
						{
							t = ( a [ ip ] [ iq ] ) / h;
						}
						else
						{
							theta = 0.5f * h / ( a [ ip ] [ iq ] );
							t = 1.0f / ( (float)fabs ( theta ) + (float)sqrt ( 1.0f + theta * theta ) );
							if ( theta < 0.0f ) 
							{
								t = -1.0f * t;
							}
						}

						c = 1.0f / (float)sqrt ( 1 + t * t );
						s = t * c;
						tau = s / ( 1.0f + c );
						h = t * a [ ip ] [ iq ];
						z [ ip ] -= h;
						z [ iq ] += h;
						d [ ip ] -= h;
						d [ iq ] += h;
						a [ ip ] [ iq ] = 0.0f;
						for ( j = 0; j <= ip - 1; j++ )
						{
							ROTATE ( a, j, ip, j, iq )
						}
						for ( j = ip + 1; j <= iq - 1; j++ )
						{
							ROTATE ( a, ip, j, j, iq )
						}
						for ( j = iq + 1; j < 3; j++ )
						{
							ROTATE ( a, ip, j, iq, j )
						}
						for ( j = 0; j < 3; j++ )
						{
							ROTATE ( v, j, ip, j, iq )
						}
					}
				}
			}
		}

		for ( ip = 0; ip < 3; ip++ )
		{
			b [ ip ] += z [ ip ];
			d [ ip ] = b [ ip ];
			z [ ip ] = 0.0f;
		}
	}
	printf ( "too many iterations in jacobi\n" );
	exit ( 1 );
}

int estimateRank ( float *a )
{
	float w [ 3 ];
	float u [ 3 ] [ 3 ];
	float mat [ 3 ] [ 3 ];
	int i;

	mat [ 0 ] [ 0 ] = a [ 0 ];
	mat [ 0 ] [ 1 ] = a [ 1 ];
	mat [ 0 ] [ 2 ] = a [ 2 ];
	mat [ 1 ] [ 1 ] = a [ 3 ];
	mat [ 1 ] [ 2 ] = a [ 4 ];
	mat [ 2 ] [ 2 ] = a [ 5 ];
	mat [ 1 ] [ 0 ] = a [ 1 ];
	mat [ 2 ] [ 0 ] = a [ 2 ];
	mat [ 2 ] [ 1 ] = a [ 4 ];

	jacobi ( mat, w, u );

	if ( w [ 0 ] == 0.0f )
	{
		return 0;
	}
	else
	{
		for ( i = 1; i < 3; i++ )
		{
			if ( w [ i ] < 0.1f )
			{
				return i;
			}
		}

		return 3;
	}

}

void matInverse ( float mat[][3], float midpoint[], float rvalue[][3], float w[], float u[][3] )
{
	// there is an implicit assumption that mat is symmetric and real
	// U and V in the SVD will then be the same matrix whose rows are the eigenvectors of mat
	// W will just be the eigenvalues of mat
//	float w [ 3 ];
//	float u [ 3 ] [ 3 ];
	int i;

	jacobi ( mat, w, u );

	if ( w [ 0 ] == 0.0f )
	{
//		printf ( "error: largest eigenvalue is 0!\n" );
	}
	else
	{
		for ( i = 1; i < 3; i++ )
		{
			if ( w [ i ] < 0.001f ) // / w [ 0 ] < TOLERANCE )
			{
					w [ i ] = 0;
			}
			else
			{
				w [ i ] = 1.0f / w [ i ];
			}
		}
		w [ 0 ] = 1.0f / w [ 0 ];
	}

	rvalue [ 0 ] [ 0 ] = w [ 0 ] * u [ 0 ] [ 0 ] * u [ 0 ] [ 0 ] +
					w [ 1 ] * u [ 1 ] [ 0 ] * u [ 1 ] [ 0 ] +
					w [ 2 ] * u [ 2 ] [ 0 ] * u [ 2 ] [ 0 ];
	rvalue [ 0 ] [ 1 ] = w [ 0 ] * u [ 0 ] [ 0 ] * u [ 0 ] [ 1 ] +
					w [ 1 ] * u [ 1 ] [ 0 ] * u [ 1 ] [ 1 ] +
					w [ 2 ] * u [ 2 ] [ 0 ] * u [ 2 ] [ 1 ];
	rvalue [ 0 ] [ 2 ] = w [ 0 ] * u [ 0 ] [ 0 ] * u [ 0 ] [ 2 ] +
					w [ 1 ] * u [ 1 ] [ 0 ] * u [ 1 ] [ 2 ] +
					w [ 2 ] * u [ 2 ] [ 0 ] * u [ 2 ] [ 2 ];
	rvalue [ 1 ] [ 0 ] = w [ 0 ] * u [ 0 ] [ 1 ] * u [ 0 ] [ 0 ] +
					w [ 1 ] * u [ 1 ] [ 1 ] * u [ 1 ] [ 0 ] +
					w [ 2 ] * u [ 2 ] [ 1 ] * u [ 2 ] [ 0 ];
	rvalue [ 1 ] [ 1 ] = w [ 0 ] * u [ 0 ] [ 1 ] * u [ 0 ] [ 1 ] +
					w [ 1 ] * u [ 1 ] [ 1 ] * u [ 1 ] [ 1 ] +
					w [ 2 ] * u [ 2 ] [ 1 ] * u [ 2 ] [ 1 ];
	rvalue [ 1 ] [ 2 ] = w [ 0 ] * u [ 0 ] [ 1 ] * u [ 0 ] [ 2 ] +
					w [ 1 ] * u [ 1 ] [ 1 ] * u [ 1 ] [ 2 ] +
					w [ 2 ] * u [ 2 ] [ 1 ] * u [ 2 ] [ 2 ];
	rvalue [ 2 ] [ 0 ] = w [ 0 ] * u [ 0 ] [ 2 ] * u [ 0 ] [ 0 ] +
					w [ 1 ] * u [ 1 ] [ 2 ] * u [ 1 ] [ 0 ] +
					w [ 2 ] * u [ 2 ] [ 2 ] * u [ 2 ] [ 0 ];
	rvalue [ 2 ] [ 1 ] = w [ 0 ] * u [ 0 ] [ 2 ] * u [ 0 ] [ 1 ] +
					w [ 1 ] * u [ 1 ] [ 2 ] * u [ 1 ] [ 1 ] +
					w [ 2 ] * u [ 2 ] [ 2 ] * u [ 2 ] [ 1 ];
	rvalue [ 2 ] [ 2 ] = w [ 0 ] * u [ 0 ] [ 2 ] * u [ 0 ] [ 2 ] +
					w [ 1 ] * u [ 1 ] [ 2 ] * u [ 1 ] [ 2 ] +
					w [ 2 ] * u [ 2 ] [ 2 ] * u [ 2 ] [ 2 ];
}

float calcError ( float a[][3], float b[], float btb, float point[] )
{
	float rvalue = btb;
 
	rvalue += -2.0f * ( point [ 0 ] * b [ 0 ] + point [ 1 ] * b [ 1 ] + point [ 2 ] * b [ 2 ] );
	rvalue += point [ 0 ] * ( a [ 0 ] [ 0 ] * point [ 0 ] + a [ 0 ] [ 1 ] * point [ 1 ] + a [ 0 ] [ 2 ] * point [ 2 ] );
	rvalue += point [ 1 ] * ( a [ 1 ] [ 0 ] * point [ 0 ] + a [ 1 ] [ 1 ] * point [ 1 ] + a [ 1 ] [ 2 ] * point [ 2 ] );
	rvalue += point [ 2 ] * ( a [ 2 ] [ 0 ] * point [ 0 ] + a [ 2 ] [ 1 ] * point [ 1 ] + a [ 2 ] [ 2 ] * point [ 2 ] );

	return rvalue;
}

float *calcNormal ( float halfA[], float norm[], float expectedNorm[] )
{
/*
	float a [ 3 ] [ 3 ];
	float w [ 3 ];
	float u [ 3 ] [ 3 ];

	a [ 0 ] [ 0 ] = halfA [ 0 ];
	a [ 0 ] [ 1 ] = halfA [ 1 ];
	a [ 0 ] [ 2 ] = halfA [ 2 ];
	a [ 1 ] [ 1 ] = halfA [ 3 ];
	a [ 1 ] [ 2 ] = halfA [ 4 ];
	a [ 1 ] [ 0 ] = halfA [ 1 ];
	a [ 2 ] [ 0 ] = halfA [ 2 ];
	a [ 2 ] [ 1 ] = halfA [ 4 ];
	a [ 2 ] [ 2 ] = halfA [ 5 ];

	jacobi ( a, w, u );

	if ( u [ 1 ] != 0 )
	{
		if ( w [ 1 ] / w [ 0 ] > 0.2f )
		{
			// two dominant eigen values, just return the expectedNorm
			norm [ 0 ] = expectedNorm [ 0 ];
			norm [ 1 ] = expectedNorm [ 1 ];
			norm [ 2 ] = expectedNorm [ 2 ];
			return;
		}
	}

	norm [ 0 ] = u [ 0 ] [ 0 ];
	norm [ 1 ] = u [ 0 ] [ 1 ];
	norm [ 2 ] = u [ 0 ] [ 2 ];
*/
	float dot = norm [ 0 ] * expectedNorm [ 0 ] + norm [ 1 ] * expectedNorm [ 1 ] +
				norm [ 2 ] * expectedNorm [ 2 ];

	if ( dot < 0 )
	{
		norm [ 0 ] *= -1.0f;
		norm [ 1 ] *= -1.0f;
		norm [ 2 ] *= -1.0f;

		dot *= -1.0f;
	}

	if ( dot < 0.707f )
	{
		return expectedNorm;
	}
	else
	{
		return norm;
	}
}

void descent ( float A[][3], float B[], float guess[], BoundingBoxf *box )
{
	int i;
	float r [ 3 ];
	float delta, delta0;
	int n = 10;
	float alpha, div;
	float newPoint [ 3 ];
	float c;
	float store [ 3 ];

	store [ 0 ] = guess [ 0 ];
	store [ 1 ] = guess [ 1 ];
	store [ 2 ] = guess [ 2 ];

	if ( method == 2 || method == 0 )
	{

	i = 0;
	r [ 0 ] = B [ 0 ] - ( A [ 0 ] [ 0 ] * guess [ 0 ] + A [ 0 ] [ 1 ] * guess [ 1 ] + A [ 0 ] [ 2 ] * guess [ 2 ] );
	r [ 1 ] = B [ 1 ] - ( A [ 1 ] [ 0 ] * guess [ 0 ] + A [ 1 ] [ 1 ] * guess [ 1 ] + A [ 1 ] [ 2 ] * guess [ 2 ] );
	r [ 2 ] = B [ 2 ] - ( A [ 2 ] [ 0 ] * guess [ 0 ] + A [ 2 ] [ 1 ] * guess [ 1 ] + A [ 2 ] [ 2 ] * guess [ 2 ] );

	delta = r [ 0 ] * r [ 0 ] + r [ 1 ] * r [ 1 ] + r [ 2 ] * r [ 2 ];
	delta0 = delta * TOLERANCE * TOLERANCE;

	while ( i < n && delta > delta0 )
	{
		div = r [ 0 ] * ( A [ 0 ] [ 0 ] * r [ 0 ] + A [ 0 ] [ 1 ] * r [ 1 ] + A [ 0 ] [ 2 ] * r [ 2 ] );
		div += r [ 1 ] * ( A [ 1 ] [ 0 ] * r [ 0 ] + A [ 1 ] [ 1 ] * r [ 1 ] + A [ 1 ] [ 2 ] * r [ 2 ] );
		div += r [ 2 ] * ( A [ 2 ] [ 0 ] * r [ 0 ] + A [ 2 ] [ 1 ] * r [ 1 ] + A [ 2 ] [ 2 ] * r [ 2 ] );

		if ( fabs ( div ) < 0.0000001f )
		{
			break;
		}

		alpha = delta / div;

		newPoint [ 0 ] = guess [ 0 ] + alpha * r [ 0 ];
		newPoint [ 1 ] = guess [ 1 ] + alpha * r [ 1 ];
		newPoint [ 2 ] = guess [ 2 ] + alpha * r [ 2 ];

		guess [ 0 ] = newPoint [ 0 ];
		guess [ 1 ] = newPoint [ 1 ];
		guess [ 2 ] = newPoint [ 2 ];

		r [ 0 ] = B [ 0 ] - ( A [ 0 ] [ 0 ] * guess [ 0 ] + A [ 0 ] [ 1 ] * guess [ 1 ] + A [ 0 ] [ 2 ] * guess [ 2 ] );
		r [ 1 ] = B [ 1 ] - ( A [ 1 ] [ 0 ] * guess [ 0 ] + A [ 1 ] [ 1 ] * guess [ 1 ] + A [ 1 ] [ 2 ] * guess [ 2 ] );
		r [ 2 ] = B [ 2 ] - ( A [ 2 ] [ 0 ] * guess [ 0 ] + A [ 2 ] [ 1 ] * guess [ 1 ] + A [ 2 ] [ 2 ] * guess [ 2 ] );

		delta = r [ 0 ] * r [ 0 ] + r [ 1 ] * r [ 1 ] + r [ 2 ] * r [ 2 ];

		i++;
	}

	if ( guess [ 0 ] >= box->begin.x && guess [ 0 ] <= box->end.x && 
		guess [ 1 ] >= box->begin.y && guess [ 1 ] <= box->end.y &&
		guess [ 2 ] >= box->begin.z && guess [ 2 ] <= box->end.z )
	{
		return;
	}
	}

	if ( method == 0 || method == 1 )
	{
		c = A [ 0 ] [ 0 ] + A [ 1 ] [ 1 ] + A [ 2 ] [ 2 ];
		if ( c == 0 )
		{
			return;
		}
		c = ( 0.75f / c );

		guess [ 0 ] = store [ 0 ];
		guess [ 1 ] = store [ 1 ];
		guess [ 2 ] = store [ 2 ];

		r [ 0 ] = B [ 0 ] - ( A [ 0 ] [ 0 ] * guess [ 0 ] + A [ 0 ] [ 1 ] * guess [ 1 ] + A [ 0 ] [ 2 ] * guess [ 2 ] );
		r [ 1 ] = B [ 1 ] - ( A [ 1 ] [ 0 ] * guess [ 0 ] + A [ 1 ] [ 1 ] * guess [ 1 ] + A [ 1 ] [ 2 ] * guess [ 2 ] );
		r [ 2 ] = B [ 2 ] - ( A [ 2 ] [ 0 ] * guess [ 0 ] + A [ 2 ] [ 1 ] * guess [ 1 ] + A [ 2 ] [ 2 ] * guess [ 2 ] );

		for ( i = 0; i < n; i++ )
		{
			guess [ 0 ] = guess [ 0 ] + c * r [ 0 ];
			guess [ 1 ] = guess [ 1 ] + c * r [ 1 ];
			guess [ 2 ] = guess [ 2 ] + c * r [ 2 ];

			r [ 0 ] = B [ 0 ] - ( A [ 0 ] [ 0 ] * guess [ 0 ] + A [ 0 ] [ 1 ] * guess [ 1 ] + A [ 0 ] [ 2 ] * guess [ 2 ] );
			r [ 1 ] = B [ 1 ] - ( A [ 1 ] [ 0 ] * guess [ 0 ] + A [ 1 ] [ 1 ] * guess [ 1 ] + A [ 1 ] [ 2 ] * guess [ 2 ] );
			r [ 2 ] = B [ 2 ] - ( A [ 2 ] [ 0 ] * guess [ 0 ] + A [ 2 ] [ 1 ] * guess [ 1 ] + A [ 2 ] [ 2 ] * guess [ 2 ] );
		}
	}
/*
	if ( guess [ 0 ] > store [ 0 ] + 1 || guess [ 0 ] < store [ 0 ] - 1 ||
		guess [ 1 ] > store [ 1 ] + 1 || guess [ 1 ] < store [ 1 ] - 1 ||
		guess [ 2 ] > store [ 2 ] + 1 || guess [ 2 ] < store [ 2 ] - 1 )
	{
		printf ( "water let point go from %f,%f,%f to %f,%f,%f\n", 
			store [ 0 ], store [ 1 ], store [ 2 ], guess [ 0 ], guess [ 1 ], guess [ 2 ] );
		printf ( "A is %f,%f,%f %f,%f,%f %f,%f,%f\n", A [ 0 ] [ 0 ], A [ 0 ] [ 1 ], A [ 0 ] [ 2 ],
			A [ 1 ] [ 0 ] , A [ 1 ] [ 1 ], A [ 1 ] [ 2 ], A [ 2 ] [ 0 ], A [ 2 ] [ 1 ], A [ 2 ] [ 2 ] );
		printf ( "B is %f,%f,%f\n", B [ 0 ], B [ 1 ], B [ 2 ] );
		printf ( "bounding box is %f,%f,%f to %f,%f,%f\n", 
			box->begin.x, box->begin.y, box->begin.z, box->end.x, box->end.y, box->end.z );
	}
*/
}

float calcPoint ( float halfA[], float b[], float btb, float midpoint[], float rvalue[], BoundingBoxf *box, float *mat )
{
	float newB [ 3 ];
	float a [ 3 ] [ 3 ];
	float inv [ 3 ] [ 3 ];
	float w [ 3 ];
	float u [ 3 ] [ 3 ];

	a [ 0 ] [ 0 ] = halfA [ 0 ];
	a [ 0 ] [ 1 ] = halfA [ 1 ];
	a [ 0 ] [ 2 ] = halfA [ 2 ];
	a [ 1 ] [ 1 ] = halfA [ 3 ];
	a [ 1 ] [ 2 ] = halfA [ 4 ];
	a [ 1 ] [ 0 ] = halfA [ 1 ];
	a [ 2 ] [ 0 ] = halfA [ 2 ];
	a [ 2 ] [ 1 ] = halfA [ 4 ];
	a [ 2 ] [ 2 ] = halfA [ 5 ];

	switch ( method )
	{
	case 0:
	case 1:
	case 2:
		rvalue [ 0 ] = midpoint [ 0 ];
		rvalue [ 1 ] = midpoint [ 1 ];
		rvalue [ 2 ] = midpoint [ 2 ];

		descent ( a, b, rvalue, box );
		return calcError ( a, b, btb, rvalue );
		break;
	case 3:
		matInverse ( a, midpoint, inv, w, u );


		newB [ 0 ] = b [ 0 ] - a [ 0 ] [ 0 ] * midpoint [ 0 ] - a [ 0 ] [ 1 ] * midpoint [ 1 ] - a [ 0 ] [ 2 ] * midpoint [ 2 ];
		newB [ 1 ] = b [ 1 ] - a [ 1 ] [ 0 ] * midpoint [ 0 ] - a [ 1 ] [ 1 ] * midpoint [ 1 ] - a [ 1 ] [ 2 ] * midpoint [ 2 ];
		newB [ 2 ] = b [ 2 ] - a [ 2 ] [ 0 ] * midpoint [ 0 ] - a [ 2 ] [ 1 ] * midpoint [ 1 ] - a [ 2 ] [ 2 ] * midpoint [ 2 ];

		rvalue [ 0 ] = inv [ 0 ] [ 0 ] * newB [ 0 ] + inv [ 1 ] [ 0 ] * newB [ 1 ] + inv [ 2 ] [ 0 ] * newB [ 2 ] + midpoint [ 0 ];
		rvalue [ 1 ] = inv [ 0 ] [ 1 ] * newB [ 0 ] + inv [ 1 ] [ 1 ] * newB [ 1 ] + inv [ 2 ] [ 1 ] * newB [ 2 ] + midpoint [ 1 ];
		rvalue [ 2 ] = inv [ 0 ] [ 2 ] * newB [ 0 ] + inv [ 1 ] [ 2 ] * newB [ 1 ] + inv [ 2 ] [ 2 ] * newB [ 2 ] + midpoint [ 2 ];
		return calcError ( a, b, btb, rvalue );
		break;
	case 4:
		method = 3;
		calcPoint ( halfA, b, btb, midpoint, rvalue, box, mat );
		method = 4;
/*
		int rank;
		float eqs [ 4 ] [ 4 ];

		// form the square matrix
		eqs [ 0 ] [ 0 ] = mat [ 0 ];
		eqs [ 0 ] [ 1 ] = mat [ 1 ];
		eqs [ 0 ] [ 2 ] = mat [ 2 ];
		eqs [ 0 ] [ 3 ] = mat [ 3 ];
		eqs [ 1 ] [ 1 ] = mat [ 4 ];
		eqs [ 1 ] [ 2 ] = mat [ 5 ];
		eqs [ 1 ] [ 3 ] = mat [ 6 ];
		eqs [ 2 ] [ 2 ] = mat [ 7 ];
		eqs [ 2 ] [ 3 ] = mat [ 8 ];
		eqs [ 3 ] [ 3 ] = mat [ 9 ];
		eqs [ 1 ] [ 0 ] = eqs [ 2 ] [ 0 ] = eqs [ 2 ] [ 1 ] = eqs [ 3 ] [ 0 ] = eqs [ 3 ] [ 1 ] = eqs [ 3 ] [ 2 ] = 0;

		// compute the new QR decomposition and rank
		rank = qr ( eqs );

		method = 2;
		calcPoint ( halfA, b, btb, midpoint, rvalue, box, mat );
		method = 4;
/*
		if ( rank == 0 )
		{
			// it's zero, no equations
			rvalue [ 0 ] = midpoint [ 0 ];
			rvalue [ 1 ] = midpoint [ 1 ];
			rvalue [ 2 ] = midpoint [ 2 ];
		}
		else 
		{
			if ( rank == 1 )
			{
				// one equation, it's a plane
				float temp = ( eqs [ 0 ] [ 0 ] * midpoint [ 0 ] + eqs [ 0 ] [ 1 ] * midpoint [ 1 ] + eqs [ 0 ] [ 2 ] * midpoint [ 2 ] - eqs [ 0 ] [ 3 ] ) /
							( eqs [ 0 ] [ 0 ] * eqs [ 0 ] [ 0 ] + eqs [ 0 ] [ 1 ] * eqs [ 0 ] [ 1 ] + eqs [ 0 ] [ 2 ] * eqs [ 0 ] [ 2 ] );

				rvalue [ 0 ] = midpoint [ 0 ] - temp * eqs [ 0 ] [ 0 ];
				rvalue [ 1 ] = midpoint [ 1 ] - temp * eqs [ 0 ] [ 1 ];
				rvalue [ 2 ] = midpoint [ 2 ] - temp * eqs [ 0 ] [ 2 ];
			}
			else
			{
				if ( rank == 2 )
				{
					// two equations, it's a line
					float a, b, c, d, e, f, g;

					// reduce back to upper triangular
					qr ( eqs, 2, 0.000001f );

					a = eqs [ 0 ] [ 0 ];
					b = eqs [ 0 ] [ 1 ];
					c = eqs [ 0 ] [ 2 ];
					d = eqs [ 0 ] [ 3 ];
					e = eqs [ 1 ] [ 1 ];
					f = eqs [ 1 ] [ 2 ];
					g = eqs [ 1 ] [ 3 ];

					// solved using the equations
					// ax + by + cz = d
					//      ey + fz = g
					// minimize (x-px)^2 + (y-py)^2 + (z-pz)^2
					if ( a > 0.000001f || a < -0.000001f )
					{
						if ( e > 0.00000f || e < -0.000001f )
						{
							rvalue [ 2 ] = ( -1 * b * d * e * f + ( a * a + b * b ) * g * f + c * e * ( d * e - b * g ) +
								a * e * ( ( b * f - c * e ) * midpoint [ 0 ] - a * f * midpoint [ 1 ] + a * e * midpoint [ 2 ] ) ) /
								( a * a * ( e * e + f * f ) + ( c * e - b * f ) * ( c * e - b * f ) );
							rvalue [ 1 ] = ( g - f * rvalue [ 2 ] ) / e;
							rvalue [ 0 ] = ( d - b * rvalue [ 1 ] - c * rvalue [ 2 ] ) / a;
						}
						else
						{
							// slightly degenerate case where e==0
							rvalue [ 2 ] = g / f;
							rvalue [ 1 ] = ( b * d * f - b * c * g - a * b * f * midpoint [ 0 ] + a * a * f * midpoint [ 1 ] ) /
								( a * a * f + b * b * f );
							rvalue [ 0 ] = ( d - b * rvalue [ 1 ] - c * rvalue [ 2 ] ) / a;
						}
					}
					else
					{
						// degenerate case where a==0 so e == 0 (upper triangular)

						rvalue [ 2 ] = g / f;
						rvalue [ 1 ] = ( d - c * rvalue [ 2 ] ) / b;
						rvalue [ 0 ] = midpoint [ 0 ];
					}

				}
				else
				{
					// must be three equations or more now... solve using back-substitution
					rvalue [ 2 ] = mat [ 8 ] / mat [ 7 ];
					rvalue [ 1 ] = ( mat [ 6 ] - mat [ 5 ] * rvalue [ 2 ] ) / mat [ 4 ];
					rvalue [ 0 ] = ( mat [ 3 ] - mat [ 2 ] * rvalue [ 2 ] - mat [ 1 ] * rvalue [ 1 ] ) / mat [ 0 ];
				}
			}
		}
*/

		float ret;
		float tmp;

		ret = mat [ 9 ] * mat [ 9 ];

		tmp = mat [ 0 ] * rvalue [ 0 ] + mat [ 1 ] * rvalue [ 1 ] + mat [ 2 ] * rvalue [ 2 ] - mat [ 3 ];
		ret += tmp * tmp;

		tmp = mat [ 4 ] * rvalue [ 1 ] + mat [ 5 ] * rvalue [ 2 ] - mat [ 6 ];
		ret += tmp * tmp;

		tmp = mat [ 7 ] * rvalue [ 2 ] - mat [ 8 ];
		ret += tmp * tmp;

		return ret;

		break;
	case 5:
		rvalue [ 0 ] = midpoint [ 0 ];
		rvalue [ 1 ] = midpoint [ 1 ];
		rvalue [ 2 ] = midpoint [ 2 ];

		return calcError ( a, b, btb, rvalue );
	}

	return 0 ;
}
