/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_LINK_H
#define VX_LINK_H

//#include "VX_Enums.h"
#include "Vec3D.h"
#include "Quat3D.h"

class CVX_Voxel;
class CVX_MaterialLink;



//!Defines a solid link between two adjacent voxels and holds its current state
/*!A CVX_Link calculates the force and moments between two voxels given their relative positions and orientations.

A 12 degree of freedom beam element is utilized to physically model the connection. Pure tension/compression, pure torsion, and slender beam equations are used. Small angle approximations are applied in appropriate scenarios to accelerate computation.

The force and moment for either voxel can be queried, as well as the current axial stress and strain of the link. If voxels are changing size (under thermal or other actuation) then updateRestLength() must be called every timestep. Likewise if the poissons ratio of either voxel is non-zero updateTransverseInfo() must be called likewise.

Information pertaining to one voxel or the other is indicated by the boolean parameter "positiveEnd". If positiveEnd is true, information will be returned for the voxel with the most positive coordinate in the original (undeformed) lattice.
*/
class CVX_Link {
	public:

	//! Defines an axis (X, Y, or Z)
	enum linkAxis {			
		X_AXIS = 0,			//!< X Axis
		Y_AXIS = 1,			//!< Y Axis
		Z_AXIS = 2			//!< Z Axis
	};


	CVX_Link(CVX_Voxel* voxel1, CVX_Voxel* voxel2, CVX_MaterialLink* material); //!< Constructs a link object between two adjacent voxels that represents a solid material connection. The order of voxel1 and voxel2 is unimportant, but the specified linkDirection is interpreted as the originating from voxel1 and pointing to voxel2. A CVX_LinkMaterial representing the desired combination of the two voxel materials must be precomputed and passed as a parameter as well. @param[in] voxel1 One voxel @param[in] voxel2 The other voxel @param[in] material The material properties for this link.
	void reset(); //!< Resets all current state information about this link to the initial value.

	CVX_Voxel* voxel(bool positiveEnd) const {return positiveEnd?pVPos:pVNeg;} //!< Returns a pointer to one of the two voxels that compose this link. @param[in] positiveEnd Specifies which voxel is desired.
	Vec3D<> force(bool positiveEnd) const {return positiveEnd?forcePos:forceNeg;} //!< Returns the current force acting on a voxel due to the position and orientation of the other. @param[in] positiveEnd Specifies which voxel information is desired about.
	Vec3D<> moment(bool positiveEnd) const {return positiveEnd?momentPos:momentNeg;} //!< Returns the current moment acting on a voxel due to the position and orientation of the other. @param[in] positiveEnd Specifies which voxel information is desired about.


	float axialStrain() const {return strain;} //!< returns the current overall axial strain (unitless) between the two voxels.
	float axialStrain(bool positiveEnd) const; //!< Returns the current calculated axial strain of the half of the link contained in the specified voxel. @param[in] positiveEnd Specifies which voxel information is desired about.
	float axialStress() const {return _stress;} //!< returns the current overall true axial stress (MPa) between the two voxels.

	bool isSmallAngle() const {return smallAngle;} //!< Returns true if this link is currently operating with a small angle assumption.
	bool isYielded() const; //!< Returns true if the stress on this bond has ever exceeded its yield stress
	bool isFailed() const; //!< Returns true if the stress on this bond has ever exceeded its failure stress

	float strainEnergy() const; //!< Calculates and return the strain energy of this link according to current forces and moments. (units: Joules, or Kg m^2 / s^2)
	float axialStiffness(); //!< Calculates and returns the current linear axial stiffness of this link at it's current strain.

	void updateForces(); //!< Called every timestep to calculate the forces and moments acting between the two constituent voxels in their current relative positions and orientations.

	void updateRestLength(); //!< Updates the rest length of this voxel. Call this every timestep where the nominal size of either voxel may have changed, due to actuation or thermal expansion.
	void updateTransverseInfo(); //!< Updates information about this voxel pertaining to volumetric deformations. Call this every timestep if the poisson's ratio of the link material is non-zero.


private:
	CVX_Voxel* pVNeg, *pVPos; //voxels in the negative
	Vec3D<> forceNeg, forcePos;
	Vec3D<> momentNeg, momentPos;

	float strain;
	float maxStrain, /*maxStrainRatio,*/ strainOffset; //keep track of the maximums for yield/fail/nonlinear materials (and the ratio of the maximum from 0 to 1 [all positive end strain to all negative end strain])
	float updateStrain(float axialStrain); //updates strainNeg and strainPos according to the provided axial strain. returns current stress as well (MPa)

	typedef int linkState;
	enum linkFlags { //default of each should be zero for easy clearing
		LOCAL_VELOCITY_VALID = 1<<0 //has something changes to render local velocity calculations (for damping) invalid? (this happens when small angle or global base size has changed)
	};
	linkState boolStates;			//single int to store many boolean state values as bit flags according to 
	
	bool isLocalVelocityValid() const {return boolStates & LOCAL_VELOCITY_VALID ? true : false;} //
	void setBoolState(linkFlags flag, bool set=true) {set ? boolStates |= (int)flag : boolStates &= ~(int)flag;}

	//linkAxis axis() {return ax;}
	linkAxis axis;

	//beam parameters
	float a1() const;
	float a2() const;
	float b1() const;
	float b2() const;
	float b3() const;


	CVX_MaterialLink* mat;
	float strainRatio; //ration of Epos to Eneg (EPos/Eneg)

	Vec3D<double> pos2, angle1v, angle2v; //pos1 is always = 0,0,0
	Quat3D<double> angle1, angle2; //this bond in local coordinates. 
	bool smallAngle; //based on compiled precision setting
	double currentRestLength;
	float currentTransverseArea, currentTransverseStrainSum; //so we don't have to re-calculate everytime

	float _stress; //keep this around for convenience

	Quat3D<double> orientLink(/*double restLength*/); //updates pos2, angle1, angle2, and smallAngle. returns the rotation quaternion (after toAxisX) used to get to this orientation

	//unwind a coordinate as if the bond was in the the positive X direction (and back...)
	template <typename T> void toAxisX			(Vec3D<T>* const pV) const {switch (axis){case Y_AXIS: {T tmp = pV->x; pV->x=pV->y; pV->y = -tmp; break;} case Z_AXIS: {T tmp = pV->x; pV->x=pV->z; pV->z = -tmp; break;} default: break;}} //transforms a vec3D in the original orientation of the bond to that as if the bond was in +X direction
	template <typename T> void toAxisX			(Quat3D<T>* const pQ) const {switch (axis){case Y_AXIS: {T tmp = pQ->x; pQ->x=pQ->y; pQ->y = -tmp; break;} case Z_AXIS: {T tmp = pQ->x; pQ->x=pQ->z; pQ->z = -tmp; break;} default: break;}}
	template <typename T> Vec3D<T> toAxisX		(const Vec3D<T>& v)	const {switch (axis){case Y_AXIS: return Vec3D<T>(v.y, -v.x, v.z); case Z_AXIS: return Vec3D<T>(v.z, v.y, -v.x); default: return v;}} //transforms a vec3D in the original orientation of the bond to that as if the bond was in +X direction
	template <typename T> Quat3D<T> toAxisX		(const Quat3D<T>& q)	const {switch (axis){case Y_AXIS: return Quat3D<T>(q.w, q.y, -q.x, q.z); case Z_AXIS: return Quat3D<T>(q.w, q.z, q.y, -q.x); default: return q;}} //transforms a vec3D in the original orientation of the bond to that as if the bond was in +X direction
	template <typename T> void toAxisOriginal	(Vec3D<T>* const pV) const {switch (axis){case Y_AXIS: {T tmp = pV->y; pV->y=pV->x; pV->x = -tmp; break;} case Z_AXIS: {T tmp = pV->z; pV->z=pV->x; pV->x = -tmp; break;} default: break;}}
	template <typename T> void toAxisOriginal	(Quat3D<T>* const pQ) const {switch (axis){case Y_AXIS: {T tmp = pQ->y; pQ->y=pQ->x; pQ->x = -tmp; break;} case Z_AXIS: {T tmp = pQ->z; pQ->z=pQ->x; pQ->x = -tmp; break;} default: break;}}

//	friend class CVX_Voxel;
	friend class CVoxelyze;
	friend class CVX_LinearSolver;
	friend class CVXS_SimGLView; //TEMPORARY
};



#endif //VX_LINK_H



//RESOURCES

//quaternion properties (for reference)
//1) To rotate a vector V, form a quaternion with w = 0; To rotate by Quaternion Q, do Q*V*Q.Conjugate() and trim off the w component.
//2) To do multiple rotations: To Rotate by Q1 THEN Q2, Q2*Q1*V*Q1.Conjugate*Q2.Conjugate(), or make a Qtot = Q2*Q1 and do Qtot*V*Qtot.Conjugate()
//3) Q1*Q1.Conjugate - Identity
//4) To do a reverse rotation Q1, just do Q1.conjugate*V*Q1
//5) An orientation quaternion is really just the relative rotation from the identity quaternion (1,0,0,0) to this orientation.
//6) If an orientation is represented by Q1, to rotate that orientation by Q2 the new orientation is Q2*Q1
//http://www.cprogramming.com/tutorial/3d/quaternions.html


//The giant stiffness matrix VoxCAD uses to model the connections between beams: (Forces, torques) = K*(displacements, angles)
//Sources:
//http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&ved=0CDkQFjAC&url=http%3A%2F%2Fsteel.sejong.ac.kr%2Fdown%2Fpaper%2Fi-paper-13.pdf&ei=fmFuUP_kAeOGyQGIrIDYDQ&usg=AFQjCNG3YZI0bd9OO69VQqV7PTO3KIsEyQ&cad=rja
//http://www.colorado.edu/engineering/cas/courses.d/IFEM.d/
//http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&ved=0CCcQFjAB&url=http%3A%2F%2Fwww.eng.fsu.edu%2F~chandra%2Fcourses%2Feml4536%2FChapter4.ppt&ei=919uUKOnDamryQGc_oEI&usg=AFQjCNER15oW2k4KtNrNR3FdvxNnUbnRVw&cad=rja
/*
F = |k| x
				1	2	3	4	5	6	7	8	9	10	11	12
|F1x|		|	a1						-a1						 |	|X1 |
|F1y|		|		b1y				b2z		-b1y			b2z	 | 	|Y1 |
|F1z|		|			b1z		-b2y			-b1z	-b2y	 | 	|Z1 |
|M1x|		|				a2						-a2			 | 	|TX1|
|M1y|		|			-b2z	2*b3y			b2z		b3y		 | 	|TY1|
|M1z| = 	|		b2y				2*b3z	-b2y			b3z	 |*	|TZ1|
|F2x|		|	-a1						a1						 | 	|X2 |
|F2y|		|		-b1y			-b2z	b1y				-b2z | 	|Y2 |
|F2z|		|			-b1z	b2y				b1z		b2y		 | 	|Z2 |
|M2x|		|				-a2						a2			 | 	|TX2|
|M2y|		|			-b2z	b3y				b2z		2*b3y	 | 	|TY2|
|M2z|		|		b2y				b3z		-b2y			2*b3z| 	|TZ2|

//Confirmed at http://www.eng-tips.com/viewthread.cfm?qid=14924&page=97
k[ 1][ 1] =  E*area/L;				k[ 1][ 7] = -k[1][1];
k[ 2][ 2] =  12.0*E*Iz/(L*L*L);		k[ 2][ 6] =  6.0*E*Iz/(L*L);	k[ 2][ 8] = -k[2][2];		k[ 2][12] =  k[2][6];		
k[ 3][ 3] =  12.0*E*Iy/(L*L*L);		k[ 3][ 5] = -6.0*E*Iy/(L*L);	k[ 3][ 9] = -k[3][3];		k[ 3][11] =  k[3][5];
k[ 4][ 4] =  G*J/L;					k[ 4][10] = -k[4][4];
k[ 5][ 5] =  4.0*E*Iy/L;			k[ 5][ 9] =  6.0*E*Iy/(L*L);	k[ 5][11] =  2.0*E*Iy/L;
k[ 6][ 6] =  4.0*E*Iz/L;			k[ 6][ 8] = -6.0*E*Iz/(L*L);	k[ 6][12] =  2.0*E*Iz/L;
k[ 7][ 7] =  k[1][1];
k[ 8][ 8] =  k[2][2];				k[ 8][12] = -k[2][6];
k[ 9][ 9] =  k[3][3];				k[ 9][11] =  k[5][9];
k[10][10] =  k[4][4];
k[11][11] =  k[5][5];
k[12][12] =  k[6][6];

Likewise, for caculating local damping forces
F= -cv = -zeta*2*sq(MK)*v or (for rotations) -zeta*2*sq(I*wk)*w

		||M1|				|
|M| =	|	|I1|			|	(|M1| is 3x3 identity * mass of voxel 1, etc.)
		|		|M2|		|
		|			|I2|	| (12x12)

					1		2		3		4		5		6		7		8		9		10		11		12
|F1x|			|	a1M1											-a1M1											|	|V1x|
|F1y|			|			b1yM1							b2zI1			-b1yM1							b2zI1	|	|V1y|
|F1z|			|					b1zM1			-b2yI1							-b1zM1			-b2yI1			|	|V1z|
|M1x|			|							a2I1											-a2I1					|	|w1x|
|M1y|			|					-b2zM1			2*b3yI1							b2zM1			b3yI1			|	|w1y|
|M1z| =	zeta *	|			b2yM1							2*b3zI1			-b2yM1							b3zI1	| *	|w1z|
|F2x|			|	-a1M2											a1M2											|	|V2x|
|F2y|			|			-b1yM2							-b2zI2			b1yM2							-b2zI2	|	|V2y|
|F2z|			|					-b1zM2			b2yM2							b1zM2			b2yM2			|	|V2z|
|M2x|			|							-a2I2											a2I2					|	|w2x|
|M2y|			|					-b2zM2			b3yI2							b2zM2			2*b3yI2			|	|w2y|
|M2z|			|			b2yM2							b3zI2			-b2yM2							2*b3zI2	|	|w2z|

a1M1 = 2*sqrt(a1*m1)
a1M2 = 2*sqrt(a1*m2)
b1yM1 = 2*sqrt(b1y*m2)
etc...

(extra negation added to make signs work out)
F1x = -zeta*a1M1*(V2x-V1x)
F1y = -zeta*b1yM1(V2y-V1y) + zeta*b2zI1(w1z+w2z)
F1z = -zeta*b1zM1(V2z-V1z) - zeta*b2yI1(w1y+w2y)
M1x = -zeta*a2I1(w2x-w1x)
M1y = zeta*b2zM1(V2z-V1z) + zeta*b3yI1(2*w1y+w2y)
M1z = -zeta*b2yM1(V2y-V1y) + zeta*b3zI1(2*w1z+w2z)
F2x = zeta*a1M2*(V2x-V1x)
F2y = zeta*b1yM2(V2y-V1y) - zeta*b2zI2(w1z+w2z)
F2z = zeta*b1zM2(V2z-V1z) + zeta*b2yI2(w1y+w2y)
M2x = zeta*a2I2(w2x-w1x)
M2y = zeta*b2zM2(V2z-V1z) + zeta*b3yI2(w1y+2*w2y)
M2z = -zeta*b2yM2(V2y-V1y) + zeta*b3zI2(w1z+2*w2z)

To ease computation, the quantities constant to a voxel have been factored out and are returned by the voxel's dampingMultiplier() function.
These quantites are zeta, 2, sqrt(m), and dt.
Therefore each sqrt(mk) (or equivalent term) has been pre-calculated with sqrt(m) factored out.
For example of a1 term, (and easy extension to b1's), velocity = dPos2.x/dt and k = a1.
Continuing example, factoring out everything constant per voxel we have (-zeta*s*sqrt(m)/dt) * (sqrt(a1)*dPos2.x).
The former quantity is returned by the voxel's dampingMultiplier() function. The latter quantity (for all) is precomputed without the positions and angles of course)


//rayleigh damping:
C = nu*M+Del*K




STRAIN ENERGY of the bond is the area under the force-displacement curve that can be recovered.

PURE TENSION AND COMPRESSION:
For our purposes, even with non-linear materials, it is assumed materials will rebound with their base elastic modulus.

Strain energy is the shaded area, slope is EA/L, therefore bottom edge length is FL/AE.
Area is then Fcurrent*(FL/AE)/2. (or, given EA/L is stiffness k, strain energy = 0.5*F^2/k (or F^2/(2k))
http://www.roymech.co.uk/Useful_Tables/Beams/Strain_Energy.html
http://www.freestudy.co.uk/statics/complex/t5.pdf

Fcurrent
F (n)
|
|       ___
|     /   /|
|   /    /#|
|  /    /##|
| /    /###|
|/    /####|
------------------------
	Disp (m)

PURE TORSION
For rotational, the process is similar: T^2/(2k) where K is units of N-m

BENDING
The strain energy from bending = integral [M(x)^2 dx / (2EI)].
Because we only load the ends of the beam, M(x) = (M2-M1)*x/L+M1 (Linear interoplation, M1 and M2 are the moments at each end).
Plugging and chugging though the integral, we get:

Strain energy = L/(6EI) * (M1^2 + M1*M2 + M2^2)

Note that the sign of M2 must be reversed from that calculated by the stiffness matrix.
This is because a positive "Moment" on a beam (in the moment diagram) is actually two opposite-direction torques on the ends of the beam. ALA http://www.efunda.com/formulae/solid_mechanics/beams/sign_convention.cfm

This means Strain energy = L/(6EI) * (M1^2 - M1*M2' + M2'^2)

SHEAR
Not currently modeled in voxcad.

SUPERPOSITION:
We assume each of these modes is independent, thus total strain energy is simply the sum of them all.

*/
