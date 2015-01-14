/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_COLLISION_H
#define VX_COLLISION_H

class CVX_Voxel;
#include "Vec3D.h"

//!Defines a potential collision between two voxels
/*!A CVX_Collision object is created for two voxels (presumably in close proximity) that are likely to self-intersect. If they don't intersect, no force is present. Otherwise, an appropriate repelling force is calculated here.

Voxels are assumed to have spherical collision boundaries with a radius of CVX_Collision::envelopeRadius * voxel edge dimension. This radius should be slightly larger than 0.5 to prevent voxels from penetrating too far between other voxels.

updateContactForce() is called to calculate the repelling force. This force is retreived by calling contactForce() with an appropriate voxel pointer to return the force (in the global coordinate system) acting on that voxel as a result of this (potential) collision.
*/
class CVX_Collision
{
public:
	CVX_Collision(CVX_Voxel* v1, CVX_Voxel* v2); //!< Constructor taking the two voxels to watch for collision between. The order is irrelevant. @param[in] v1 One voxel @param[in] v2 The other voxel
	CVX_Collision& operator=(const CVX_Collision& col); //!< Overload "=" operator.
	CVX_Collision(const CVX_Collision& col) {*this = col;} //!< copy constructor.

	Vec3D<float> const contactForce(CVX_Voxel* pVoxel); //!< Returns the repelling force acting on pVoxel from the penetration of the other voxel if their collision boundaries overlap. Otherwise returns a zero force. This force will only be accurate if updateContactForce() has been called since the voxels last moved. @param[in] pVoxel The voxel in question. This should be voxel1() or voxel2() to have any meaning. Otherwise a zero force is returned.
	void updateContactForce(); //!< Updates the state this collision based on the current positions and properties of voxel1() and voxel2(). Call contactForce() with either voxel as the argument to obtain the repelling penetration force (if any) that exisits.

	CVX_Voxel* voxel1() const {return pV1;} //!<One voxel of this potential collision pair.
	CVX_Voxel* voxel2() const {return pV2;} //!<The other voxel of this potential collision pair.

	static float envelopeRadius; //!<The collision envelope radius that these two voxels collide at. Even though voxels are cubic, a spherical collision envelope is used for computation efficiency. Values are multiplied by the length of an edge of the voxel to determine the actual collision radius. Values less than 0.5 or greater than 0.866 are probably of limited use. Prefer around 0.625 (default).

private:
	CVX_Voxel *pV1, *pV2;
	float penetrationStiff; //in N/m for these two voxels
	float dampingC; //damping factor for these two voxels
	Vec3D<float> force;
};

#endif //VX_COLLISION_H
