/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_MATERIALLINK_H
#define VX_MATERIALLINK_H

#include "VX_MaterialVoxel.h"

//!Defines the homogenous material properties of a link connecting two voxels.
/*!The constructor takes the two voxel materials and calculates the "best" material properties of a third material that captures the physical behavior. Beam constants are precomputed to quick access during simulation.

If the two input voxel materials are identical then this material is applied and beam constants are precomputed.

If the materials are different a third material is carefully crafted from the two and beam constants precomputed.
*/

class CVX_MaterialLink : public CVX_MaterialVoxel {
	public:
	CVX_MaterialLink(CVX_MaterialVoxel* mat1, CVX_MaterialVoxel* mat2); //!< Creates a link material from the two specified voxel materials. The order is unimportant. @param[in] mat1 voxel material on one side of the link. @param[in] mat2 voxel material on the other side of the link.
	CVX_MaterialLink(const CVX_MaterialLink& VIn) {*this = VIn;} //!< Copy constructor
	virtual CVX_MaterialLink& operator=(const CVX_MaterialLink& VIn); //!< Equals operator

protected:
	virtual bool updateAll(); //!< Updates and recalculates eveything possible (used by inherited classed when material properties have changed)
	virtual bool updateDerived(); //!< Updates all the derived quantities cached as member variables for this and derived classes. (Especially if density, size or elastic modulus changes.)

	CVX_MaterialVoxel *vox1Mat; //!< Constituent material 1 from one voxel
	CVX_MaterialVoxel *vox2Mat; //!< Constituent material 2 from the other voxel

	float _a1; //!< Cached a1 beam constant.
	float _a2; //!< Cached a2 beam constant.
	float _b1; //!< Cached b1 beam constant.
	float _b2; //!< Cached b2 beam constant.
	float _b3; //!< Cached b3 beam constant.
	float _sqA1; //!< Cached sqrt(a1) constant for damping calculations.
	float _sqA2xIp; //!< Cached sqrt(a2*L*L/6) constant for damping calculations.
	float _sqB1; //!< Cached sqrt(b1) constant for damping calculations.
	float _sqB2xFMp; //!< Cached sqrt(b2*L/2) constant for damping calculations.
	float _sqB3xIp; //!< Cached sqrt(b3*L*L/6) constant for damping calculations.
	
	friend class CVoxelyze; //give the main simulation class full access
	friend class CVX_Link; //give links direct access to parameters
};


#endif //VX_MATERIALLINK_H