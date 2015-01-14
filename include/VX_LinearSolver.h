/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef CVX_LINEARSOLVER_H
#define CVX_LINEARSOLVER_H

class CVoxelyze;
#include <string>
#include <vector>

#ifdef PARDISO_5
#ifdef _WIN32
	#pragma comment (lib, "libpardiso500-WIN-X86-64.lib") //link to the Pardiso library
#endif
extern "C" void pardisoinit (void* pt, int* mtype, int* solver, int* iparm, double* dparm, int* error);
extern "C" void pardiso (void* pt, int* maxfct, int* mnum, int* mtype, int* phase, int* n, double* a, int* ia, int* ja, int* perm, int* nrhs, int* iparm, int* msglvl, double* b, double* x, int* error, double* dparm);
#endif

//! A linear solver for Voxelyze.
/*!
Although voxelyze is fundamentally a dynamic non-linear materials simulator, sometimes a linear solution is still appropriate for a given problem. This class provides the means to do a one-time linear solve of the system in far less time than the equivalent doTimeStep() sequence.

The simulation is currently always linearized about the voxels' nominal positions, so only the elastic modulus of materials is used. Density, poissons ratio, etc. are all disregarded.

Currently only the pardiso solver is supported, although other more permissive solvers are in the works. See www.pardiso-project.com for the appropriate license (free for academic use) and libraries. Define PARDISO_5 in the preprocessor to compile in this pardiso support.

Because solver execution time can be lengthy, a rudimentary set of status variables is maintained during the solve process. They can be accessed safely while the process is running. Likewise cancelFlag can be set to true and the solver will abort execution as soon as it can. Note that this can still be a lengthy wait.
*/
class CVX_LinearSolver
{
public:
	CVX_LinearSolver(CVoxelyze* voxelyze); //!< Links to a voxelyze object and initializes the solver. The pointer to the voxelyze object must remain valid for the lifetime of this object. @param[in] voxelyze pointer to the voxelyze object to simulate.
	bool solve(); //!< Formulates and solves the linear system and writes the resulting voxel positions and angles back to the linked voxelyze object. Returns false if the solver errors out. (check errorMsg for the reason). NOTE: calling this function modifies the state of the linked voxelyze object! This function may take a while if there are a large number of voxels.

	//parameters to get information during the solving process
	int progressTick; //!< An arbitrary progress number somewhere between zero and progressMaxTick to be used updating a progress bar.
	int progressMaxTick; //!< An arbitrary maximum for progress bars.
	std::string progressMsg; //!< A message indicating the current status of the solver
	std::string errorMsg; //!< If an error occurs the reason will be found here.
	bool cancelFlag; //!< A user-settable flag to indicate to the solver that an abort has been request during a solve process. May still not stop immediately, though.

private: //off limits variable and functions (internal)
	CVoxelyze* vx;
	int dof; //degrees of freedom in the problem
	std::vector<double> a, b, x;
	std::vector<int> ia, ja; //row index (1 based!), columns each value is in (1-based!)

	//Pardiso variables:
	int mtype; //defines matrix type
	int nrhs; //number of right-hand side vectors
	void *pt[64];
	int iparm[64];
	double dparm[64];
	int maxfct, mnum, phase, error, msglvl;

	//functions
	void calculateA(); //calculates the a (stiffness) matrix!
	void addAValue(int row, int column, float value);
	void consolidateA(); //gets rid of all the zeros for solving!
	void applyBX(); //apply forces and fixe boundary conditions
	void convertTo1Base(); //convert to 1-based indices for pardiso:
	void postResults(); //overwrites state of voxelyze object with the results
	void OutputMatrices(); //for debugging small system only!!

	void updateProgress(float percent, std::string message) {progressTick=(int)(percent*100), progressMsg = message;} //percent 0-1.0
};

//http://www.eng.fsu.edu/~chandra/courses/eml4536/Chapter4.ppt

//types:
// BEAM X						BEAM Y				BEAM Z				ALL 3
// Fx = | # * * * * * | x		| # * * * * # |		| # * * * # * |		| # * * * # # |
// Fy = | * # * * * # | y		| * # * * * * |		| * # * # * * |		| * # * # * # |
// Fz = | * * # * # * | z		| * * # # * * |		| * * # * * * |		| * * # # # * |
// Tx = | * * * # * * | rx		| * * # # * * |		| * # * # * * |		| * # # # * * |
// Ty = | * * # * # * | ry		| * * * * # * |		| # * * * # * |		| # * # * # * |
// Tz = | * # * * * # | rz		| # * * * * # |		| * * * * * # |		| # # * * * # |
//

//X
// Fx1 = |	a1	*	*	*	*	*	|	-a1	*	*	*	*	*	| x1
// Fy1 = |		b1	*	*	*	b2	|	*	-b1	*	*	*	b2	| y1
// Fz1 = |			b1	*	-b2	*	|	*	*	-b1	*	-b2	*	| z1
// Tx1 = |				a2	*	*	|	*	*	*	-a2	*	*	| rx1
// Ty1 = |					2b3	*	|	*	*	b2	*	b3	*	| ry1
// Tz1 = |						2b3	|	*	-b2	*	*	*	b3	| rz1
//		 |------------------------------------------------------|
// Fx2 = |							|	a1	*	*	*	*	*	| x2
// Fy2 = |							|		b1	*	*	*	-b2	| y2
// Fz2 = |							|			b1	*	b2	*	| z2
// Tx2 = |							|				a2	*	*	| rx2
// Ty2 = |							|					2b3	*	| ry2
// Tz2 = |							|						2b3	| rz2
//
//Y
//
// Fx1 = |	b1	*	*	*	*	-b2	|	-b1	*	*	*	*	-b2	| x1
// Fy1 = |		a1	*	*	*	*	|	*	-a1	*	*	*	*	| y1
// Fz1 = |			b1	b2	*	*	|	*	*	-b1	b2	*	*	| z1
// Tx1 = |				2b3	*	*	|	*	*	-b2	b3	*	*	| rx1
// Ty1 = |					a2	*	|	*	*	*	*	-a2	*	| ry1
// Tz1 = |						2b3	|	b2	*	*	*	*	b3	| rz1
//		 |------------------------------------------------------|
// Fx2 = |							|	b1	*	*	*	*	b2	| x2
// Fy2 = |							|		a1	*	*	*	*	| y2
// Fz2 = |							|			b1	-b2	*	*	| z2
// Tx2 = |							|				2b3	*	*	| rx2
// Ty2 = |							|					a2	*	| ry2
// Tz2 = |							|						2b3	| rz2
//
//Z
//
// Fx1 = |	b1	*	*	*	b2	*	|	-b1	*	*	*	b2	*	| x1
// Fy1 = |		b1	*	-b2	*	*	|	*	-b1	*	-b2	*	*	| y1
// Fz1 = |			a1	*	*	*	|	*	*	-a1	*	*	*	| z1
// Tx1 = |				2b3	*	*	|	*	b2	*	b3	*	*	| rx1
// Ty1 = |					2b3	*	|	-b2	*	*	*	b3	*	| ry1
// Tz1 = |						a2	|	*	*	*	*	*	-a2	| rz1
//		 |------------------------------------------------------|
// Fx2 = |							|	b1	*	*	*	-b2	*	| x2
// Fy2 = |							|		b1	*	b2	*	*	| y2
// Fz2 = |							|			a1	*	*	*	| z2
// Tx2 = |							|				2b3	*	*	| rx2
// Ty2 = |							|					2b3	*	| ry2
// Tz2 = |							|						a2	| rz2
#endif //CVX_LINEARSOLVER_H

