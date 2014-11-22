/*******************************************************************************
Copyright (c) 2010, Jonathan Hiller (Cornell University)
If used in publication cite "J. Hiller and H. Lipson "Dynamic Simulation of Soft Heterogeneous Objects" In press. (2011)"

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

class CVX_LinearSolver
{

public: //variables and flags to be interacted with
	CVX_LinearSolver(CVoxelyze* voxelyze);
	bool solve(); //formulates and solves system!

	//parameters to get information during the solving process
	int progressTick, progressMaxTick;
	std::string progressMsg, errorMsg;
	bool cancelFlag;

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

	void updateProgress(float percent, std::string message) {progressTick=percent*100, progressMsg = message;} //percent 0-1.0
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

