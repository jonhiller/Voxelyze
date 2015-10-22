/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_LinearSolver.h"
#include "Voxelyze.h"
#include "VX_MaterialLink.h"
#include <iostream>

//VERSION 5
//#define USE_DIRECT
#define UNUSED FLT_MAX

//some static info to reference in the algorithms
static int blockOff[6][3] = {{0,4,5},{1,3,5},{2,3,4},{1,2,3},{0,2,4},{0,1,5}};
static dofComponent dofMap[6] = {X_TRANSLATE, Y_TRANSLATE, Z_TRANSLATE, X_ROTATE, Y_ROTATE, Z_ROTATE};

CVX_LinearSolver::CVX_LinearSolver(CVoxelyze* voxelyze)
{
	vx = voxelyze;
	iteration = 0;

	//Pardiso Params!
	mtype = 2; // Real symmetric matrix //was 2
	nrhs = 1; // Number of right hand sides.
	maxfct = 1; //Maximum number of numerical factorizations.
	mnum = 1; //Which factorization to use.
	msglvl = 0; //Print statistical information
	error = 0; //Initialize error flag
#ifdef USE_DIRECT
	int solver = 0; //0 = use default (non-iterative) Pardiso solver, 1=iterative
#else
	int solver = 1; //0 = use default (non-iterative) Pardiso solver, 1=iterative
#endif


	progressTick = 0;
	progressMaxTick = 100; //never changes
	progressMsg = "";
	errorMsg = "";
	cancelFlag = false;

#ifdef PARDISO_5
	pardisoinit(pt, &mtype, &solver, iparm, dparm, &error); //initialize pardiso
#endif
}


bool CVX_LinearSolver::solve(bool structureUnchanged) //formulates and solves system!
{
	if (!structureUnchanged) 
		iteration = 0; //this acts as a flag to recalculate the structure of the A matrix

	updateProgress(0, "Forming matrices...");
	cancelFlag = false; //this may be set to true, in which case we should interrupt the solve process...
	bool Success = true; //flag to track whether a part of the process fails.

	//deal with disconnected voxels of lack of fixed voxels here?

	//set DOF
	dof = vx->voxelCount()*6;
	if (dof == 0) return false;

	calculateA();

	applyBX();
	convertTo1Base();
	//OutputMatrices(); //uncomment to output info for small systems

	if (dof == 0){ errorMsg = "No free degrees of freedom found. Aborting.\n"; return false;}

		
#ifdef USE_DIRECT
	iparm[0] = 0;
#else
	for (int i=0; i<64; i++) iparm[i] = 0;
	iparm[0] = 1; //1 = don't use any defaults
	iparm[1] = 2; //2 = use METIS
	iparm[2] = -1; //-1 = use environment variable for number of threads? <---
	iparm[3] = 0; // preconditioned CGS? (at least the first time)
	iparm[4] = 0; // 0 = don't user user permutation
	iparm[5] = 0; // 0 = write solution to x
	iparm[7] = 0; // max number of iterative refinement steps <--------------
	iparm[9] = 0; // 8 = default for sym. ind. matrics. (eps pivot)
	iparm[10] = 0; // don't use non-symmmetric scaling
	iparm[11] = 0; // don't transpose
	iparm[12] = 0; // don't use non-symettric matchings
	iparm[17] = 0; //flag to calculate non-zeros in LU (-1 to calculate, 0 to not)
	iparm[18] = 0; //flag to calculate gflops (-1 to calculate, 0 to not) <-----------
	iparm[20] = 1; // 1= better pivoting for symettric
	iparm[23] = 0; // 1=2 level parallel
	iparm[24] = 1; // do parallel solve
	iparm[25] = 0; // do forward/backward solve with L/U
	iparm[27] = 0; // 0 = sequential metis ordering (to parallel (=1)?) <----------
	iparm[28] = 0; // 0 = 64 bit, 1 = 32 bit <------------------------
	iparm[29] = 0; // default supernodes
	iparm[30] = 0; // compute everything
	iparm[31] = 0; // 0 = direct, 1 = iterative solver (iterative solver seems to suppress output and all output calc's) <------------------------
	iparm[32] = 0; // 0 = no determinate 
	iparm[33] = 0; // 0 = no identical results
	iparm[35] = 0; // 0 = overwrite internal factor
	iparm[36] = 0; // 0 = return inversion as upper triangular
	iparm[37] = 0; // 0 = default
	iparm[50] = 0; // 0 = openmp, 1=open mp/mpi <-------------------
	iparm[51] = 0; // 1 = for openmp - numer of copute nodes
#endif


	//if iterative
//	iparm[0] = 0;
//	iparm[2] = 4;
//	iparm[2] = -1;

//	iparm[3] = 12;

//	iparm[31] = 1; //Use the multi-recursive iterative linear solver

//#endif
	//dparm[0] = 5;


	int idum = 0; //Integer dummy var

#ifdef PARDISO_5
	updateProgress(0.02f, "Pardiso: Analyzing...");
	phase = 13;
#ifndef USE_DIRECT
	if (iteration != 0){
	//	std::cout << "only phase 2/3 \n";
		phase = 23;
		iparm[3] = 12;
	}
#endif
	//	std::cout << msglvl << "\n";
	//for (int i=0; i<64; i++) std::cout << i << " " << iparm[i] << "\n";


	pardiso(pt, &maxfct, &mnum, &mtype, &phase, &dof, &a[0], &ia[0], &ja[0], &idum, &nrhs, iparm, &msglvl, &b[0], &x[0], &error, dparm);

//	updateProgress(0.05f, "Pardiso: Numerical factorization...");
//	phase = 22;
//	pardiso(pt, &maxfct, &mnum, &mtype, &phase, &dof, &a[0], &ia[0], &ja[0], &idum, &nrhs, iparm, &msglvl, &b[0], &x[0], &error, dparm);

//	updateProgress(0.79f, "Pardiso: Solving, iterative refinement...");
//	phase = 33;
//	pardiso(pt, &maxfct, &mnum, &mtype, &phase, &dof, &a[0], &ia[0], &ja[0], &idum, &nrhs, iparm, &msglvl, &b[0], &x[0], &error, dparm);

	if (error != 0){
		std::cout << "error: " << error << "\n";

		Success=false;
		switch (error){
		case -1: errorMsg = "Pardiso error: Input inconsistent\n";
		case -2: errorMsg = "Pardiso error: Not enough memory\n";
		case -3: errorMsg = "Pardiso error: Reordering Problem\n";
		case -4: errorMsg = "Pardiso error: Zero pivot, numerical factorization or iterative refinement problem\n";
		case -10: errorMsg = "No License file Pardiso.lic found.\nGo to www.pardiso-project.org and follow the instructions to obtain a license file for your computer.\nPlace the file in the executable directory.\n";
		case -11: errorMsg = "License is expired\n";
		case -12: errorMsg = "Wrong username or hostname\n";
		default: errorMsg = "Pardiso Error\n";
		}
	}

//	updateProgress(0.9f, "Pardiso: Cleaning up...");
//	phase = -1; /* Release internal memory. */
//	pardiso(pt, &maxfct, &mnum, &mtype, &phase, &dof, &a[0], &ia[0], &ja[0], &idum, &nrhs, iparm, &msglvl, &b[0], &x[0], &error, dparm);
#else
	Success = false;
#endif



//	std::cout << "number of it refinements: " << iparm[6] << "\n";
//	std::cout << "number of iterations: " << iparm[19] << "\n"; //C/G iterations

	//std::cout << "number of iterations: " << iparm[6] << "\n";
//	std::cout << "factorization Gflops: " << iparm[18] << "\n";
//	std::cout << "CG/CGS diagnostics: " << iparm[19] << "\n";

	if (!Success) return false;



	updateProgress(0.9f, "Processing results...");
	postResults();
	iteration++;
	return true;
}


void CVX_LinearSolver::calculateA() //calculates the big stiffness matrix!
{
	int vCount = vx->voxelCount(), lCount = vx->linkCount();
	int nA = 12*vCount+18*lCount; //approximate number of non-zero elements in A (overestimates by quite a bit!)

	if (iteration == 0){ //stuff that only needs to be done when the structure has changed (besides just stiffnesses. Voxel pointers must remain valid)
		//build temporary reverse lookup from voxel* to index
	//	std::unordered_map<CVX_Voxel*, int> v2i;

		v2i.clear();
		for (int i=0; i<vCount; i++) {v2i[vx->voxel(i)] = i;}

		ia.resize(dof+1);
		std::fill(ia.begin(), ia.end(), 0.0); //start with nothing
		ja.resize(nA);
		std::fill(ja.begin(), ja.end(), 0.0); //start with nothing
		a.resize(nA); //optimized set everything to 0
		std::fill(a.begin(), a.end(), UNUSED); //start with empty


			//std::vector<int> i2List;
		int i2List[6];

		int iACounter = 1, jACounter = 0; //first ia always 0

		for (int i=0; i<vCount; i++){ //for each voxel...
			CVX_Voxel* iv = vx->voxel(i);

			//populate i2
			int i2ListSize = 0;
			for (int j=0; j<6; j++){ //for each possible link
				if (iv->link((CVX_Voxel::linkDirection)j)){
					int i2 = v2i[iv->adjacentVoxel((CVX_Voxel::linkDirection)j)];
					if (i2>i) i2List[i2ListSize++] = i2; // i2List.push_back(i2);
				}
			}
			std::sort(i2List,i2List+i2ListSize);

			for (int j=0; j<6; j++){ //for each degree of freedom of the current voxel
				int diagAIndex = jACounter; //index in a and ja of the diagonal element

				//diagonal block
				ja[jACounter++] = 6*i+j; //diagonal element
				if (j<3){ //2 off-diagonals for first 3 dof, none for the rest
					ja[jACounter++] = 6*i+blockOff[j][1];
					ja[jACounter++] = 6*i+blockOff[j][2];
				}
		
				//off-diagonal block(s)
				//int nI2 = (int)(i2List.size()); //number of other voxels connected to this one
				for (int m=0; m<i2ListSize; m++){ //was nI2;
					for (int k=0; k<3; k++)	ja[jACounter++] = 6*i2List[m]+blockOff[j][k];
				}

				ia[iACounter] = ia[iACounter-1] + (jACounter-diagAIndex);
				iACounter++;
			}
		}
	}
	else {
		convertFrom1Base(); //put ia and ja back to 0-based
		std::fill(a.begin(), a.end(), 0); //more correct guesses on the if statement in addAValue?
	}


	//add in elements to a matrix
	for (int i=0; i<lCount; i++){
		CVX_Link* pL = vx->link(i);
		int i1 = v2i[pL->voxel(true)];
		int i2 = v2i[pL->voxel(false)];
		if (i1>i2) {int tmp=i1; i1=i2; i2=tmp;} //swap to keep i1 lower than i2

		int ax = (int)pL->axis;

		for (int j=0; j<6; j++){ //for each DOF
			int row1 = i1*6+j;
			int row2 = i2*6+j;

			//diagonals:
			if (j<3){
				float diagValD = (ax==j) ? pL->a1() : pL->b1(); //diagonals on the diagonal block
				float diagValO = -diagValD; //diagonals on the off-diagonal block
				addAValue(row1, row1, diagValD);
				addAValue(row1, row2, diagValO);
				addAValue(row2, row2, diagValD);
			}
			else {
				float diagValD = (ax==j%3) ? pL->a2() : 2*pL->b3(); //diagonals on the diagonal block
				float diagValO = (ax==j%3) ? -pL->a2() : pL->b3(); //diagonals on the off-diagonal block
				addAValue(row1, row1, diagValD);
				addAValue(row1, row2, diagValO);
				addAValue(row2, row2, diagValD);
			}
		}

		//off-diagonals:
		int R1, C1, R2, C2;
		float val;
		switch (ax){
			case 0: //X_AXIS
				R1 = 1; C1 = 5;
				R2 = 2; C2 = 4;
				val = pL->b2();
				break;
			case 1: //Y_AXIS
				R1 = 0; C1 = 5;
				R2 = 2; C2 = 3;
				val = -pL->b2();
				break;
			default: //Z_AXIS
				R1 = 0; C1 = 4;
				R2 = 1; C2 = 3;
				val = pL->b2();
				break;
		}

		addAValue(i1*6+R1, i1*6+C1, val);
		addAValue(i1*6+R1, i2*6+C1, val);
		addAValue(i1*6+C1, i2*6+R1, -val);
		addAValue(i2*6+R1, i2*6+C1, -val);

		addAValue(i1*6+R2, i1*6+C2, -val);
		addAValue(i1*6+R2, i2*6+C2, -val);
		addAValue(i1*6+C2, i2*6+R2, val);
		addAValue(i2*6+R2, i2*6+C2, val);
	}

	if (iteration == 0) 
		consolidateA(); //remove all the zeros
}

void CVX_LinearSolver::addAValue(int row, int column, float value) //after ia and ja are populated, set a value in the a matrix. Assumes 0-based indices!
{
	int curInd = ia[row];
	int endInd = ia[row+1];
	while (ja[curInd] != column){
		if (++curInd == endInd){
			assert(false); //error - this a location doesn't exist in the sparse structure
			return;
		}
	}
	if (a[curInd] == UNUSED) a[curInd]=value;
	else a[curInd]+=value;
}


void CVX_LinearSolver::consolidateA() //gets rid of all the zeros for quicker solving! (maybe...). Assumes 1-based arrays!
{
	int index = 0; //master index of longer arrays
	int shift = 0;

	for (int i=0; i<dof; i++){ //for each element of ia;
		ia[i+1] -= shift;
		while (index < ia[i+1]){ //for everything on this row...
			while (a[index + shift] == UNUSED){ //if this is an element to remove
				shift++;
				ia[i+1]--;
			}
			a[index] = a[index+shift];
			ja[index] = ja[index+shift];

			index++;
		}
	}

	a.resize(index); //potential to free up some memory
	ja.resize(index);
}

void CVX_LinearSolver::applyBX() //Assumes 0-based indices
{
	int vCount = vx->voxelCount();
	
	if (iteration == 0){ //stuff that only needs to be done when the structure has changed (besides just stiffnesses.)
		x.resize(dof);
		b.resize(dof);
		std::fill(b.begin(), b.end(), 0.0); //start with no forces applied
		//std::vector<int> aToZero; //list of "a" matrix indices to set to zero at the end.
		aToZero.clear();
		fixed.resize(dof); //any fixed degrees of freedom
		int fixedCount = 0;

		//fill in all displacements first: (and build list to note which are prescribed/fixed)
		for (int i=0; i<vCount; i++){
			CVX_Voxel* pVox = vx->voxel(i);
			bool hasExternal = pVox->externalExists();
			Vec3D<double> position(pVox->displacement());
			Vec3D<double> angle = (pVox->orientation().w == 1) ? Vec3D<double>(0,0,0) : pVox->orientation().ToRotationVector();
			Vec3D<float> force(hasExternal ? pVox->external()->force() : Vec3D<float>());
			Vec3D<float> moment(hasExternal ? pVox->external()->moment() : Vec3D<float>());
		
			for (int j=0; j<6; j++){
				int thisDof = 6*i+j;
				x[thisDof] = (j<3)?position[j]:angle[j%3]; //add in displacement
				bool iAmFixed = hasExternal ? pVox->external()->isFixed(dofMap[j]) : false; //note if it is presecribed/fixed
				fixed[thisDof] = iAmFixed;

				if (iAmFixed) fixedCount++;
				else b[thisDof] = (j<3)?force[j]:moment[j%3]; //add in forces if it's not a fixed degree of freedom
			}
		}

		aToZero.reserve(12*fixedCount);

		//Add in forces that make prescribed displacements work and mark the "a" values that are to be removed
		const int iaSize = ia.size();
		for (int i=0; i<iaSize-1; i++){ //i is the row of this element
			const int kStart = ia[i], kEnd=ia[i+1];
			for(int k=kStart; k<kEnd; k++){
				const int j = ja[k]; //j is the column of this element
				
				if (fixed[i] || fixed[j]){ //same row as fixed degree of freedom
					b[i] -= x[j]*a[k]; //subract out any force for the linked DOF for this element
					if (i != j) aToZero.push_back(k); //if not on diagonal, nuke it!
				}
			}
		}


		//for (int thisDof=0; thisDof<dof; thisDof++){
		//	if (fixed[thisDof]){
		//		int dofCounter = 0; //keep track of the row we're searching when examining the column above the active diagonal element, or the column when in the final row.
		//		
		//		//int RowBegin = 
		//		int lastAToConsider = ia[thisDof+1];
		//		for (int k=0; k<lastAToConsider; k++){ //up through the end of this row
		//			bool thisRow = (k>=ia[thisDof]); //searching the final row (otherwise searching the column above)
		//			if (thisRow) dofCounter = ja[k]; //keep track of the column as we step through the row.
		//			if (k==ia[dofCounter+1]) dofCounter++; //keep track of the row as we step through columns

		//			if (ja[k]==thisDof || thisRow){ //if in column or row of this degree of freedom...
		//				b[dofCounter] -= x[thisDof]*a[k];
		//				if (iteration == 0 && k!=ia[thisDof]) aToZero.push_back(k); //if not diagonal, it will be zeroed. However, defer zeroing since some of these values could be used twice...
		//			}

		//		}
		//	}
		//}


	}
	else { //already have the structure, but still need to re-calculate forces in case they changed...
		std::fill(b.begin(), b.end(), 0.0); //start with no forces applied
		for (int i=0; i<vCount*6; i++){ //through each degree of freedom
			CVX_Voxel* pVox = vx->voxel(i/6);
			if (pVox->externalExists() && !fixed[i]) b[i] = (i%6<3)? pVox->external()->force()[i%3]:pVox->external()->moment()[i%3]; //add in forces if it's not a fixed degree of freedom
		}
	}

	//for all fixed degrees of freedom, set the a value to something that will work out
	for (int thisDof=0; thisDof<dof; thisDof++){
		if (fixed[thisDof]){
			a[ia[thisDof]] = 1.0; //unit diagonal
			b[thisDof] = x[thisDof];
		}
	}

	const int numToZero = (int)aToZero.size();
	for (int i=0; i<numToZero; i++) a[aToZero[i]] = 0; //zero out the ones we've marked accordingly
}

void CVX_LinearSolver::convertTo1Base()
{
	for (int i=0; i<(int)ia.size(); i++) ia[i]++;
	for (int i=0; i<(int)ja.size(); i++) ja[i]++;
}

void CVX_LinearSolver::convertFrom1Base()
{
	for (int i=0; i<(int)ia.size(); i++) ia[i]--;
	for (int i=0; i<(int)ja.size(); i++) ja[i]--;
}

void CVX_LinearSolver::postResults() //overwrites state of voxelyze object with the results
{
	int vCount = vx->voxelCount();
	for (int i=0; i<vCount; i++){
		CVX_Voxel* pVox = vx->voxel(i);
		pVox->pos = pVox->originalPosition() + Vec3D<double>(x[6*i], x[6*i+1], x[6*i+2]);
		pVox->linMom = Vec3D<double>(0,0,0);
		pVox->orient = Quat3D<double>(Vec3D<double>(x[6*i+3], x[6*i+4], x[6*i+5]));
		pVox->angMom = Vec3D<double>(0,0,0);
	}

	int lCount = vx->linkCount();
	for (int i=0; i<lCount; i++){
		vx->link(i)->updateForces();
	}
}



#include <fstream>
void CVX_LinearSolver::OutputMatrices()
{ 
	std::ofstream file("A_Matrix.txt"); //filename);
	file << "A Matrix:\n";

	int jaCount = 0;
	for (int i=0; i<(int)ia.size()-1; i++){
		int colCount = 0;
		while (colCount < dof){
			if (jaCount+1 < ia[i+1] && colCount + 1 == ja[jaCount]){
				file << a[jaCount] << "\t";
				jaCount++;
			}
			else file << "\t";
			
			colCount++;
		}
		file << "\n";
	}

	file << "\n\nRaw Matrices:\na\tja\n";
	for (int j=0; j<ia[dof-1]; j++){
		file << a[j] << "\t";
		file << ja[j] << "\n";
	}
	file << "\ni\n";
	for (int j=0; j<dof+1; j++){
		file << ia[j] << "\t";
	}
	file << "\n\nx\tb\n";

	for (int j=0; j<dof; j++){
		file << x[j] << "\t";
		file << b[j] << "\n";
	}


	file.close();
	
}


