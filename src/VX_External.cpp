/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#include "VX_External.h"

CVX_External::CVX_External() 
{
	_extRotationQ = 0;
	mfcs = 0;
	reset();
}

CVX_External::~CVX_External()
{
	if (_extRotationQ) delete _extRotationQ;
	if (mfcs) delete mfcs;

}


CVX_External& CVX_External::operator=(const CVX_External& eIn)
{
	dofFixed = eIn.dofFixed;
	extForce = eIn.extForce;
	extMoment = eIn.extMoment;
	extTranslation = eIn.extTranslation;
	extRotation = eIn.extRotation;
	if (eIn.mfcs) {
		if (!mfcs) mfcs = new std::vector<std::vector<double>>; //initialize to zero
		
		mfcs = eIn.mfcs;
		//for (int i = 0; i < eIn.mfcCount(); i++) {
		//	std::vectdouble* mfcsFrom = eIn.mfcElements(i);
		//	double thisMfcs[6];
		//	for (int j = 0; j < 6; j++) thisMfcs[j] = mfcsFrom[j];
		//	mfcs->push_back(thisMfcs);
		//}

	}
	else if (mfcs){
		delete[] mfcs;
		mfcs = 0;
	}
	rotationChanged();
	return *this;
}

bool CVX_External::operator==(const CVX_External& b)
{
	//a bit of work to determin equality of mfc
	bool mfcEqual = true;
	if (!mfcs && b.mfcs || mfcs && !b.mfcs) mfcEqual = false;
	if (mfcs && b.mfcs) {
		if (mfcCount() != b.mfcCount()) mfcEqual = false;
		else { //same count of mfcs
			for (int i = 0; i < mfcCount(); i++) {
				double* els = mfcElements(i);
				double* bels = b.mfcElements(i);

				for (int j = 0; j < 6; j++) {
					if (els[j] != bels[j]){
						mfcEqual = false;
						break;
					}
				}
			}
		}
	}

	return dofFixed == b.dofFixed && extForce == b.extForce && extMoment == b.extMoment && extTranslation == b.extTranslation && extRotation == b.extRotation && mfcEqual;
}

void CVX_External::reset()
{
	dofFixed=0;
	extForce = extMoment = Vec3D<float>();
	extTranslation = Vec3D<double>();
	extRotation = Vec3D<double>();
	if (mfcs) delete[] mfcs;
	mfcs = 0;
	rotationChanged();
}


void CVX_External::setFixed(bool xTranslate, bool yTranslate, bool zTranslate, bool xRotate, bool yRotate, bool zRotate)
{
	dofFixed = dof(xTranslate, yTranslate, zTranslate, xRotate, yRotate, zRotate);
	extTranslation = extRotation = Vec3D<double>(); //clear displacements
}

void CVX_External::setDisplacement(dofComponent dof, double displacement)
{
	dofSet(dofFixed, dof, true);
	if (displacement != 0.0f){
		if (dof & X_TRANSLATE) extTranslation.x = displacement;
		if (dof & Y_TRANSLATE) extTranslation.y = displacement;
		if (dof & Z_TRANSLATE) extTranslation.z = displacement;
		if (dof & X_ROTATE) extRotation.x = displacement;
		if (dof & Y_ROTATE)	extRotation.y = displacement;
		if (dof & Z_ROTATE) extRotation.z = displacement;
	}

	rotationChanged();
}

void CVX_External::setDisplacementAll(const Vec3D<double>& translation, const Vec3D<double>& rotation)
{
	dofSetAll(dofFixed, true);
	extTranslation = translation;
	extRotation = rotation;

	rotationChanged();
}

void CVX_External::clearDisplacement(dofComponent dof)
{
	dofSet(dofFixed, dof, false);

	if (dof & X_TRANSLATE) extTranslation.x = 0.0;
	if (dof & Y_TRANSLATE) extTranslation.y = 0.0;
	if (dof & Z_TRANSLATE) extTranslation.z = 0.0;
	if (dof & X_ROTATE) extRotation.x = 0.0;
	if (dof & Y_ROTATE)	extRotation.y = 0.0;
	if (dof & Z_ROTATE) extRotation.z = 0.0;

	rotationChanged();
}

void CVX_External::clearDisplacementAll()
{
	dofSetAll(dofFixed, false);
	extTranslation = Vec3D<double>();
	extRotation = Vec3D<double>();

	rotationChanged();
}

void CVX_External::addMfc(double cX, double cY, double cZ, double cRX, double cRY, double cRZ)
{
	if (!mfcs) mfcs = new std::vector<std::vector<double>>;

	std::vector<double> toAdd = { cX, cY, cZ, cRX, cRY, cRZ };
	mfcs->push_back(toAdd);
}

int CVX_External::mfcCount() const
{
	if (mfcs) return mfcs->size();
	else return 0;
}

void CVX_External::removeMfc(int mfcIndex)
{
	if (mfcCount() != 0 && mfcIndex < mfcs->size()) {
		mfcs->erase(mfcs->begin() + mfcIndex);
		
		if (mfcs->size() == 0) clearAllMfcs();
	}
}

void CVX_External::clearAllMfcs()
{
	if (mfcs) {
		delete[] mfcs;
		mfcs = 0;
	}
}

double* CVX_External::mfcElements(int mfcIndex) const
{
	if (mfcCount() != 0 && mfcIndex < mfcs->size()) {
		return &((mfcs->at(mfcIndex))[0]); //pointer to first element
	}
	else return 0;
}

void CVX_External::rotationChanged()
{
	if (extRotation != Vec3D<double>()){
		if (!_extRotationQ) _extRotationQ = new Quat3D<double>;
		*_extRotationQ = Quat3D<double>(extRotation);
	}
	else { //rotation is zero in all axes
		if (_extRotationQ) *_extRotationQ = Quat3D<double>();
	}
}




