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
	reset();
}

CVX_External::~CVX_External()
{
	if (_extRotationQ) delete _extRotationQ;
}


CVX_External& CVX_External::operator=(const CVX_External& eIn)
{
	dofFixed = eIn.dofFixed;
	extForce = eIn.extForce;
	extMoment = eIn.extMoment;
	extTranslation = eIn.extTranslation;
	extRotation = eIn.extRotation;
	rotationChanged();
	return *this;
}

void CVX_External::reset()
{
	dofFixed=0;
	extForce = extMoment = Vec3D<float>();
	extTranslation = Vec3D<double>();
	extRotation = Vec3D<double>();
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




