/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/

#ifndef VX_EXTERNAL_H
#define VX_EXTERNAL_H

#include "Vec3D.h"
#include "Quat3D.h"

typedef unsigned char dofObject;  //Bits: 0 0 Tz, Ty, Tz, Z, Y, X. 0 if free, 1 if fixed
enum dofComponent {
	X_TRANSLATE=1<<0,
	Y_TRANSLATE=1<<1,
	Z_TRANSLATE=1<<2,
	X_ROTATE=1<<3,
	Y_ROTATE=1<<4,
	Z_ROTATE=1<<5
};
inline void dofSet(dofObject& obj, dofComponent dof, bool set) {set ? obj|=dof : obj&=~dof;}
inline void dofSetAll(dofObject& obj, bool set) {set ? obj|=0x3F : obj&=~0x3F;}
inline bool dofIsSet(dofObject obj, dofComponent dof){return (dof&obj)?true:false;}
inline bool dofIsAllSet(dofObject obj){return (obj&0x3F)==0x3F;}
inline bool dofIsNoneSet(dofObject obj){return !(obj&0x3F);}
inline dofObject dof(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz) {dofObject ret=0; dofSet(ret, X_TRANSLATE, tx); dofSet(ret, Y_TRANSLATE, ty); dofSet(ret, Z_TRANSLATE, tz); dofSet(ret, X_ROTATE, rx); dofSet(ret, Y_ROTATE, ry); dofSet(ret, Z_ROTATE, rz); return ret;}


//! Container for all external influences on a voxel such as forces and prescribed displacements.
/*!
dof = degree(s) of freedom (three translational, three rotational = 6 total)
*/
class CVX_External
{
public:
	CVX_External();
	~CVX_External(); //!<destructor
	CVX_External(const CVX_External& eIn) {*this = eIn;} //!< Copy constructor
	CVX_External& operator=(const CVX_External& eIn); //!< Equals operator
	inline bool operator==(const CVX_External& b) {return dofFixed==b.dofFixed && extForce==b.extForce && extMoment==b.extMoment && extTranslation==b.extTranslation && extRotation==b.extRotation;} //!< comparison operator

	void reset(); //!< Resets this external to defaults - i.e., no effect on a voxel  (forces, fixed, displacements, etc) 
	bool isEmpty() {return (dofFixed == 0 && extForce==Vec3D<float>() && extMoment==Vec3D<float>());} //!< returns true if this external is empty - i.e is exerting no effect on a voxel

	bool isFixed(dofComponent dof) const {return dofIsSet(dofFixed, dof);}  //!< Returns true if the specified degree of freedom is fixed for this voxel. @param[in] dof Degree of freedom to query according to the dofComponent enum.
	bool isFixedAll() const {return dofIsAllSet(dofFixed);} //!< Returns true if all 6 degrees of freedom are fixed for this voxel.
	bool isFixedAllTranslation() const {return dofIsSet(dofFixed, X_TRANSLATE) && dofIsSet(dofFixed, Y_TRANSLATE) && dofIsSet(dofFixed, Z_TRANSLATE);} //!< Returns true if all translational degrees of freedom are fixed.
	bool isFixedAllRotation() const {return dofIsSet(dofFixed, X_ROTATE) && dofIsSet(dofFixed, Y_ROTATE) && dofIsSet(dofFixed, Z_ROTATE);} //!< Returns true if all rotationsl degrees of freedom are fixed.
	
	bool isFixedAny() const {return (dofFixed != 0);} //!< Returns true if any of the 6 degrees of freedom are fixed for this voxel.
	bool isFixedAnyTranslation() const {return dofIsSet(dofFixed, X_TRANSLATE) || dofIsSet(dofFixed, Y_TRANSLATE) || dofIsSet(dofFixed, Z_TRANSLATE);} //!< Returns true if any of the three translational degrees of freedom are fixed.
	bool isFixedAnyRotation() const {return dofIsSet(dofFixed, X_ROTATE) || dofIsSet(dofFixed, Y_ROTATE) || dofIsSet(dofFixed, Z_ROTATE);} //!< Returns true if any of the three rotational degrees of freedom are fixed.

	Vec3D<double> translation() const {return extTranslation;} //!< Returns any external translation that has been applied to this external.
	Vec3D<double> rotation() const {return extRotation;} //!< Returns any external rotation that has been applied to this external as a rotation vector.
	Quat3D<double> rotationQuat() const {return _extRotationQ ? *_extRotationQ : Quat3D<double>();} //!< Returns any external rotation that has been applied to this external as a quaternion.


	void setFixed(bool xTranslate, bool yTranslate, bool zTranslate, bool xRotate, bool yRotate, bool zRotate); //!< Sets any of the degrees of freedom specified as "true" to fixed for this voxel. (GCS) @param[in] xTranslate Translation in the X direction  @param[in] yTranslate Translation in the Y direction @param[in] zTranslate Translation in the Z direction @param[in] xRotate Rotation about the X axis @param[in] yRotate Rotation about the Y axis @param[in] zRotate Rotation about the Z axis
	void setFixed(dofComponent dof, bool fixed=true) {fixed?setDisplacement(dof):clearDisplacement(dof);} //!< Sets the specified degree of freedom to either fixed or free, depending on the value of fixed. Either way, sets the translational or rotational displacement of this degree of freedom to zero. @param[in] dof the degree of freedom in question @param[in] fixed Whether this degree of freedom should be fixed (true) or free (false).
	void setFixedAll(bool fixed=true) {fixed?setDisplacementAll():clearDisplacementAll();} //!< Sets all 6 degrees of freedom to either fixed or free depending on the value of fixed. Either way, sets all displacements to zero. @param[in] fixed Whether all degrees of freedom should be fixed (true) or free (false).

	void setDisplacement(dofComponent dof, double displacement=0.0); //!< Fixes the specified degree of freedom and applies the prescribed displacement if specified. @param[in] dof the degree of freedom in question. @param[in] displacement The displacement in meters (translational dofs) or radians (rotational dofs) to apply. Large fixed displacements may cause instability.
	void setDisplacementAll(const Vec3D<double>& translation = Vec3D<double>(0,0,0), const Vec3D<double>& rotation = Vec3D<double>(0,0,0)); //!< Fixes the all degrees of freedom and applies the specified translation and rotation. @param[in] translation The translation in meters from this voxel's nominal position to fix it at. @param[in] rotation The rotation (in the form of a rotation vector) from this voxel's nominal orientation to fix it at.

	void clearDisplacement(dofComponent dof); //!< Clears any prescribed displacement from this degree of freedom and unfixes it, too. @param[in] dof the degree of freedom in question.
	void clearDisplacementAll(); //!< Clears all prescribed displacement from this voxel and completely unfixes it, too.

	Vec3D<float> force() const {return extForce;} //!< Returns the current applied external force in newtons.
	Vec3D<float> moment() const {return extMoment;} //!< Returns the current applied external moment in N-m.

	void setForce(const float xForce, const float yForce, const float zForce) {extForce = Vec3D<float>(xForce, yForce, zForce);} //!< Applies forces to this voxel in the global coordinate system. Has no effect in any fixed degrees of freedom. @param xForce Force in the X direction in newtons.  @param yForce Force in the Y direction in newtons.  @param zForce Force in the Z direction in newtons. 
	void setForce(const Vec3D<float>& force) {extForce = force;} //!< Convenience function for setExternalForce(float, float, float).
	void setMoment(const float xMoment, const float yMoment, const float zMoment) {extMoment = Vec3D<float>(xMoment, yMoment, zMoment);}  //!< Applies moments to this voxel in the global coordinate system. All rotations according to the right-hand rule. Has no effect in any fixed degrees of freedom. @param xMoment Moment in the X axis rotation in newton-meters. @param yMoment Moment in the Y axis rotation in newton-meters. @param zMoment Moment in the Z axis rotation in newton-meters. 
	void setMoment(const Vec3D<float>& moment) {extMoment = moment;} //!< Convenience function for setExternalMoment(float, float, float).

	void addForce(const float xForce, const float yForce, const float zForce) {extForce += Vec3D<float>(xForce, yForce, zForce);} //!< Applies forces to this voxel in the global coordinate system. Has no effect in any fixed degrees of freedom. @param xForce Force in the X direction in newtons.  @param yForce Force in the Y direction in newtons.  @param zForce Force in the Z direction in newtons. 
	void addForce(const Vec3D<float>& force) {extForce += force;} //!< Convenience function for setExternalForce(float, float, float).
	void addMoment(const float xMoment, const float yMoment, const float zMoment) {extMoment += Vec3D<float>(xMoment, yMoment, zMoment);}  //!< Applies moments to this voxel in the global coordinate system. All rotations according to the right-hand rule. Has no effect in any fixed degrees of freedom. @param xMoment Moment in the X axis rotation in newton-meters. @param yMoment Moment in the Y axis rotation in newton-meters. @param zMoment Moment in the Z axis rotation in newton-meters. 
	void addMoment(const Vec3D<float>& moment) {extMoment += moment;} //!< Convenience function for setExternalMoment(float, float, float).

	void clearForce(){extForce = Vec3D<float>();} //!< Clears all applied forces from this voxel.
	void clearMoment(){extMoment = Vec3D<float>();} //!< Clears all applied moments from this voxel.
	
	
private:
	dofObject dofFixed;
	
	Vec3D<float> extForce, extMoment; //External force, moment applied to these voxels (N, N-m) if relevant DOF are unfixed
	Vec3D<double> extTranslation, extRotation;
	Quat3D<double>* _extRotationQ; //cached quaternion rotation (pointer to only create if needed)

	void rotationChanged(); //called to keep cached quaternion rotation in sync
};


#endif //VX_EXTERNAL_H