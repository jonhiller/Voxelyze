/*******************************************************************************
Copyright (c) 2010, Jonathan Hiller (Cornell University)
If used in publication cite "J. Hiller and H. Lipson "Dynamic Simulation of Soft Heterogeneous Objects" In press. (2011)"

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


//!
/*!
*/
class CVX_External
{
public:
	CVX_External();
	~CVX_External(); //!<destructor
	CVX_External(const CVX_External& eIn) {*this = eIn;} //!< Copy constructor
	CVX_External& operator=(const CVX_External& eIn); //!< Equals operator
	inline bool operator==(const CVX_External& b) {return dofFixed==b.dofFixed && extForce==b.extForce && extMoment==b.extMoment && extTranslation==b.extTranslation && extRotation==b.extRotation;} //!< comparison operator

	void reset(); //resets to default: no effect (forces, fixed, displacements, etc) on a voxel
	bool isEmpty() {return (dofFixed == 0 && extForce==Vec3D<float>() && extMoment==Vec3D<float>());} //returns true if there is no effect on a voxel

	bool isFixed(dofComponent dof) const {return dofIsSet(dofFixed, dof);}  //!< Returns true if the specified degree of freedom is fixed for this voxel. @param[in] dof Degree of freedom to query according to the dofComponent enum.
	bool isFixedAll() const {return dofIsAllSet(dofFixed);} //!< Returns true if all 6 degrees of freedom are fixed for this voxel.
	bool isFixedAllTranslation() const {return dofIsSet(dofFixed, X_TRANSLATE) && dofIsSet(dofFixed, Y_TRANSLATE) && dofIsSet(dofFixed, Z_TRANSLATE);}
	bool isFixedAllRotation() const {return dofIsSet(dofFixed, X_ROTATE) && dofIsSet(dofFixed, Y_ROTATE) && dofIsSet(dofFixed, Z_ROTATE);}
	
	bool isFixedAny() const {return (dofFixed != 0);} //!< Returns true if any of the 6 degrees of freedom are fixed for this voxel.
	bool isFixedAnyTranslation() const {return dofIsSet(dofFixed, X_TRANSLATE) || dofIsSet(dofFixed, Y_TRANSLATE) || dofIsSet(dofFixed, Z_TRANSLATE);}
	bool isFixedAnyRotation() const {return dofIsSet(dofFixed, X_ROTATE) || dofIsSet(dofFixed, Y_ROTATE) || dofIsSet(dofFixed, Z_ROTATE);}

	Vec3D<double> translation() const {return extTranslation;} 
	Vec3D<double> rotation() const {return extRotation;}
	Quat3D<double> rotationQuat() const {return _extRotationQ ? *_extRotationQ : Quat3D<double>();}


	void setFixed(bool xTranslate, bool yTranslate, bool zTranslate, bool xRotate, bool yRotate, bool zRotate); //!< Set the specified true degrees of freedom as fixed for this voxel. (GCS) @param[in] xTranslate Translation in the X direction  @param[in] yTranslate Translation in the Y direction @param[in] zTranslate Translation in the Z direction @param[in] xRotate Rotation about the X axis @param[in] yRotate Rotation about the Y axis @param[in] zRotate Rotation about the Z axis
	void setFixed(dofComponent dof, bool fixed=true) {fixed?addDisplacement(dof):clearDisplacement(dof);} 
	void setFixedAll(bool fixed=true) {fixed?addDisplacementAll():clearDisplacementAll();}  

//	void setFixed(dofComponent dof, double displacement=0.0f); //!< Sets the specified degree of freedom to fixed and applies the prescribed displacement. @param[in] dof Degree of freedom to fix according to the dofComponent enum. @param[in] displacement The prescribed displacement in meters for translational degrees of freedom and radians for rotational degrees of freedom.
//	void setFixedAll(const Vec3D<double>& translation = Vec3D<double>(0,0,0), const Vec3D<double>& rotation = Vec3D<double>(0,0,0)); //!<Fixes all 6 degrees of freedom for this voxel. @param [in] translation Translation in meters to prescribe (GCS). @param[in] rotation Rotation in radians to prescribe. Applied in the order of X, Y, Z rotation in the global coordinate system.
//	void setUnfixed(dofComponent dof) {dofSet(dofFixed, dof, false);} //!< Sets the specified degree of freedom to unfixed @param[in] dof Degree of freedom to fix according to the dofComponent enum.
//	void setUnfixedAll() {dofSetAll(dofFixed, false);} //!< Unfixes all 6 degrees of freedom for this voxel.

	void addDisplacement(dofComponent dof, double displacement=0.0); //fixes, too
	void addDisplacementAll(const Vec3D<double>& translation = Vec3D<double>(0,0,0), const Vec3D<double>& rotation = Vec3D<double>(0,0,0)); //fixes, too
	void clearDisplacement(dofComponent dof); //unfixes, too
	void clearDisplacementAll(); //unfixes, too


	Vec3D<float> force() const {return extForce;} //!< Returns the current applied external force in newtons.
	Vec3D<float> moment() const {return extMoment;} //!< Returns the current applied external moment in N-m.

	void setForce(const float xForce, const float yForce, const float zForce) {extForce = Vec3D<float>(xForce, yForce, zForce);} //!< Applies forces to this voxel in the global coordinate system. Has no effect in any fixed degrees of freedom. @param xForce Force in the X direction in newtons.  @param yForce Force in the Y direction in newtons.  @param zForce Force in the Z direction in newtons. 
	void setForce(const Vec3D<float>& force) {extForce = force;} //!< Convenience function for setExternalForce(float, float, float).
	void setMoment(const float xMoment, const float yMoment, const float zMoment) {extMoment = Vec3D<float>(xMoment, yMoment, zMoment);}  //!< Applies moments to this voxel in the global coordinate system. All rotations according to the right-hand rule. Has no effect in any fixed degrees of freedom. @param xMoment Moment in the X axis rotation in newton-meters. @param yMoment Moment in the Y axis rotation in newton-meters. @param zMoment Moment in the Z axis rotation in newton-meters. 
	void setMoment(const Vec3D<float>& moment) {extMoment = moment;} //!< Convenience function for setExternalMoment(float, float, float).

	void clearForce(){extForce = Vec3D<float>();}
	void clearMoment(){extMoment = Vec3D<float>();}
	
	void addForce(const float xForce, const float yForce, const float zForce) {extForce += Vec3D<float>(xForce, yForce, zForce);} //!< Applies forces to this voxel in the global coordinate system. Has no effect in any fixed degrees of freedom. @param xForce Force in the X direction in newtons.  @param yForce Force in the Y direction in newtons.  @param zForce Force in the Z direction in newtons. 
	void addForce(const Vec3D<float>& force) {extForce += force;} //!< Convenience function for setExternalForce(float, float, float).
	void addMoment(const float xMoment, const float yMoment, const float zMoment) {extMoment += Vec3D<float>(xMoment, yMoment, zMoment);}  //!< Applies moments to this voxel in the global coordinate system. All rotations according to the right-hand rule. Has no effect in any fixed degrees of freedom. @param xMoment Moment in the X axis rotation in newton-meters. @param yMoment Moment in the Y axis rotation in newton-meters. @param zMoment Moment in the Z axis rotation in newton-meters. 
	void addMoment(const Vec3D<float>& moment) {extMoment += moment;} //!< Convenience function for setExternalMoment(float, float, float).
	
	//	void setForceMoment(const Vec3D<float>& force = Vec3D<float>(0,0,0), const Vec3D<float>& moment = Vec3D<float>(0,0,0)) {extForce=force; extMoment=moment;} //!<
//	void setForceMoment(dofComponent dof, double displacement=0.0f) {extForce=force; extMoment=moment;} //!<

	
private:
	dofObject dofFixed;
	
	Vec3D<float> extForce, extMoment; //External force, moment applied to these voxels (N, N-m) if relevant DOF are unfixed
	Vec3D<double> extTranslation, extRotation;
	Quat3D<double>* _extRotationQ; //cached quaternion rotation (pointer to only create if needed)

	void rotationChanged(); //called to keep cached quaqternion rotation in sync
};


#endif //VX_EXTERNAL_H