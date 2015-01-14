/*******************************************************************************
Copyright (c) 2015, Jonathan Hiller
To cite academic use of Voxelyze: Jonathan Hiller and Hod Lipson "Dynamic Simulation of Soft Multimaterial 3D-Printed Objects" Soft Robotics. March 2014, 1(1): 88-101.
Available at http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/


#ifndef ARRAY3D_H
#define ARRAY3D_H

//#include "XML_Rip.h"
//#include <assert.h>
#include <vector>
#include <limits.h>
//#include <fstream>

//why can't min and max just be there when you need them?
#define LOCALMIN(a,b) (((a)<(b))?(a):(b))
#define LOCALMAX(a,b) (((a)>(b))?(a):(b))

#define INDEX_INVALID -2147483647 //guaranteed very negative number within range of int on any common platform

//! A generic three-integer index container (for X, Y, Z) for use with the CArray3D class.
/*!
Indices do not have a default and can be invalid. If any indices are invalid valid() will return false.
*/
struct Index3D {
	Index3D(){x=y=z=INDEX_INVALID;} //!< Default constructor. Indices will be invalid until set.
	Index3D(int inX, int inY, int inZ){x=inX; y=inY; z=inZ;} //!< Constructs with the specified indices
	Index3D(const Index3D& i3D) {x=i3D.x; y=i3D.y; z=i3D.z;} //!< Copy constructor
	Index3D& operator=(const Index3D& i3D) {x=i3D.x; y=i3D.y; z=i3D.z; return *this;} //!< Overload equals.
	const Index3D operator+(const Index3D &i3D) const {return Index3D(x+i3D.x, y+i3D.y, z+i3D.z);} //!< Adds an Index3D to this one. Indices are added individually.
	const Index3D operator-(const Index3D &i3D) const {return Index3D(x-i3D.x, y-i3D.y, z-i3D.z);} //!< Subtracts an Index3D from this one. Indices are subtracted individually.
	bool operator==(const Index3D& i3D) const {return (x==i3D.x && y==i3D.y && z==i3D.z);} //!< Two index3Ds are equal only if all indices are equal.
	bool operator!=(const Index3D& i3D) const {return (x!=i3D.x || y!=i3D.y || z!=i3D.z);} //!< Two index3Ds are not equal if any indices are not equal.
	bool valid(){return !(x==INDEX_INVALID || y==INDEX_INVALID || z==INDEX_INVALID);} //!< Returns true if all indices are valid.
	int x; //!< Current X index.
	int y; //!< Current Y index.
	int z; //!< Current Z index.
};

//!A generic 3D multi-size array template.
/*!
An array template with a user-definable default value that is returned for any 3D index not specified otherwise. Otherwise the functions will be generally familiar to anyone used to the standard library containers.

There are three dimensions of indices, referred to i, j, and k. In many uses these will be synonomous with x, y, and z. 

Memory allocation is managed internally and totally abstracted from the user. Whenever addvalue is called the internal array is smartly resized if the index falls outside of the currently allocated area. If the dimensions are known in advance, resize() may be called once to reduce costly memory reallocations.

The value of an element can be accesed with [], (), or at(). Empty elements (or elements outside the allocated area) are synonomous with the element being defaultValue.

*/
template <typename T = float>
class CArray3D
{
public:
	//!Constructor
	CArray3D(){
		defaultValue = 0;
		clear();
	}

	//!Destructor
	~CArray3D(void){
	}

	//!Copy constructor
	CArray3D(const CArray3D& rArray){
		*this = rArray;
	}

	//!Operator "=" overload
	CArray3D& operator=(const CArray3D& rArray){
		defaultValue = rArray.defaultValue;
		data = rArray.data;
		aSize = rArray.aSize;
		aOff = rArray.aOff;
		cMin = rArray.cMin;
		cMax = rArray.cMax;
		return *this;
	}

  	const T& operator [](const Index3D i3D) const   { return at(i3D);} //!< Operator "[]" overload (takes an Index3D) const version
		  T& operator [](const Index3D i3D)			{ return at(i3D);}  //!< Operator "[]" overload (takes an Index3D)

	const T& operator ()(int i, int j, int k) const {return at(Index3D(i, j, k));} //!< Operator "()" overloads (takes individual x, y, z indices) const verison
		  T& operator ()(int i, int j, int k)		{return at(Index3D(i, j, k));} //!< Operator "()" overloads (takes individual x, y, z indices)

	//!Clears all data from the array and frees up all memory.
	void clear(){ 
		aSize = aOff = Index3D(0,0,0);
		cMin = Index3D(INT_MAX, INT_MAX, INT_MAX);
		cMax = Index3D(INT_MIN, INT_MIN, INT_MIN);
		data.clear();
	}

	//!Sets the value to which all new allocations default to. @param[in] newDefaultValue the value returned from any index that has not been set otherwise.
	void setDefaultValue(T newDefaultValue){
		int linSize = data.size();
		for (int i=0; i<linSize; i++) if (data[i]==defaultValue) data[i] = newDefaultValue; //replace all old defaults with new default
		defaultValue = newDefaultValue; //remember new default
	}

	Index3D minIndices() const {return cMin;} //!< Returns the minimum i, j, and k indices utilized by any element in the array
	Index3D maxIndices() const {return cMax;} //!< Returns the maximum i, j, and k indices utilized by any element in the array

	//! Returns the value at the specified 3d index or the default value otherwise. Const version. @param[in] i3D the 3D index (i,j,k) in question.
	const T& at(const Index3D& i3D) const { 
		int i = getIndex(i3D);
		return i == -1 ? defaultValue : data[i];
	} 
	const T& at(int i, int j, int k) const {return at(Index3D(i,j,k));} //!< Returns the value at the specified 3d index or the default value otherwise. Const version. @param[in] i the i index in question. @param[in] j the j index in question. @param[in] k the k index in question.

	//! Returns the value at the specified 3d index or the default value otherwise.  @param[in] i3D the 3D index (i,j,k) in question.
	T& at(const Index3D& i3D) {
		int i = getIndex(i3D);
		return i == -1 ? defaultValue : data[i];
	} 
	T& at(int i, int j, int k) {return at(Index3D(i,j,k));} //!< Returns the value at the specified 3d index or the default value otherwise. @param[in] i the i index in question. @param[in] j the j index in question. @param[in] k the k index in question.


	//! Resize the internal data allocation to new specified size and offset. Any data ouside the new range is discarded. The range of allocated values in a given dimension spans from newOffset to newOffset+newsize. @param[in] newSize the number of elements in i, j, and k to allocate @param[in] newOffset the offset in i, j, and k of the allocated elements.
	bool resize(const Index3D& newSize, const Index3D& newOffset=Index3D(0,0,0)){
		if (newSize==aSize && newOffset==aOff) return true;
		int newLinearSize = newSize.x*newSize.y*newSize.z;
		if (newLinearSize == 0){clear(); return true;}

		std::vector<T> newData;
		try {newData.resize(newLinearSize, defaultValue);} //new data
		catch (std::bad_alloc&){return false;} //couldn't get the memory

		//iterate through overlapping region
		Index3D oldMin = aOff, oldMax=aOff+aSize, newMin=newOffset, newMax=newOffset+newSize; //for readability: old and new min and max indices
		Index3D minOverlap(LOCALMAX(oldMin.x, newMin.x), LOCALMAX(oldMin.y, newMin.y), LOCALMAX(oldMin.z, newMin.z)); //minimum of overlapping range
		Index3D maxOverlap(LOCALMIN(oldMax.x, newMax.x), LOCALMIN(oldMax.y, newMax.y), LOCALMIN(oldMax.z, newMax.z)); //maximum of overlapping range
		for (int k=minOverlap.z; k<maxOverlap.z; k++){
			for (int j=minOverlap.y; j<maxOverlap.y; j++){
				for (int i=minOverlap.x; i<maxOverlap.x; i++){ //optimize out this loop with memcpy if it's too slow
					newData[(i-newOffset.x)+newSize.x*(j-newOffset.y) + newSize.x*newSize.y*(k-newOffset.z)]=data[getIndexFast(i,j,k)];
				}
			}
		}
		try {data=newData;} //new data
		catch (std::bad_alloc&){return false;} //couldn't get the memory

		aSize = newSize;
		aOff = newOffset;

		//update min/max
		if (cMin.x < aOff.x) cMin.x = aOff.x;
		if (cMax.x > aSize.x + aOff.x) cMax.x = aSize.x + aOff.x;
		if (cMin.y < aOff.y) cMin.y = aOff.y;
		if (cMax.y > aSize.y + aOff.y) cMax.y = aSize.y + aOff.y;
		if (cMin.z < aOff.z) cMin.z = aOff.z;
		if (cMax.z > aSize.z + aOff.z) cMax.z = aSize.z + aOff.z;

		return true;
	}
	bool resize(int iSize, int jSize, int kSize, int iOffset=0, int jOffset=0, int kOffset=0){return resize(Index3D(iSize, jSize, kSize), Index3D(iOffset, jOffset, kOffset));} //!< Resize the internal data allocation to new specified sizes and offsets in i j and k. Any data ouside the new range is discarded. The range of allocated values in a given dimension spans from iOffset to iOffset+iSize (and the same for j and k. @param[in] iSize the number of elements in i. @param[in] jSize the number of elements in j. @param[in] kSize the number of elements in k. @param[in] iOffset the offset of allocated i elements. @param[in] jOffset the offset of allocated j elements. @param[in] kOffset the offset of allocated k elements.

	//! Deallocates as much memory as possible by reducing the allocated area to minimum span of existing elements.
	bool shrink_to_fit(){
		return resize(cMax-cMin+Index3D(1,1,1), cMin);
	}

	//!Adds a value to the array or overwrites what was there. Allocates more space if needed in a (semi smart) manner. Use removeValue to remove it. @param[in] index the index to add this value at. @param[in] value The value to add.
	bool addValue(const Index3D& index, T value){
		if (value==defaultValue){ //catch if adding default value (equivalent to removeValue). Call removeValue to keep min and max up-to-date
			removeValue(index);
			return true;
		}

		int ThisIndex = getIndex(index);
		if (ThisIndex != -1){data[ThisIndex] = value;}
		else { //reallocation required
			int attempt=0;
			bool success=false;
			int scaleDivisor = 1; //attempts to add asize/scaleDivisor to any dimension that is exceeded

			while (!success){
				attempt++; //start with attempt 1
				switch (attempt){
				case 2: if (!shrink_to_fit()) return false; break; //if failed first time, try to free up some memory in other dimensions. if can't free up memory, bail!
				case 3: scaleDivisor=2; break;
				case 4: scaleDivisor=4; break;
				case 5: scaleDivisor=8; break;
				case 6: return false;
				}

				Index3D aNewMin=aOff;
				Index3D aNewMax=aOff+aSize;

				if (aNewMin==aNewMax){ //if no allocated space, start with +/- 2 in all dimensions
					aNewMin = index-Index3D(2,2,2);
					aNewMax = index+Index3D(2,2,2);
				}
				else { //if there's some allocated space, double the size (or keep going if we added a point way out...)
					while (index.x<=aNewMin.x) aNewMin.x -= aSize.x/scaleDivisor;
					while (index.x>=aNewMax.x) aNewMax.x += aSize.x/scaleDivisor;
					while (index.y<=aNewMin.y) aNewMin.y -= aSize.y/scaleDivisor;
					while (index.y>=aNewMax.y) aNewMax.y += aSize.y/scaleDivisor;
					while (index.z<=aNewMin.z) aNewMin.z -= aSize.z/scaleDivisor;
					while (index.z>=aNewMax.z) aNewMax.z += aSize.z/scaleDivisor;
				}
			
				if (resize(aNewMax-aNewMin, aNewMin)){
					ThisIndex = getIndex(index); //new index since we changed allocated size
					if (ThisIndex == -1) return false; //this should never happen, but check to make sure!
					data[ThisIndex] = value;
					success=true;
				}
			}
		}

		//keep track of min and max
		if (index.x < cMin.x) cMin.x = index.x;
		if (index.x > cMax.x) cMax.x = index.x;
		if (index.y < cMin.y) cMin.y = index.y;
		if (index.y > cMax.y) cMax.y = index.y;
		if (index.z < cMin.z) cMin.z = index.z;
		if (index.z > cMax.z) cMax.z = index.z;
		return true;
	}
	bool addValue(int i, int j, int k, T value){return addValue(Index3D(i,j,k), value);} //!< Adds a value to the array or overwrites what was there. Allocates more space if needed in a (semi smart) manner. Use removeValue to remove it. @param[in] i The i index to add this value at. @param[in] j The j index to add this value at. @param[in] k The k index to add this value at. @param[in] value The value to add.


	//! Removes any value at the specified index and returns its value to the default value. Never triggers a reallocation - use shrink_to_fit() to try to reduce the memory usage after removing element(s). @param[in] index the index to remove a value from if it exists.
	void removeValue(const Index3D& index){
		int ThisIndex = getIndex(index);
		if (ThisIndex == -1 || data[ThisIndex] == defaultValue) return; //already not there...
		data[ThisIndex] = defaultValue;
		UpdateMinMax();
	}
	void removeValue(int i, int j, int k){removeValue(Index3D(i,j,k));} //!< Removes any value at the specified index and returns its value to the default value. Never triggers a reallocation - use shrink_to_fit() to try to reduce the memory usage after removing element(s). Use removeValue to remove it. @param[in] i The i index to remove a value from if it exists. @param[in] j The j index to remove a value from if it exists. @param[in] k The k index to remove a value from if it exists.

private:

	int getIndex(const Index3D& i3D) const { //returns the 1D index anywhere in allocated space or -1 if requested index is unallocated
		if (i3D.x<aOff.x || i3D.x >= aOff.x+aSize.x || i3D.y<aOff.y || i3D.y >= aOff.y+aSize.y || i3D.z<aOff.z || i3D.z >= aOff.z+aSize.z) return -1; //if this XYZ is out of the area
		else return (i3D.x-aOff.x) + aSize.x*(i3D.y-aOff.y) + aSize.x*aSize.y*(i3D.z-aOff.z);
	}
	int getIndexFast(int inX, int inY, int inZ) const { //returns the 1D index anywhere in allocated space (no safety checks!)
		return (inX-aOff.x) + aSize.x*(inY-aOff.y) + aSize.x*aSize.y*(inZ-aOff.z);
	}

	void UpdateMinMax(){ //slow, but easy to start. optimized versions can come if needed.
		cMin = Index3D(INT_MAX, INT_MAX, INT_MAX);
		cMax = Index3D(INT_MIN, INT_MIN, INT_MIN);

		for (int k=aOff.z; k<aSize.z+aOff.z; k++){
			for (int j=aOff.y; j<aSize.y+aOff.y; j++){
				for (int i=aOff.x; i<aSize.x+aOff.x; i++){
					if (data[getIndexFast(i,j,k)] != defaultValue){ //if there's something here...
						if (i<cMin.x) cMin.x=i; //account for offset at end
						else if (i>cMax.x) cMax.x=i;
						if (j<cMin.y) cMin.y=j;
						else if (j>cMax.y) cMax.y=j;
						if (k<cMin.z) cMin.z=k;
						else if (k>cMax.z) cMax.z=k;
					}
				}
			}
		}

	}

	T defaultValue; //value to fill newly initialized space with
	std::vector<T> data;
	Index3D aSize, aOff; //allocated size and offset
	Index3D cMin, cMax; //current minimum and maximum values in x/y/x currently in 

};

#endif
