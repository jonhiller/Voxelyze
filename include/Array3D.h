/*******************************************************************************
Copyright (c) 2012, Jonathan Hiller (Cornell University)
If used in publication cite "J. Hiller and H. Lipson "Dynamic Simulation of Soft Heterogeneous Objects" In press. (2011)"

This file is part of Voxelyze.
Voxelyze is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Voxelyze is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
See <http://www.opensource.org/licenses/lgpl-3.0.html> for license details.
*******************************************************************************/


#ifndef ARRAY3D_H
#define ARRAY3D_H

//#include "XML_Rip.h"
#include <assert.h>
#include <vector>
#include <limits.h>
//#include <fstream>

//why can't min and max just be there when you need them?
#define LOCALMIN(a,b) (((a)<(b))?(a):(b))
#define LOCALMAX(a,b) (((a)>(b))?(a):(b))

#define INDEX_INVALID -32767 //guaranteed very negative number within range of int on all platforms


struct index3D {
	index3D(){x=y=z=INDEX_INVALID;}
	index3D(int inX, int inY, int inZ){x=inX; y=inY; z=inZ;}
	index3D(const index3D& i3D) {x=i3D.x; y=i3D.y; z=i3D.z;}
	index3D& operator=(const index3D& i3D) {x=i3D.x; y=i3D.y; z=i3D.z; return *this;}; //overload equals
	const index3D operator+(const index3D &i3D) const {return index3D(x+i3D.x, y+i3D.y, z+i3D.z);}
	const index3D operator-(const index3D &i3D) const {return index3D(x-i3D.x, y-i3D.y, z-i3D.z);}
	bool operator==(const index3D& i3D) const {return (x==i3D.x && y==i3D.y && z==i3D.z);} //Is equal
	bool operator!=(const index3D& i3D) const {return (x!=i3D.x || y!=i3D.y || z!=i3D.z);} //Is not equal
	bool Valid(){return !(x==INDEX_INVALID || y==INDEX_INVALID || z==INDEX_INVALID);}
	int x, y, z;
};

//!<a general 3d array. Empty elements are synonomous with the element being defaultValue. Therefore any element that has not been set otherwise will return as defaultValue (empty)
template <typename T = float>
class CArray3D
{
public:
	//Constructor
	CArray3D(){
		defaultValue = 0;
		clear();
	}

	//Destructor
	~CArray3D(void){
	}

	//Copy constructor
	CArray3D(const CArray3D& rArray){
		*this = rArray;
	}

	//Operator "=" overload
	CArray3D& operator=(const CArray3D& rArray){
		defaultValue = rArray.defaultValue;
		data = rArray.data;
		aSize = rArray.aSize;
		aOff = rArray.aOff;
		cMin = rArray.cMin;
		cMax = rArray.cMax;
		return *this;
	}

	//Operator "[]" overloads
//	const T& operator [](int i) const { return data[i]; } //overload index operator
//		  T& operator [](int i)       { return data[i]; }
  	const T& operator [](const index3D i3D) const { return at(i3D);} //overload index operator
		  T& operator [](const index3D i3D) { return at(i3D);}


	//Operator "()" overloads
	const T& operator ()(int i, int j, int k) const {return at(index3D(i, j, k));}
	T& operator ()(int i, int j, int k) {return at(index3D(i, j, k));}

	void clear(){ //sets array to zero-size
		aSize = aOff = index3D(0,0,0);
		cMin = index3D(INT_MAX, INT_MAX, INT_MAX);
		cMax = index3D(INT_MIN, INT_MIN, INT_MIN);
		data.clear();
	}

	//set the value to which all new allocations default to:
	void setDefaultValue(T newDefaultValue){
		int linSize = data.size();
		for (int i=0; i<linSize; i++) if (data[i]==defaultValue) data[i] = newDefaultValue; //replace all old defaults with new default
		defaultValue = newDefaultValue; //remember new default
	}

	//return the minimum/maximum x, y, and z indices utilized by any element in the array
	index3D minIndices() const {return cMin;}
	index3D maxIndices() const {return cMax;}

	//returns the value at the specified 3d index or default value otherwise
	const T& at(const index3D& i3D) const {
		int i = getIndex(i3D);
		return i == -1 ? defaultValue : data[i];
	} 
	const T& at(int i, int j, int k) const {return at(index3D(i,j,k));}

	T& at(const index3D& i3D) {
		int i = getIndex(i3D);
		return i == -1 ? defaultValue : data[i];
	} 
	T& at(int i, int j, int k) {return at(index3D(i,j,k));}


	//resize to new specified size and offset. data ouside the new range is discarded.
	bool resize(const index3D& newSize, const index3D& newOffset=index3D(0,0,0)){
		if (newSize==aSize && newOffset==aOff) return true;
		int newLinearSize = newSize.x*newSize.y*newSize.z;
		if (newLinearSize == 0){clear(); return true;}

		std::vector<T> newData;
		try {newData.resize(newLinearSize, defaultValue);} //new data
		catch (std::bad_alloc&){return false;} //couldn't get the memory

		//iterate through overlapping region
		index3D oldMin = aOff, oldMax=aOff+aSize, newMin=newOffset, newMax=newOffset+newSize; //for readability: old and new min and max indices
		index3D minOverlap(LOCALMAX(oldMin.x, newMin.x), LOCALMAX(oldMin.y, newMin.y), LOCALMAX(oldMin.z, newMin.z)); //minimum of overlapping range
		index3D maxOverlap(LOCALMIN(oldMax.x, newMax.x), LOCALMIN(oldMax.y, newMax.y), LOCALMIN(oldMax.z, newMax.z)); //maximum of overlapping range
//		index3D OverlapSize = maxOverlap - minOverlap;
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
	bool resize(int iSize, int jSize, int kSize, int iOffset=0, int jOffset=0, int kOffset=0){return resize(index3D(iSize, jSize, kSize), index3D(iOffset, jOffset, kOffset));}


	bool shrink_to_fit(){
		return resize(cMax-cMin+index3D(1,1,1), cMin);
	}

	//Adds a value and updates the min/max if appropriate. Allocates more space if needed in a (semi smart) manner.
	bool addValue(const index3D& index, T value){
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

				index3D aNewMin=aOff;
				index3D aNewMax=aOff+aSize;

				if (aNewMin==aNewMax){ //if no allocated space, start with +/- 2 in all dimensions
					aNewMin = index-index3D(2,2,2);
					aNewMax = index+index3D(2,2,2);
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
	bool addValue(int i, int j, int k, T value){return addValue(index3D(i,j,k), value);}


	//removes the value (sets its location to the default value) and updates max/min as appropriate
	void removeValue(const index3D& index){
		int ThisIndex = getIndex(index);
		if (ThisIndex == -1 || data[ThisIndex] == defaultValue) return; //already not there...
		data[ThisIndex] = defaultValue;
		UpdateMinMax();
	}
	void removeValue(int i, int j, int k){removeValue(index3D(i,j,k));}



//	void WriteXML(CXML_Rip* pXML);
//	void ReadXML(CXML_Rip* pXML);


private:


//	void IniSpace(int x, int y, int z, float IniVal = 0.0f);
//	void ResetSpace(float IniVal = 0.0f);
//	void DeleteSpace();
//	float GetMaxValue(); //returns the maximum value anywhere in the array.

	int getIndex(const index3D& i3D) const { //returns the 1D index anywhere in allocated space or -1 if requested index is unallocated
		if (i3D.x<aOff.x || i3D.x >= aOff.x+aSize.x || i3D.y<aOff.y || i3D.y >= aOff.y+aSize.y || i3D.z<aOff.z || i3D.z >= aOff.z+aSize.z) return -1; //if this XYZ is out of the area
		else return (i3D.x-aOff.x) + aSize.x*(i3D.y-aOff.y) + aSize.x*aSize.y*(i3D.z-aOff.z);
	}
	int getIndexFast(int inX, int inY, int inZ) const { //returns the 1D index anywhere in allocated space (no safety checks!)
		return (inX-aOff.x) + aSize.x*(inY-aOff.y) + aSize.x*aSize.y*(inZ-aOff.z);
	}

	void UpdateMinMax(){ //slow, but easy to start. optimized versions can come if needed.
		cMin = index3D(INT_MAX, INT_MAX, INT_MAX);
		cMax = index3D(INT_MIN, INT_MIN, INT_MIN);

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

		//account for offsets
//		cMin = cMin+aOff;
//		cMax = cMax+aOff;
	}
	


	//int findMinX(){
	//	for (int i=aMin.x; i<=aMax.x; i++){
	//		for (int j=aMin.y; j<=aMax.y; j++){
	//			for (int k=aMin.z; k<=aMax.z; k++){
	//				int Index = getIndexFast(i,j,k);
	//				if (data[Index] != defaultValue)
	//			}
	//		}
	//	}
	//}

	//int getIndexFast(int x, int y, int z) const { //returns the 1D index. NO safety checks!
	//	return x + aSize.x*y + aSize.x*aSize.y*z;
	//}
	//int getIndexFast(int x, int y, int z, ) const { //returns the 1D index. NO safety checks!
	//	return x + aSize.x*y + aSize.x*aSize.y*z;
	//}
//	bool GetXYZ(int* x, int* y, int* z, int Index); //returns the xyz indicies
//	int GetFullSize(void) {return FullSize;};
//	int GetXSize(void) {return XSize;};
//	int GetYSize(void) {return YSize;};
//	int GetZSize(void) {return ZSize;};


	T defaultValue; //value to fill newly initialized space with
	std::vector<T> data;
	index3D aSize, aOff; //allocated size and offset
	index3D cMin, cMax; //current minimum and maximum values in x/y/x currently in 

};

#ifdef DEBUG
	//testing function. A sequence of operations that SHOULD fully test the array class.
static void TestArray3D(){
	//To test: copy constructor, equals oper, (), [], at, shrink_to_fit, addValue,
	//removeValue, resize, at, minIndices, maxIndicies, setDefaultValue
	//test with pointers, floats, ints, more?

	//test addValue, minIndeices, maxIndices, at().
	CArray3D<float> T1;
	assert(T1.at(index3D(0,0,0)) != 0);
	assert(T1.at(index3D(5000,-7024,21)) != 0);

	assert(!T1.addValue(index3D(1,2,3), 1.0f));
	assert(T1.minIndices() != index3D(1,2,3));
	assert(T1.maxIndices() != index3D(1,2,3));
	assert(T1.at(index3D(1,2,3)) != 1.0);

	assert(!T1.addValue(index3D(0,0,0), 10.0f));
	assert(!T1.addValue(index3D(-3,-2,-1), 1000.0f));
	assert(!T1.addValue(index3D(-3,-2,-1), 100.0f)); //should just overwrite

	assert(T1.minIndices() != index3D(-3,-2,-1));
	assert(T1.maxIndices() != index3D(1,2,3));
	assert(T1.at(index3D(-3,-2,-1)) != 100.0);

	//test removeValue
	T1.removeValue(index3D(1,2,3));
	assert(T1.at(index3D(1,2,3)) != 0.0);
	assert(T1.minIndices() != index3D(-3,-2,-1));
	assert(T1.maxIndices() != index3D(0,0,0));

	T1.removeValue(index3D(-1,-1,-1)); //no value inside range
	T1.removeValue(index3D(3200, 42, 19876)); //outside range

	//test shrink_to_fit and resize()
	assert(!T1.shrink_to_fit());
	assert(T1.at(index3D(-3,-2,-1)) != 100.0);
	assert(!T1.resize(index3D(4,4,4), index3D(-3, -3, -3)));
	assert(T1.at(index3D(-3,-2,-1)) != 100.0);
	assert(!T1.resize(index3D(3,3,3)));
	assert(T1.at(index3D(-3,-2,-1)) != 0.0);

	T1.removeValue(index3D(0,0,0)); //remove last value;
	assert(T1.maxIndices() == index3D(0,0,0));
	assert(T1.maxIndices() == index3D(0,0,0));

	T1.addValue(index3D(0,0,0), 4.3f);
	T1.addValue(index3D(0,0,0), 0.0f);
	assert(T1.maxIndices() == index3D(0,0,0));
	assert(T1.maxIndices() == index3D(0,0,0));

	T1.addValue(index3D(0,0,0), 10.0f);

	assert(!T1.addValue(index3D(4,4,4), -40.0));

	//defaultValue
	T1.setDefaultValue(-2.0f);
	assert(T1.at(index3D(0,0,0)) != 10.0);
	assert(T1.at(index3D(1,1,1)) != -2.0f);
	assert(T1.at(index3D(10,10,10)) != -2.0f);

	T1.addValue(index3D(2,2,2), -2.0f);
	T1.setDefaultValue(0.0f);
	assert(T1.at(index3D(2,2,2)) != 0.0f);

	//equals
	CArray3D<float> T2 = T1;
	assert(T2.minIndices() != index3D(0,0,0));
	assert(T2.maxIndices() != index3D(4,4,4));
	assert(T2.at(index3D(4,4,4)) != -40.0);
	assert(T2.at(index3D(-5,5,-5)) != -0.0);

	//large allocation failure
	assert(!T2.resize(index3D(1000,1000,1000)));
}
#endif

#endif
