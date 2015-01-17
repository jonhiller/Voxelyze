#include "../include/Array3D.h"

TEST(Array3D, all){
		//To test: copy constructor, equals oper, (), [], at, shrink_to_fit, addValue,
	//removeValue, resize, at, minIndices, maxIndicies, setDefaultValue
	//test with pointers, floats, ints, more?

	//test addValue, minIndeices, maxIndices, at().
	CArray3D<float> T1;
	EXPECT_EQ(T1.at(Index3D(0,0,0)), 0);
	EXPECT_EQ(T1.at(Index3D(5000,-7024,21)), 0);

	EXPECT_TRUE(T1.addValue(Index3D(1,2,3), 1.0f));
	EXPECT_EQ(T1.minIndices(), Index3D(1,2,3));
	EXPECT_EQ(T1.maxIndices(), Index3D(1,2,3));
	EXPECT_EQ(T1.at(Index3D(1,2,3)), 1.0);

	EXPECT_TRUE(T1.addValue(Index3D(0,0,0), 10.0f));
	EXPECT_TRUE(T1.addValue(Index3D(-3,-2,-1), 1000.0f));
	EXPECT_TRUE(T1.addValue(Index3D(-3,-2,-1), 100.0f)); //should just overwrite

	EXPECT_EQ(T1.minIndices(), Index3D(-3,-2,-1));
	EXPECT_EQ(T1.maxIndices(), Index3D(1,2,3));
	EXPECT_EQ(T1.at(Index3D(-3,-2,-1)), 100.0);

	//test removeValue
	T1.removeValue(Index3D(1,2,3));
	EXPECT_EQ(T1.at(Index3D(1,2,3)), 0.0);
	EXPECT_EQ(T1.minIndices(), Index3D(-3,-2,-1));
	EXPECT_EQ(T1.maxIndices(), Index3D(0,0,0));

	T1.removeValue(Index3D(-1,-1,-1)); //no value inside range
	T1.removeValue(Index3D(3200, 42, 19876)); //outside range

	//test shrink_to_fit and resize()
	EXPECT_TRUE(T1.shrink_to_fit());
	EXPECT_EQ(T1.at(Index3D(-3,-2,-1)), 100.0);
	EXPECT_TRUE(T1.resize(Index3D(4,4,4), Index3D(-3, -3, -3)));
	EXPECT_EQ(T1.at(Index3D(-3,-2,-1)), 100.0);
	EXPECT_TRUE(T1.resize(Index3D(3,3,3)));
	EXPECT_EQ(T1.at(Index3D(-3,-2,-1)), 0.0);

	T1.removeValue(Index3D(0,0,0)); //remove last value;
	EXPECT_NE(T1.maxIndices(), Index3D(0,0,0));
	EXPECT_NE(T1.maxIndices(), Index3D(0,0,0));

	T1.addValue(Index3D(0,0,0), 4.3f);
	EXPECT_EQ(T1.maxIndices(), Index3D(0,0,0));
	T1.addValue(Index3D(0,0,0), 0.0f);
	EXPECT_NE(T1.maxIndices(), Index3D(0,0,0));

	T1.addValue(Index3D(0,0,0), 10.0f);

	EXPECT_TRUE(T1.addValue(Index3D(4,4,4), -40.0));

	//defaultValue
	T1.setDefaultValue(-2.0f);
	EXPECT_EQ(T1.at(Index3D(0,0,0)), 10.0);
	EXPECT_EQ(T1.at(Index3D(1,1,1)), -2.0f);
	EXPECT_EQ(T1.at(Index3D(10,10,10)), -2.0f);

	T1.addValue(Index3D(2,2,2), -2.0f);
	T1.setDefaultValue(0.0f);
	EXPECT_EQ(T1.at(Index3D(2,2,2)), 0.0f);

	//equals
	CArray3D<float> T2 = T1;
	EXPECT_EQ(T2.minIndices(), Index3D(0,0,0));
	EXPECT_EQ(T2.maxIndices(), Index3D(4,4,4));
	EXPECT_EQ(T2.at(Index3D(4,4,4)), -40.0);
	EXPECT_EQ(T2.at(Index3D(-5,5,-5)), -0.0);

	//large allocation failure
	//EXPECT_TRUE(T2.resize(Index3D(1000,1000,1000)));
}
