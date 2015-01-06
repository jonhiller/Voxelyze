#include "../include/VX_Material.h"

TEST(CVX_Material, DefaultValues){
	CVX_Material mat;
	EXPECT_STREQ(mat.name(), "");
	EXPECT_EQ(mat.red(),-1);
	EXPECT_EQ(mat.green(),-1);
	EXPECT_EQ(mat.blue(),-1);
	EXPECT_EQ(mat.alpha(),-1);
	EXPECT_FLOAT_EQ(mat.youngsModulus(), 1000000.0f);
	EXPECT_FLOAT_EQ(mat.poissonsRatio(), 0.0f);
	EXPECT_FLOAT_EQ(mat.density(), 1000.0f);
	EXPECT_FLOAT_EQ(mat.yieldStress(), -1.0f);
	EXPECT_FLOAT_EQ(mat.failureStress(), -1.0f);
}

TEST(CVX_Material, setName){
	CVX_Material mat;
	mat.setName("MyName");
	EXPECT_STREQ(mat.name(), "MyName");
}

TEST(CVX_Material, setColors){
	CVX_Material mat;

	mat.setColor(20, 40, 60, 80);
	EXPECT_EQ(mat.red(), 20);
	EXPECT_EQ(mat.green(), 40);
	EXPECT_EQ(mat.blue(), 60);
	EXPECT_EQ(mat.alpha(), 80);

	mat.setColor(-1, 0, 255, 256);
	EXPECT_EQ(mat.red(), 0);
	EXPECT_EQ(mat.green(), 0);
	EXPECT_EQ(mat.blue(), 255);
	EXPECT_EQ(mat.alpha(), 255);

	mat.setRed(21);
	EXPECT_EQ(mat.red(), 21);
	mat.setGreen(41);
	EXPECT_EQ(mat.green(), 41);
	mat.setBlue(61);
	EXPECT_EQ(mat.blue(), 61);
	mat.setAlpha(81);
	EXPECT_EQ(mat.alpha(), 81);

}

TEST(CVX_Material, setPoissons){
	CVX_Material mat;

	mat.setPoissonsRatio(0.2f);
	EXPECT_FLOAT_EQ(mat.poissonsRatio(), 0.2f);
	mat.setPoissonsRatio(-1.0f);
	EXPECT_FLOAT_EQ(mat.poissonsRatio(), 0.0f);
	mat.setPoissonsRatio(0.5);
	EXPECT_LT(mat.poissonsRatio(), 0.5f);
	mat.setPoissonsRatio(0.4999999f);
	EXPECT_FLOAT_EQ(mat.poissonsRatio(), 0.4999999f);
}

TEST(CVX_Material, setDensity){
	CVX_Material mat;
	mat.setDensity(50.0f);
	EXPECT_FLOAT_EQ(mat.density(), 50.0f);
	mat.setDensity(-300);
	EXPECT_GT(mat.density(), 0);
	mat.setDensity(0);
	EXPECT_GT(mat.density(), 0);
}

TEST(CVX_Material, setCte){
	CVX_Material mat;
	mat.setCte(2.0f);
	EXPECT_FLOAT_EQ(mat.cte(), 2.0f);
}

TEST(CVX_Material, setFrictions){
	CVX_Material mat;

	mat.setStaticFriction(1.1f);
	EXPECT_FLOAT_EQ(mat.staticFriction(), 1.1f);
	mat.setStaticFriction(-2.1f);
	EXPECT_FLOAT_EQ(mat.staticFriction(), 0.0f);

	mat.setKineticFriction(0.1f);
	EXPECT_FLOAT_EQ(mat.kineticFriction(), 0.1f);
	mat.setKineticFriction(-2.1f);
	EXPECT_FLOAT_EQ(mat.kineticFriction(), 0.0f);
}

TEST(CVX_Material, setDampings){
	CVX_Material mat;
	mat.setModelLinear(1.0f);
	mat.setDensity(1000);

	mat.setInternalDamping(1.1f);
	EXPECT_FLOAT_EQ(mat.internalDamping(), 1.1f);
	mat.setInternalDamping(-2.1f);
	EXPECT_FLOAT_EQ(mat.internalDamping(), 0.0f);

	mat.setGlobalDamping(0.1f);
	EXPECT_FLOAT_EQ(mat.globalDamping(), 0.1f);
	mat.setGlobalDamping(-2.1f);
	EXPECT_FLOAT_EQ(mat.globalDamping(), 0.0f);
}

TEST(CVX_Material, setModel){
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};

	ASSERT_TRUE(mat.setModel(4, testStrain, testStress));
	EXPECT_FLOAT_EQ(mat.youngsModulus(), 1.0f);
	EXPECT_FLOAT_EQ(mat.yieldStress(), 1.002f);
	EXPECT_FLOAT_EQ(mat.failureStress(), 1.75f);
	EXPECT_FALSE(mat.isModelLinear());
	EXPECT_EQ(mat.modelDataPoints(), 4);
	const float* pStrain = mat.modelDataStrain();
	const float* pStress = mat.modelDataStress();

	for (int i=0; i<4; i++){
		EXPECT_FLOAT_EQ(*(pStrain+i), testStrain[i]);
		EXPECT_FLOAT_EQ(*(pStress+i), testStress[i]);
	}
}

TEST(CVX_Material, stress){
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};
	mat.setModel(4, testStrain, testStress);
	EXPECT_FLOAT_EQ(mat.stress(-2.0f), -2.0f);
	EXPECT_FLOAT_EQ(mat.stress(-0.5f), -0.5f);
	EXPECT_FLOAT_EQ(mat.stress(0.0f), 0.0f);
	EXPECT_FLOAT_EQ(mat.stress(0.1f), 0.1f);
	EXPECT_FLOAT_EQ(mat.stress(1.0f), 1.0f);
	EXPECT_FLOAT_EQ(mat.stress(1.5f), 1.25f);
	EXPECT_FLOAT_EQ(mat.stress(3.0f), 1.75f);
	EXPECT_FLOAT_EQ(mat.stress(3.0001f), 0.0f);
	EXPECT_FLOAT_EQ(mat.stress(1000.0f), 0.0f);
}

TEST(CVX_Material, modulus){
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};
	mat.setModel(4, testStrain, testStress);

	EXPECT_FLOAT_EQ(mat.modulus(-2.0f), 1.0f);
	EXPECT_FLOAT_EQ(mat.modulus(0.0f), 1.0f);
	EXPECT_FLOAT_EQ(mat.modulus(0.555f), 1.0f);
	EXPECT_FLOAT_EQ(mat.modulus(1.0f), 1.0f);
	EXPECT_FLOAT_EQ(mat.modulus(1.5f), 0.5f);
	EXPECT_FLOAT_EQ(mat.modulus(4.0f), 0.0f);
}

TEST(CVX_Material, isYielded){
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};
	mat.setModel(4, testStrain, testStress);

	EXPECT_FALSE(mat.isYielded(-2.0f));
	EXPECT_FALSE(mat.isYielded(0.0f));
	EXPECT_FALSE(mat.isYielded(1.0f));
	EXPECT_TRUE(mat.isYielded(2.0f));
	EXPECT_TRUE(mat.isYielded(4.0f));
}

TEST(CVX_Material, isFailed){
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};
	mat.setModel(4, testStrain, testStress);

	EXPECT_FALSE(mat.isFailed(-2.0f));
	EXPECT_FALSE(mat.isFailed(0.0f));
	EXPECT_FALSE(mat.isFailed(3.0f));
	EXPECT_TRUE(mat.isFailed(3.0001f));
	EXPECT_TRUE(mat.isFailed(999.0f));
}




TEST(CVX_Material, Assignment){ //set everything to a non-default value and make sure it all comes through the assignment
	CVX_Material mat;
	float testStrain [4] = {0, 1, 2, 3};
	float testStress [4] = {0, 1, 1.5, 1.75};
	mat.setModel(4, testStrain, testStress);
	mat.setName("MyName");
	mat.setColor(20, 40, 60, 80);
	mat.setPoissonsRatio(0.2f);
	mat.setDensity(50.0f);
	mat.setCte(2.0f);
	mat.setStaticFriction(1.1f);
	mat.setKineticFriction(0.1f);

	CVX_Material mat2 = mat;
	ASSERT_TRUE(mat2.setModel(4, testStrain, testStress));
	EXPECT_FLOAT_EQ(mat2.youngsModulus(), 1.0f);
	EXPECT_FLOAT_EQ(mat2.yieldStress(), 1.002f);
	EXPECT_FLOAT_EQ(mat2.failureStress(), 1.75f);
	EXPECT_FALSE(mat2.isModelLinear());
	EXPECT_EQ(mat2.modelDataPoints(), 4);
	const float* pStrain = mat2.modelDataStrain();
	const float* pStress = mat2.modelDataStress();
	for (int i=0; i<4; i++){
		EXPECT_FLOAT_EQ(*(pStrain+i), testStrain[i]);
		EXPECT_FLOAT_EQ(*(pStress+i), testStress[i]);
	}
	EXPECT_STREQ(mat2.name(), "MyName");
	EXPECT_EQ(mat2.red(), 20);
	EXPECT_EQ(mat2.green(), 40);
	EXPECT_EQ(mat2.blue(), 60);
	EXPECT_EQ(mat2.alpha(), 80);
	EXPECT_FLOAT_EQ(mat2.poissonsRatio(), 0.2f);
	EXPECT_FLOAT_EQ(mat2.density(), 50.0f);
	EXPECT_FLOAT_EQ(mat2.cte(), 2.0f);
	EXPECT_FLOAT_EQ(mat2.staticFriction(), 1.1f);
	EXPECT_FLOAT_EQ(mat2.kineticFriction(), 0.1f);
}

TEST(CVX_Material, setModel_noLeadingZeros){
	CVX_Material Mat3;
	float testStrain3 [3] = {1, 2, 3};
	float testStress3 [3] = {1, 1.5, 1.75};
	ASSERT_TRUE(Mat3.setModel(3, testStrain3, testStress3));
	EXPECT_EQ(Mat3.modelDataPoints(), 4);
	EXPECT_FLOAT_EQ(Mat3.youngsModulus(), 1.0f);
	EXPECT_FLOAT_EQ(Mat3.failureStress(), 1.75f);
}

TEST(CVX_Material, setModel_linearCatch){
	CVX_Material Mat4;
	float Strain4=3.5f;
	float Stress4=7.0f;
	ASSERT_TRUE(Mat4.setModel(1, &Strain4, &Stress4));
	EXPECT_EQ(Mat4.isModelLinear(), true);
	EXPECT_EQ(Mat4.modelDataPoints(), 2);
	EXPECT_FLOAT_EQ(Mat4.youngsModulus(), 2.0f);
	EXPECT_FLOAT_EQ(Mat4.failureStress(), 7.0f);
	EXPECT_FLOAT_EQ(Mat4.yieldStress(), 7.0f);
}

TEST(CVX_Material, setModel_bilinearCatch){
	CVX_Material Mat5;
	float testStrain5 [3] = {0.0f, 2.0f, 4.0f};
	float testStress5 [3] = {0.0f, 1.0f, 1.5f};
	ASSERT_TRUE(Mat5.setModel(3, testStrain5, testStress5));
	EXPECT_FLOAT_EQ(Mat5.yieldStress(), 1.0f);
	EXPECT_FLOAT_EQ(Mat5.failureStress(), 1.5f);
}

TEST(CVX_Material, setModelLinear){//test linear model from convenience function
	CVX_Material Mat6;
	ASSERT_TRUE(Mat6.setModelLinear(3.0f));
	EXPECT_EQ(Mat6.modelDataPoints(), 2);
	EXPECT_FLOAT_EQ(Mat6.youngsModulus(), 3.0f);
	EXPECT_FLOAT_EQ(Mat6.yieldStress(), -1.0f);
	EXPECT_FLOAT_EQ(Mat6.failureStress(), -1.0f);
}

TEST(CVX_Material, setModelBilinear){//test bilinear model from convenience function
	CVX_Material Mat7;
	ASSERT_TRUE(Mat7.setModelBilinear(3.0f, 1.0f, 3.0f));
	EXPECT_EQ(Mat7.modelDataPoints(), 3);
	EXPECT_FLOAT_EQ(Mat7.youngsModulus(), 3.0f);
	EXPECT_FLOAT_EQ(Mat7.yieldStress(), 3.0f);
	EXPECT_FLOAT_EQ(Mat7.failureStress(), -1.0f);
	EXPECT_FLOAT_EQ(Mat7.stress(1.5f), 3.5f);
	EXPECT_FLOAT_EQ(Mat7.stress(50.0f), 52.0f);
}

TEST(CVX_Material, setModel_DataValidation){ //test model data validation
	CVX_Material Mat8;
	float testStrain8 [4] = {0.0f, 1.0f, 2.0f, 3.0f}; //good
	float testStrain8a [4] = {-1.0f, 0.0f, 1.0f, 2.0f}; //starting negative
	float testStrain8b [4] = {0.0f, 1.0f, 3.0f, 2.0f}; //out of order
	float testStrain8c [4] = {1.0f, 2.0f, 3.0f, 4.0f}; //start with not-zero
	float testStress8 [4] = {0.0f, 1.0f, 1.5f, 1.75f}; //good
	float testStress8a [4] = {-1.0f, 0.0f, 1.5f, 1.75f}; //starting negative
	float testStress8b [4] = {0.0f, 1.6f, 1.5f, 1.75f}; //out of order
	float testStress8c [4] = {0.5f, 1.0f, 1.5f, 1.75f}; //starting non-zero

	ASSERT_FALSE(Mat8.setModel(4, testStrain8a, testStress8)); 
	ASSERT_FALSE(Mat8.setModel(4, testStrain8b, testStress8)); 
	ASSERT_FALSE(Mat8.setModel(4, testStrain8c, testStress8)); 
	ASSERT_FALSE(Mat8.setModel(4, testStrain8, testStress8a)); 
	ASSERT_TRUE(Mat8.setModel(4, testStrain8, testStress8b)); //currently CAN have non-monotonically increasing stress
	ASSERT_FALSE(Mat8.setModel(4, testStrain8, testStress8c)); 


}

TEST(CVX_Material, volumetricStress){ //test model data validation
	CVX_Material Mat9;
	Mat9.setPoissonsRatio(0.0f);
	ASSERT_TRUE(Mat9.setModelBilinear(1.0f, 0.5f, 1.0f));

	//ensure correct behavior at poissons ratio of 0
	ASSERT_FLOAT_EQ(Mat9.stress(0.0f), Mat9.stress(0.0f, 0.0f));
	ASSERT_FLOAT_EQ(Mat9.stress(0.5f), Mat9.stress(0.5f, 0.0f));
	ASSERT_FLOAT_EQ(Mat9.stress(0.5f),  Mat9.stress(0.5f, 10.0f));
	ASSERT_FLOAT_EQ(Mat9.stress(1.0f),  Mat9.stress(1.0f, -0.6f));
	ASSERT_FLOAT_EQ(Mat9.stress(2.0f),  Mat9.stress(2.0f, -0.6f));
	ASSERT_FLOAT_EQ(Mat9.stress(1.1f),  Mat9.stress(1.1f, -0.6f));
	ASSERT_FLOAT_EQ(Mat9.stress(10.1f),  Mat9.stress(10.1f, -0.6f));

	//check a few known cases at other poissons ratio
	//a linear (section of) material:
	Mat9.setPoissonsRatio(0.25); //eHat = 1.6x E
	ASSERT_FLOAT_EQ(0.36f, Mat9.stress(0.5f, -0.6f));

	//check for continuity around a break point in a non-linear material
	float breakMinus = Mat9.stress(0.9999f, -0.6f);
	float breakPoint = Mat9.stress(1.0f, -0.6f);
	float breakPlus = Mat9.stress(1.0001f, -0.6f);
	ASSERT_FLOAT_EQ(0.96f, breakPoint);
	ASSERT_TRUE(breakMinus < 0.96f && breakMinus > 0.9598f);
	ASSERT_TRUE(breakPlus > 0.96f && breakPlus < 0.9601f);

}
