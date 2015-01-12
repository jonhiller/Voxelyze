#include "../include/VX_MaterialLink.h"

TEST(CVX_MaterialLink, combineLinearMaterials){
	//neither with failure stress
	CVX_MaterialVoxel Mat1, Mat2;
	ASSERT_TRUE(Mat1.setModelLinear(1.0f));
	ASSERT_TRUE(Mat2.setModelLinear(10.0f));

	CVX_MaterialLink Mat3(&Mat1, &Mat2);

	ASSERT_FLOAT_EQ(20.0f/11.0f, Mat3.youngsModulus());
	ASSERT_FALSE(Mat3.isFailed(1));

	//one with failure stress
	CVX_MaterialVoxel Mat4, Mat5;
	ASSERT_TRUE(Mat4.setModelLinear(10.0f, 10.0f));
	ASSERT_TRUE(Mat5.setModelLinear(10.0f));

	CVX_MaterialLink Mat6(&Mat4, &Mat5);

	ASSERT_FLOAT_EQ(10.0f, Mat6.youngsModulus());
	ASSERT_FALSE(Mat6.isFailed(0.9f));
	ASSERT_TRUE(Mat6.isFailed(1.1f));

	//both with failure stress, select lower one
	CVX_MaterialVoxel Mat7, Mat8;
	ASSERT_TRUE(Mat7.setModelLinear(10.0f, 30.0f));
	ASSERT_TRUE(Mat8.setModelLinear(1.0f, 2.0f));

	CVX_MaterialLink Mat9(&Mat7, &Mat8);

	ASSERT_FLOAT_EQ(20.0f/11.0f, Mat9.youngsModulus());
	ASSERT_FALSE(Mat9.isFailed(1.0f));
	ASSERT_TRUE(Mat9.isFailed(1.2f));
	ASSERT_TRUE(Mat9.isFailed(5.0f));

}

TEST(CVX_MaterialLink, combineNonlinearMaterials){
	//both with failure stress
	CVX_MaterialVoxel Mat1, Mat2;
	ASSERT_TRUE(Mat1.setModelBilinear(1.0f, 0.5f, 1.0f, 2.0f));
	ASSERT_TRUE(Mat2.setModelBilinear(2.0f, 1.0f, 4.0f, 6.0f));

	CVX_MaterialLink Mat3(&Mat1, &Mat2);

	ASSERT_FLOAT_EQ(4.0f/3.0f, Mat3.youngsModulus());
	ASSERT_FLOAT_EQ(4.0f/3.0f, Mat3.modulus(0.5));
	ASSERT_FLOAT_EQ(2.0f/2.5f, Mat3.modulus(1.5));
	ASSERT_FLOAT_EQ(0.0f, Mat3.modulus(2.5));
	ASSERT_FLOAT_EQ(0.0f, Mat3.modulus(3.5));

	ASSERT_FALSE(Mat3.isFailed(1.8));
	ASSERT_TRUE(Mat3.isFailed(1.9));

	//neither with failure stress
	CVX_MaterialVoxel Mat4, Mat5;
	ASSERT_TRUE(Mat4.setModelBilinear(1.0f, 0.5f, 1.0f));
	ASSERT_TRUE(Mat5.setModelBilinear(2.0f, 1.0f, 4.0f));

	CVX_MaterialLink Mat6(&Mat4, &Mat5);

	ASSERT_FLOAT_EQ(2.0f/2.5f, Mat6.modulus(1.5));
	ASSERT_FLOAT_EQ(1.0/1.5f, Mat6.modulus(2.5));
	ASSERT_FLOAT_EQ(1.0/1.5f, Mat6.modulus(3.5));

	ASSERT_FALSE(Mat6.isFailed(1.8));
	ASSERT_FALSE(Mat6.isFailed(1.9));
	ASSERT_FALSE(Mat6.isFailed(3.5));
}