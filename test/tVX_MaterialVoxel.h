#include "../VX_MaterialVoxel.h"


TEST(CVX_MaterialVoxel, sizes){
	CVX_Material mat;
	mat.setNominalSize(0.002);
	mat.setDensity(100.0f);
	EXPECT_FLOAT_EQ(mat.mass(), 8e-7f);
	EXPECT_FLOAT_EQ(mat.momentInertia(), 3.2e-12f / 6.0f);
	EXPECT_FLOAT_EQ((float)mat.size().x, 0.002f);
	EXPECT_FLOAT_EQ((float)mat.size().y, 0.002f);
	EXPECT_FLOAT_EQ((float)mat.size().z, 0.002f);

	mat.setExternalScaleFactor(1.5f);
	EXPECT_FLOAT_EQ((float)mat.size().x, 0.003f);
	EXPECT_FLOAT_EQ((float)mat.size().y, 0.003f);
	EXPECT_FLOAT_EQ((float)mat.size().z, 0.003f);
	EXPECT_FLOAT_EQ(mat.mass(), 8e-7f); //mass shouldn't change
	//moment of inertian maybe should change, but doesn't currently

	mat.setExternalScaleFactor(Vec3D<double>(0.5f, 1.0f, 1.5f));
	EXPECT_FLOAT_EQ((float)mat.size().x, 0.001f);
	EXPECT_FLOAT_EQ((float)mat.size().y, 0.002f);
	EXPECT_FLOAT_EQ((float)mat.size().z, 0.003f);

	mat.setNominalSize(0.004);
	EXPECT_FLOAT_EQ((float)mat.size().x, 0.002f);
	EXPECT_FLOAT_EQ((float)mat.size().y, 0.004f);
	EXPECT_FLOAT_EQ((float)mat.size().z, 0.006f);

	mat.setExternalScaleFactor(-3);
	EXPECT_FLOAT_EQ((float)mat.size().x, 0.004f*FLT_MIN);
	EXPECT_FLOAT_EQ((float)mat.size().y, 0.004f*FLT_MIN);
	EXPECT_FLOAT_EQ((float)mat.size().z, 0.004f*FLT_MIN);
}
