#include "../include/VX_Voxel.h"

TEST(CVX_Voxel, DefaultValues){
	CVX_MaterialVoxel mat;
	CVX_Voxel vox(&mat, 0, 0, 0);
	EXPECT_FALSE(vox.external()->isFixed(X_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(X_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_ROTATE));


}

TEST(CVX_Voxel, setFixed)
{
	CVX_MaterialVoxel mat;
	CVX_Voxel vox(&mat, 0, 0, 0);

	vox.external()->setFixedAll();
	EXPECT_TRUE(vox.external()->isFixedAll());
	EXPECT_TRUE(vox.external()->isFixed(X_TRANSLATE));
	EXPECT_TRUE(vox.external()->isFixed(Y_TRANSLATE));
	EXPECT_TRUE(vox.external()->isFixed(Z_TRANSLATE));
	EXPECT_TRUE(vox.external()->isFixed(X_ROTATE));
	EXPECT_TRUE(vox.external()->isFixed(Y_ROTATE));
	EXPECT_TRUE(vox.external()->isFixed(Z_ROTATE));
	
	vox.external()->setFixedAll(false); // setUnfixedAll();
	EXPECT_FALSE(vox.external()->isFixed(X_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(X_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_ROTATE));

	vox.external()->setFixed(X_TRANSLATE, 1.0);
	EXPECT_TRUE(vox.external()->isFixed(X_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixed(X_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Y_ROTATE));
	EXPECT_FALSE(vox.external()->isFixed(Z_ROTATE));

//	vox.external()->setDisplacement(Y_TRANSLATE, -1.0);
//	vox.external()->setDisplacement(Z_TRANSLATE);
//	vox.external()->setDisplacement(X_ROTATE, 1.5707963267949f); //do rotations to bring back to original orientation
//	EXPECT_FLOAT_EQ((float)vox.orientation().ToRotationVector().x, (float)1.5707963267949);
//	vox.external()->setDisplacement(Y_ROTATE, (float)-1.5707963267949);
//	vox.external()->setDisplacement(Z_ROTATE, (float)-1.5707963267949);
//	vox.external()->setDisplacement(Y_ROTATE, (float)1.5707963267949);
//
//	EXPECT_FLOAT_EQ((float)vox.position().x, 1.0f);
//	EXPECT_FLOAT_EQ((float)vox.position().y, -1.0f);
//	EXPECT_FLOAT_EQ((float)vox.position().z, 0.0f);
//	EXPECT_NEAR((float)vox.orientation().ToRotationVector().x, 0, 1e-7f);
//	EXPECT_NEAR((float)vox.orientation().ToRotationVector().y, 0, 1e-7f);
//	EXPECT_NEAR((float)vox.orientation().ToRotationVector().z, 0, 1e-7f);
	
	vox.external()->setFixed(X_TRANSLATE, false); //setUnfixed(X_TRANSLATE);
	EXPECT_FALSE(vox.external()->isFixed(X_TRANSLATE));
	EXPECT_FALSE(vox.external()->isFixedAll());





}