#include "../include/Mesh3D.h"

TEST(Mesh3D, MarchCubeSimple){
	CArray3Df arr;

	arr.addValue(0,0,0,1.0f);

	CMesh3D mesh(arr, 0.5f);

	EXPECT_EQ(mesh.triangleCount(), 8);
	EXPECT_EQ(mesh.vertexCount(), 24); //uncombined

	EXPECT_EQ(mesh.meshMin().x, -0.5f);
	EXPECT_EQ(mesh.meshMax().x, 0.5f);
	EXPECT_EQ(mesh.meshMin().y, -0.5f);
	EXPECT_EQ(mesh.meshMax().y, 0.5f);
	EXPECT_EQ(mesh.meshMin().z, -0.5f);
	EXPECT_EQ(mesh.meshMax().z, 0.5f);

	mesh.save("MarchCubeSimple.stl");
}

TEST(Mesh3D, MarchCubeScale){
	CArray3Df arr;
	arr.addValue(0,0,0,1.0f);

	CMesh3D mesh(arr, 0.5f, 2.0f);

	EXPECT_EQ(mesh.meshMin().x, -1.0f);
	EXPECT_EQ(mesh.meshMax().x, 1.0f);
}

TEST(Mesh3D, MarchCubeThreshold){
	CArray3Df arr;
	arr.addValue(0,0,0,1.0f);

	CMesh3D mesh(arr, 0.25f);

	EXPECT_EQ(mesh.meshMin().x, -0.75f);
	EXPECT_EQ(mesh.meshMax().x, 0.75f);

	//higher values
	arr.addValue(0,0,0,4.0f);
	CMesh3D mesh2(arr, 2.0f);
	EXPECT_EQ(mesh2.meshMax().x, 0.5f);

	//negative default vaules and threshold
	arr.setDefaultValue(-4.0f);
	arr.addValue(0,0,0,2.0f);
	CMesh3D mesh3(arr, -1.0f);
	EXPECT_EQ(mesh3.meshMax().x, 0.5f);


}


TEST(Mesh3D, MarchCube2){
	CArray3Df arr;

	arr.addValue(0,0,0,1.0f);
	arr.addValue(1,0,0,1.0f);

	CMesh3D mesh(arr, 0.5f);

	EXPECT_EQ(mesh.triangleCount(), 16);
	EXPECT_EQ(mesh.vertexCount(), 48); //uncombined

	EXPECT_EQ(mesh.meshMin().x, -0.5f);
	EXPECT_EQ(mesh.meshMax().x, 1.5f);
	EXPECT_EQ(mesh.meshMin().y, -0.5f);
	EXPECT_EQ(mesh.meshMax().y, 0.5f);
	EXPECT_EQ(mesh.meshMin().z, -0.5f);
	EXPECT_EQ(mesh.meshMax().z, 0.5f);

	mesh.save("MarchCube2.stl");


}