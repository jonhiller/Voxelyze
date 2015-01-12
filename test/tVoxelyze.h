#include "../include/Voxelyze.h"
#include "../include/VX_Voxel.h"
#include "../include/VX_Link.h"

#include <iostream>
#include <fstream>

#define TX 0
#define TY 1
#define TZ 2
#define RX 3
#define RY 4
#define RZ 5
#define cNNN Vec3D<float>(0,0,0)
#define FPX Vec3D<float>(1e-3f,0,0)
#define FNX Vec3D<float>(-1e-3f,0,0)
#define FPY Vec3D<float>(0,1e-3f,0)
#define FNY Vec3D<float>(0,-1e-3f,0)
#define FPZ Vec3D<float>(0,0,1e-3f)
#define FNZ Vec3D<float>(0,0,-1e-3f)
#define MPX Vec3D<float>(1e-9f,0,0)
#define MNX Vec3D<float>(-1e-9f,0,0)
#define MPY Vec3D<float>(0,1e-9f,0)
#define MNY Vec3D<float>(0,-1e-9f,0)
#define MPZ Vec3D<float>(0,0,1e-9f)
#define MNZ Vec3D<float>(0,0,-1e-9f)

Vec3D<> toOffset(CVX_Voxel::linkDirection dir){
	Vec3D<> off;
	switch (dir){
	case CVX_Voxel::X_POS: off = Vec3D<>(1,0,0);  break;
	case CVX_Voxel::X_NEG: off = Vec3D<>(-1,0,0); break;
	case CVX_Voxel::Y_POS: off = Vec3D<>(0,1,0);  break;
	case CVX_Voxel::Y_NEG: off = Vec3D<>(0,-1,0); break;
	case CVX_Voxel::Z_POS: off = Vec3D<>(0,0,1);  break;
	case CVX_Voxel::Z_NEG: off = Vec3D<>(0,0,-1); break;
	default: off = Vec3D<>(0,0,0);
	}
	return off;
}

float calcPeriod(double timestep, std::vector<double> data){ //detects zero crossings to get period
	std::vector<double> zeroCrossings;

	for(int i=1; i<(int)data.size(); i++){
		if ((data[i-1]<=0 && data[i] > 0) || (data[i-1]>=0 && data[i] < 0)){ //positive slope crossing OR negative 
			double perc = data[i-1] / (data[i-1] - data[i]); //linear interp
			zeroCrossings.push_back(timestep*((double)i-1 + perc));
		}
	}

	double acc = 0.0;
	int numAcc = 0;
	if (zeroCrossings.size() < 2) return -1;
	for (int i=1; i<(int)zeroCrossings.size(); i++){
		acc += zeroCrossings[i]-zeroCrossings[i-1];
		numAcc++;
	}

	return (float)(acc/numAcc*2);
}

int test2Vox(bool v1Fixed, CVX_Voxel::linkDirection dir, Vec3D<float> force, Vec3D<float> moment, dofObject dof, int maxTimeSteps, float expectedValue, unsigned char returnData, std::string fileOutName = ""){
	std::ofstream file;
	if (fileOutName != "") file.open("output-CVoxelyze-" + fileOutName + ".txt");

	double sz=0.001;
	CVoxelyze Sim(sz);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.2f);


	Vec3D<> off = toOffset(dir);
	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	if (v1Fixed){
		pV1->external()->setFixedAll();
	}
	else {
		pV1->external()->setForce(-force);
		pV1->external()->setMoment(-moment);
		pV1->external()->setFixed(dofIsSet(dof, X_TRANSLATE), dofIsSet(dof, Y_TRANSLATE), dofIsSet(dof, Z_TRANSLATE), dofIsSet(dof, X_ROTATE), dofIsSet(dof, Y_ROTATE), dofIsSet(dof, Z_ROTATE));
	}

	CVX_Voxel* pV2 = Sim.setVoxel(pMat1, (int)off.x, (int)off.y, (int)off.z);
	pV2->external()->setForce(force);
	pV2->external()->setMoment(moment);
	pV2->external()->setFixed(dofIsSet(dof, X_TRANSLATE), dofIsSet(dof, Y_TRANSLATE), dofIsSet(dof, Z_TRANSLATE), dofIsSet(dof, X_ROTATE), dofIsSet(dof, Y_ROTATE), dofIsSet(dof, Z_ROTATE));
	
	float ts = Sim.recommendedTimeStep();
	int convergeCount = 0; //consecutive times we've got the value we want
	double data = 0;
	int k;
	for (k=0; k<maxTimeSteps; k++){
		Sim.doTimeStep(ts);
		if ((int)returnData/3==0) data = (pV2->position()-off*sz)[(int)returnData%3]; //position
		else {data = pV2->orientation().ToRotationVector()[(int)returnData%3];} //rotation
		
		if (fileOutName != "") file << data << "\n";

	//	if ((float)data == (float)expectedValue) convergeCount++;
		if (abs(data - expectedValue) < abs(expectedValue)*1e-5 || (float)data == (float)expectedValue) convergeCount++; //equals to catch zero expected values
		else convergeCount=0;
		if (convergeCount == 10){
			break; //X consecutive desired values
		}
	}
//	return (float)data; //return final data

	if (fileOutName != "") file.close();

	return k;
}



TEST(CVoxelyze, simpleSetup){
	CVoxelyze Sim(0.001f);
	CVX_Material* pMat1 = Sim.addMaterial();
//	EXPECT_FLOAT_EQ(0.001f, (float)pMat1->nominalSize());
	
	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);

//	EXPECT_TRUE((*Sim.linkList()->begin())->isLinear());

	EXPECT_EQ(0, Sim.indexMinX());
	EXPECT_EQ(1, Sim.indexMaxX());
	EXPECT_EQ(0, Sim.indexMinY());
	EXPECT_EQ(0, Sim.indexMaxY());
	EXPECT_EQ(0, Sim.indexMinZ());
	EXPECT_EQ(0, Sim.indexMaxZ());

	EXPECT_EQ(2, Sim.voxelCount());
	EXPECT_EQ(pMat1, Sim.voxel(0,0,0)->material());
	EXPECT_EQ(pMat1, Sim.voxel(1,0,0)->material());
	EXPECT_EQ(NULL, Sim.voxel(2,0,0));

}


TEST(CVoxelyze, singleBondFixedFree){
	//apply force, look at translations
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,	TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FNX, cNNN, dof(false, true, true, true, true, true), 1000, -1e-6f,	TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 1e-6f,	TY));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FNY, cNNN, dof(true, false, true, true, true, true), 1000, -1e-6f,	TY));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 1e-6f,	TZ));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_POS, FNZ, cNNN, dof(true, true, false, true, true, true), 1000, -1e-6f,	TZ));

	//test an axial and transverse for each other (instead of 6 each)
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,  TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::X_NEG, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 1e-6f,  TY));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Y_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,  TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Y_POS, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 1e-6f,  TY));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Y_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,  TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Y_NEG, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 1e-6f,  TY));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Z_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,  TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Z_POS, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 1e-6f,  TZ));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Z_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 1e-6f,  TX));
	EXPECT_GT(150, test2Vox(true, CVX_Voxel::Z_NEG, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 1e-6f,  TZ));

	//apply moment, look at rotations
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 1.2e-5f,	RX));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MNX, dof(true, true, true, false, true, true), 1000, -1.2e-5f,RX, "singleBondFixedFree"));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 3e-6f,	RY));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MNY, dof(true, true, true, true, false, true), 1000, -3e-6f,	RY));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 3e-6f,	RZ));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_POS, cNNN, MNZ, dof(true, true, true, true, true, false), 1000, -3e-6f,	RZ));
	
	//in a few other orientations
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::X_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 1.2e-5f,	RX));
	EXPECT_GT(100,  test2Vox(true, CVX_Voxel::X_NEG, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 3e-6f,	RY)); //this one is actually really good compared to others...
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Y_POS, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 1.2e-5f,	RY));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Y_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 3e-6f,	RX));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Y_NEG, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 1.2e-5f,	RY));
	EXPECT_GT(100,  test2Vox(true, CVX_Voxel::Y_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 3e-6f,	RX));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Z_POS, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 1.2e-5f,	RZ));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Z_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 3e-6f,	RX));
	EXPECT_GT(200, test2Vox(true, CVX_Voxel::Z_NEG, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 1.2e-5f,	RZ));
	EXPECT_GT(100,  test2Vox(true, CVX_Voxel::Z_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 3e-6f,	RX));

	//apply moment, look at translation
	EXPECT_GT(10,  test2Vox(true, CVX_Voxel::X_POS, cNNN, MPX, dof(false, true, true, false, true, true), 1000, 0.0f,	TX));
	EXPECT_GT(10,  test2Vox(true, CVX_Voxel::X_POS, cNNN, MNX, dof(false, true, true, false, true, true), 1000, 0.0f,	TX));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, cNNN, MPY, dof(true, true, false, true, false, true), 1000, -6e-9f,	TZ));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, cNNN, MNY, dof(true, true, false, true, false, true), 1000, 6e-9f,	TZ));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, cNNN, MPZ, dof(true, false, true, true, true, false), 1000, 6e-9f,	TY));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, cNNN, MNZ, dof(true, false, true, true, true, false), 1000, -6e-9f,	TY));

	//apply force, look at rotation
	EXPECT_GT(10,  test2Vox(true, CVX_Voxel::X_POS, FPX, cNNN, dof(false, true, true, false, true, true), 1000, 0.0f,	RX));
	EXPECT_GT(10,  test2Vox(true, CVX_Voxel::X_POS, FNX, cNNN, dof(false, true, true, false, true, true), 1000, 0.0f,	RX));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, FPY, cNNN, dof(true, false, true, true, true, false), 1000, 6e-3f,	RZ));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, FNY, cNNN, dof(true, false, true, true, true, false), 1000, -6e-3f,	RZ));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, FPZ, cNNN, dof(true, true, false, true, false, true), 1000, -6e-3f,	RY));
	EXPECT_GT(300, test2Vox(true, CVX_Voxel::X_POS, FNZ, cNNN, dof(true, true, false, true, false, true), 1000, 6e-3f,	RY));
}



TEST(CVoxelyze, singleBondFreeFree)
{
	//apply force, look at translations
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f,  TX, "singleBondFreeFree"));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FNX, cNNN, dof(false, true, true, true, true, true), 1000, -5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 5e-7f, TY));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FNY, cNNN, dof(true, false, true, true, true, true), 1000, -5e-7f, TY));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 5e-7f,  TZ));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_POS, FNZ, cNNN, dof(true, true, false, true, true, true), 1000, -5e-7f, TZ));

	//test an axial and transverse for each other (instead of 6 each)
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::X_NEG, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 5e-7f, TY));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Y_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Y_POS, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 5e-7f, TY));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Y_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Y_NEG, FPY, cNNN, dof(true, false, true, true, true, true), 1000, 5e-7f, TY));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Z_POS, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Z_POS, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 5e-7f, TZ));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Z_NEG, FPX, cNNN, dof(false, true, true, true, true, true), 1000, 5e-7f, TX));
	EXPECT_GT(100, test2Vox(false, CVX_Voxel::Z_NEG, FPZ, cNNN, dof(true, true, false, true, true, true), 1000, 5e-7f, TZ));

	//apply moment, look at rotations
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,  RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MNX, dof(true, true, true, false, true, true), 1000, -6e-6f, RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 6e-6f,  RY));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MNY, dof(true, true, true, true, false, true), 1000, -6e-6f, RY));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 6e-6f,  RZ));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_POS, cNNN, MNZ, dof(true, true, true, true, true, false), 1000, -6e-6f, RZ));
	
	//in a few other orientations
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,	 RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::X_NEG, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 6e-6f,	 RY));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Y_POS, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 6e-6f,	 RY));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Y_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,	RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Y_NEG, cNNN, MPY, dof(true, true, true, true, false, true), 1000, 6e-6f,	RY));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Y_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,	RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Z_POS, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 6e-6f,	RZ));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Z_POS, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,	RX));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Z_NEG, cNNN, MPZ, dof(true, true, true, true, true, false), 1000, 6e-6f,	RZ));
	EXPECT_GT(150, test2Vox(false, CVX_Voxel::Z_NEG, cNNN, MPX, dof(true, true, true, false, true, true), 1000, 6e-6f,	RX));

}

TEST(CVoxelyze, resetTime){
	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	
	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);

	pV1->external()->setFixedAll();
	pV2->external()->setForce(1e-3f, 0, 0); //axial tension

	for (int i=0; i<100; i++) Sim.doTimeStep();
	Sim.resetTime();
	EXPECT_FLOAT_EQ(1e-3f, (float)pV2->position().x);
}

TEST(CVoxelyze, internalDamping)
{
	std::ofstream file("output-CVoxelyze-internalDamping.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);

	//surround with fixed voxels
	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	pV1->external()->setForce(1e-6f, 1e-6f, 1e-6f);

	for (int i=0; i<6; i++){
		Vec3D<> off = toOffset((CVX_Voxel::linkDirection)i);
		CVX_Voxel* pV = Sim.setVoxel(pMat1, (int)off.x, (int)off.y, (int)off.z);
		pV->external()->setFixedAll();
	}

	float ts = Sim.recommendedTimeStep();
	for (int k=0; k<100; k++){ //22 experimentally determined
		Sim.doTimeStep(ts);
		file << pV1->position().y << "\n";
	}
	EXPECT_FLOAT_EQ(1e-9f/6, (float)pV1->position().y);

}

TEST(CVoxelyze, globalDamping)
{
	std::ofstream file("output-CVoxelyze-globalDamping.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setGlobalDamping(1.0);
	pMat1->setInternalDamping(0);

	//surround with fixed voxels
	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	pV1->external()->setForce(1e-6f, 1e-6f, 1e-6f);

	for (int i=0; i<6; i++){
		Vec3D<> off = toOffset((CVX_Voxel::linkDirection)i);
		CVX_Voxel* pV = Sim.setVoxel(pMat1, (int)off.x, (int)off.y, (int)off.z);
		pV->external()->setFixedAll();
	}

	float ts = Sim.recommendedTimeStep();
	for (int k=0; k<100; k++){ //22 experimentally determined
		Sim.doTimeStep(ts);
		//file << pV1->position().x << "\t" << pV1->position().y << "\t" << pV1->position().z << "\n";
	}
	EXPECT_FLOAT_EQ(1e-9f/6, (float)pV1->position().x);
	EXPECT_FLOAT_EQ(1e-9f/6, (float)pV1->position().y);
	EXPECT_FLOAT_EQ(1e-9f/6, (float)pV1->position().z);

	//2 voxel cantilever
	CVoxelyze Sim2(0.001);
	CVX_Material* pMat2 = Sim2.addMaterial(1e6, 1e3);
	pMat2->setGlobalDamping(0.25f);
	pMat2->setInternalDamping(0);

	CVX_Voxel* pV21 = Sim2.setVoxel(pMat2,0,0,0);
	pV21->external()->setFixedAll();
	CVX_Voxel* pV22 = Sim2.setVoxel(pMat2,1,0,0);
	pV22->external()->setForce(1e-6f, 1e-6f, 1e-6f);

	ts = Sim2.recommendedTimeStep();
	for (int k=0; k<300; k++){ //22 experimentally determined
		Sim2.doTimeStep(ts);
		//file << pV22->position().y << "\n";

	}
	EXPECT_FLOAT_EQ(4e-9f, (float)pV22->position().y);
}

TEST(CVoxelyze, combinedDamping)
{
	std::ofstream file("output-CVoxelyze-combinedDamping.txt");

	//check a block for stability
	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0f);

	for (int i=0; i<4; i++){
		for (int j=0; j<3; j++){
			for (int k=0; k<3; k++){
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				else if (i==3) pV->external()->setForce(0,0,1e-6f);
			}
		}
	}

	for (int i=0; i<2; i++){ //rough... just checking for instability
		pMat1->setGlobalDamping(0.05f+0.05f*i);
		float ts = Sim.recommendedTimeStep();
		Sim.resetTime();
		for (int k=0; k<1000; k++){
			Sim.doTimeStep(ts);
			if (i==0) file << Sim.voxel(3,0,0)->position().z << "\n";
		}
		EXPECT_NEAR(1.742e-8, Sim.voxel(3,0,0)->position().z, 1e-10); //was 1.74
	}

	////max damping (damping factors of 1 and 1)
	//pMat1->setGlobalDamping(1.0);
	//pMat1->setInternalDamping(1.0f);
	//float ts = Sim.recommendedTimeStep();
	//Sim.resetTime();
	//for (int k=0; k<1500; k++){
	//	Sim.doTimeStep(ts);
	//	//file << Sim.voxel(3,0,0)->position().z << "\n";
	//}
	//EXPECT_NEAR(1.742e-8, Sim.voxel(3,0,0)->position().z, 1e-10);

	file.close();

}


TEST(CVoxelyze, scale)
{
	std::ofstream file("output-CVoxelyze-scale.txt");

	double sz=0.001;
	CVoxelyze Sim(sz);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.2f);


	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	pV1->external()->setFixedAll();
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);
	
	for (int i=3; i<8; i++){
		float force = 1/pow(10.0f, i), result = 4/pow(10.0f, i+3);
		pV2->external()->setForce(force, force, force);
		float ts = Sim.recommendedTimeStep();
		Sim.resetTime();
		for (int k=0; k<240; k++){
			Sim.doTimeStep(ts);
			file << pV2->position().y << "\n";

		}
		EXPECT_NEAR(result, (float)pV2->position().y, result/1000);
	}

	file.close();

}


TEST(CVoxelyze, freqency){
	std::ofstream file("output-CVoxelyze-frequency.txt");

	CVoxelyze Sim(0.001f);
	CVX_Material* pMat1 = Sim.addMaterial(1e6f, 1e3f);
	pMat1->setInternalDamping(0);

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);

	//Axial natural frequency
	pV1->external()->setFixedAll();
	pV2->external()->setFixed(false, true, true, true, true, true);
	pV2->external()->setForce(1e-3f, 0, 0); //axial tension

	float ts = Sim.recommendedTimeStep()/10;
	std::vector<double> data;
	file << ts << "\n";
	for (int i=0; i<1000; i++){
		Sim.doTimeStep(ts);
		file << pV2->position().x-0.001001 << "\n";
		data.push_back(pV2->position().x-0.001001);
	}
	double expectedPeriod = 2*3.1415926/sqrt(1e9);
	double actualPeriod = calcPeriod(ts, data);
	EXPECT_NEAR(expectedPeriod, actualPeriod, expectedPeriod/1000); //0.01% error ok due to data discretization

	file.close();
}

TEST(CVoxelyze, largeDeformationDamping)
{
	std::ofstream file("output-CVoxelyze-largeDeformationDamping.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.2f);

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);

	float ts = Sim.recommendedTimeStep();
	file << ts << "\n"; //output the timestep
	pV1->external()->setFixedAll();

	//apply force, look at translations
	CVX_Link* pL = Sim.link(0, 0, 0, CVX_Voxel::X_POS);
	pV2->external()->setForce(-0.2f, 0.0f, 0.2f);
	for (int k=0; k<200; k++){
		Sim.doTimeStep(ts);
		file << pV2->position().z << "\t" <<  pL->isSmallAngle() << "\n";
	}
	
	EXPECT_NEAR(9.5587e-4, pV2->position().z, 1e-7); 


	file.close();
}

TEST(CVoxelyze, largeDeformation)
{
	std::ofstream file("output-CVoxelyze-largeDeformation.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.2f);


	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);

	float ts = Sim.recommendedTimeStep();
	file << ts << "\n"; //output the timestep
	pV1->external()->setFixedAll();
	float lastValue = 0, lastDerivative = 0, maxDerivative2 = 0;

	//apply force, look at translations
	for (int i=0; i<100; i++){ //several forces.
		pV2->external()->setForce(0, 0, i*2e-3f);
		CVX_Link* pL = Sim.link(0, 0, 0, CVX_Voxel::X_POS);
		for (int k=0; k<100; k++){
			Sim.doTimeStep(ts);
			if (i==20) file << pV2->position().z << "\n";
		}

		if (i==0) EXPECT_TRUE(pL->isSmallAngle());
		if (i==20) EXPECT_FALSE(pL->isSmallAngle());

		//double derivative to watch for jumps between linear and non-linear
		float thisValue = (float)pV2->position().z, thisDerivative=0, thisDerivative2=0;
		if (i>0) {
			thisDerivative = (thisValue-lastValue)/ts;
			if (i>1){
				thisDerivative2 = (thisDerivative-lastDerivative)/ts;
				if (abs(thisDerivative2) > maxDerivative2) maxDerivative2 = abs(thisDerivative2);
			}
			lastDerivative = thisDerivative;
		}
		lastValue = thisValue;

		file << pV2->position().x <<"\t" << thisValue << "\t" << thisDerivative << "\t" << thisDerivative2 << "\t" << pL->isSmallAngle() <<"\n";
		ASSERT_TRUE(maxDerivative2 < 5000);
	}

	file.close();
}


TEST(CVoxelyze, doubleBondCantilever)
{
	std::ofstream file("output-CVoxelyze-doubleBondCantilever.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.1f);

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,1,0,0);
	CVX_Voxel* pV3 = Sim.setVoxel(pMat1,2,0,0);

	float ts = Sim.recommendedTimeStep();
	file << ts << "\n"; //output the timestep
	pV1->external()->setFixedAll();

	//apply force, look at translations
	for (int i=0; i<3; i++){ //3 translations
		pV3->external()->setFixed(!(i==0), !(i==1), !(i==2), true, !(i==2), !(i==1));
		for (int j=0; j<2; j++) { //positive and negative forcing
			float sign = (j==0)?1.0f:-1.0f;
			Vec3D<float> forceToAdd(0,0,0);
			forceToAdd[i%3] = sign*5e-6f;
			pV3->external()->setForce(forceToAdd);
			int convergeCount = 0; //consecutive times we've got the value we want
			float expectedValue = (i==0 ? sign*1e-8f : sign*1.6e-7f);

			Sim.resetTime();
			int k;
			for (k=0; k<3000; k++){
				Sim.doTimeStep(ts);
				float data = (float)((pV3->position()-Vec3D<double>(0.002,0,0))[i%3]);
				if (i==1 && j==0)
					file << data << "\n";

				if (abs(data - expectedValue) < abs(expectedValue)*1e-4 ) convergeCount++; //equals to catch zero expected values
				else convergeCount=0;
				if (convergeCount == 10) break; //X consecutive desired values
			}
			if (i==0) EXPECT_GT(200, k);
			else EXPECT_GT(500, k);

		//	if (i==0) EXPECT_FLOAT_EQ(sign*1e-8f, (float)(pV3->position()-Vec3D<>(0.002,0,0))[i%3]);
		//	else EXPECT_FLOAT_EQ(sign*1.6e-7f, (float)(pV3->position()-Vec3D<>(0.002,0,0))[i%3]); //FL^3/3EI = 0.000005*0.002^3/(3*1000000*0.001^4/12) = 1.6e-7
		}
	}

	
	file.close();
}

TEST(CVoxelyze, impulse) //giant large-angle impulse to check for instabilities
{
	
	std::ofstream file("output-CVoxelyze-impulse.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0);
	pMat1->setGlobalDamping(0.05f);

	for (int i=0; i<4; i++){ 
		for (int j=0; j<2; j++){ 
			CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,0);
			if (i==0) pV->external()->setFixedAll();
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int k=0; k<1000; k++){
		CVX_Voxel* pV = Sim.voxel(3,0,0);
		if (k==10) pV->external()->setForce(0,0,100);
		else if (k==11) pV->external()->setForce(0,0,0);

		Sim.doTimeStep(ts);
		float data = (float)pV->position().z;
		file << data << "\n";
	}

	EXPECT_NEAR(0.0f, (float)(Sim.voxel(3,0,0)->position().z), 1e-5);


	
	file.close();
}



TEST(CVoxelyze, multiSimple)
{
	std::ofstream file("output-CVoxelyze-multiSimple.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3); //k=1000
	CVX_Material* pMat2 = Sim.addMaterial(1e9, 1e3); //k=1000000
	pMat1->setGlobalDamping(0.03f);
	pMat2->setGlobalDamping(0.03f);
	pMat1->setInternalDamping(0.01f);
	pMat2->setInternalDamping(0.01f);

	for (int i=0; i<2; i++){
		CVX_Voxel* pV1 = Sim.setVoxel(pMat1,2*i,0,0);
		CVX_Voxel* pV2 = Sim.setVoxel(pMat2,2*i+1,0,0);
		if (i==0) pV1->external()->setFixedAll();
		if (i==1) pV2->external()->setForce(1e-3f, 0, 0);
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<800; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(3,0,0)->position().x-0.003 << "\n";
	}

	EXPECT_FLOAT_EQ(1.5015e-6f, (float)(Sim.voxel(3,0,0)->position().x-0.003));
	//each link stiffness = 2 / (1/1000+1/1000000) = 2000000/1001 (=999.000999000999...)
	//3 in series = 2000000/3003
	//f=kx -> 1e-3=2000000/3003*x
	//x=1.5015e-6

	file.close();
}

TEST(CVoxelyze, multiSimple2) //2-thick layers (three effective materials)
{
	std::ofstream file("output-CVoxelyze-multiSimple2.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3); //k=1000
	CVX_Material* pMat2 = Sim.addMaterial(1e9, 1e3); //k=1000000
	pMat1->setGlobalDamping(0.01f);
	pMat2->setGlobalDamping(0.01f);
	pMat1->setInternalDamping(1.0f);
	pMat2->setInternalDamping(1.0f);


	for (int i=0; i<8; i++){
		CVX_Voxel* pV;
		if ((i/2)%2==0) pV = Sim.setVoxel(pMat1,i,0,0);
		else pV = Sim.setVoxel(pMat2,i,0,0);

		if (i==0) pV->external()->setFixedAll();
		if (i==7) pV->external()->setForce(1e-3f, 0, 0);
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<10000; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(7,0,0)->position().x-0.007 << "\n";
	}

	EXPECT_NEAR(3.5035e-6f, (float)(Sim.voxel(7,0,0)->position().x-0.007), 1e-10);
	//each intermediate link stiffness = 2 / (1/1000+1/1000000) = 2000000/1001 (=1998.001998001998...)
	//2 hard, 2, soft, 3 intermediate: 1 / (2*1/1000 + 3*1001/2000000 + 2*1/1000000 ) = 2000000 / (4000+3003+4) = 2000000/7007

	//f=kx -> 1e-3=2000000/7007*x
	//x=3.5035e-6

	file.close();
}

//poisson's ratio
TEST(CVoxelyze, poissonsSmall) //1-wide
{
	std::ofstream file("output-CVoxelyze-poissonsSmall.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setPoissonsRatio(0.3f);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.3f);


	for (int i=0; i<3; i++){ 
		CVX_Voxel* pV = Sim.setVoxel(pMat1,i,0,0);
		if (i==0) pV->external()->setFixedAll();
		if (i==2) pV->external()->setForce(1e-3f, 0, 0);
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<200; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(2,0,0)->position().x-0.002 << "\n";
	}

	EXPECT_NEAR(1.70524e-6, (float)(Sim.voxel(2,0,0)->position().x-0.002), 1e-9);

	file.close();
}

TEST(CVoxelyze, poissonsLarge) //2x2x9
{
	std::ofstream file("output-CVoxelyze-poissonsLarge.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	float mu = 0.3f;
	pMat1->setPoissonsRatio(mu);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.2f);


	for (int i=0; i<9; i++){ 
		for (int j=0; j<2; j++){ 
			for (int k=0; k<2; k++){ 
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				if (i==8) pV->external()->setDisplacementAll(Vec3D<>(1e-3f, 0, 0));

			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<300; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(4,0,0)->position().x-0.004 << "\n";
	}

	EXPECT_NEAR(5e-4, (float)(Sim.voxel(4,0,0)->position().x-0.004), 1e-7);

//	Vec3D<float> curSize = 2*Sim.voxel(4,0,0)->cornerPosition(CVX_Voxel::PPP); //to avoid edges
	Vec3D<float> curSize = 2*Sim.voxel(4,0,0)->cornerOffset(CVX_Voxel::PPP); //to avoid edges
	Vec3D<float> curStrain = curSize - Vec3D<float>(0.001f, 0.001f, 0.001f);
	curStrain /= 0.001f; 
	EXPECT_NEAR(1.306e-1, curStrain.x, 1e-3);
	EXPECT_NEAR(-4.048e-2, curStrain.y, 1e-5);
	EXPECT_NEAR(-4.048e-2, curStrain.z, 1e-5);

	float eHat = pMat1->youngsModulus()/((1-2*mu)*(1+mu));
	float sigmaX = eHat*((1-mu)*curStrain.x + mu*(curStrain.y+curStrain.z));

	Vec3D<float> curSize2 = Sim.voxel(4,0,0)->size(); //to include full voxels
	float fX = sigmaX*4*curSize2.y*curSize2.z;

	float totalForce = 0;
	for (int j=0; j<2; j++){ 
		for (int k=0; k<2; k++){ 
			totalForce += Sim.voxel(8,j,k)->externalForce().x;
		}
	}

	EXPECT_NEAR(totalForce, fX, 0.01); //sanity check
	file.close();
}

TEST(CVoxelyze, poissonsHigh) //5x3x3
{
	std::ofstream file("output-CVoxelyze-poissonsHigh.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	float mu = 0.495f;
	pMat1->setPoissonsRatio(mu);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(2.0f);


	for (int i=0; i<5; i++){ 
		for (int j=0; j<3; j++){ 
			for (int k=0; k<3; k++){ 
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				if (i==4) pV->external()->setDisplacementAll(Vec3D<>(1e-5f, 0, 0));

			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<300; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(2,1,1)->position().x-0.002 << "\n";
	}

	EXPECT_NEAR(5e-6, (float)(Sim.voxel(2,1,1)->position().x-0.002), 1e-9);

	file.close();
}


TEST(CVoxelyze, poissonsMixed) //7x3x3
{
	std::ofstream file("output-CVoxelyze-poissonsMixed.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6f, 1e3f);
	pMat1->setPoissonsRatio(0.3f);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.3f);
	CVX_Material* pMat2 = Sim.addMaterial(1e6f, 1e3f);
	pMat2->setPoissonsRatio(0.0f);
	pMat2->setInternalDamping(1.0f);
	pMat2->setGlobalDamping(0.3f);


	for (int i=0; i<7; i++){ 
		for (int j=0; j<3; j++){ 
			for (int k=0; k<3; k++){ 
				CVX_Voxel* pV;
				if (i>1 && i<6)	pV = Sim.setVoxel(pMat2,i,j,k);
				else pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				if (i==6) pV->external()->setDisplacementAll(Vec3D<>(1e-3f, 0, 0));
			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<300; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(3,1,1)->position().x-0.003 << "\n";
	}

	EXPECT_NEAR(5e-4, (float)(Sim.voxel(3,1,1)->position().x-0.003), 5e-6); //this one's pretty loose... likely due to asymettric bonds?

	file.close();
}


TEST(CVoxelyze, deformableMaterial) //5x3x3
{
	std::ofstream file("output-CVoxelyze-deformableMaterial.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setModelBilinear(1e6, 5e5, 1e5);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.2f);


	for (int i=0; i<5; i++){ 
		for (int j=0; j<3; j++){ 
			for (int k=0; k<3; k++){ 
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				if (i==4) pV->external()->setForce(Vec3D<float>(0.2f, 0.0f, 0.0f));

			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<400; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(4,1,1)->position().x-0.004 << "\n";
	}

	//erase the forces
	for (int j=0; j<3; j++){ 
		for (int k=0; k<3; k++){ 
			Sim.voxel(4,j,k)->external()->setForce(Vec3D<float>(0.0f, 0.0f, 0.0f));
		}
	}

	//let it relax
	for (int i=0; i<250; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(4,1,1)->position().x-0.004 << "\n";
	}

	EXPECT_NEAR(4e-4, (float)(Sim.voxel(4,1,1)->position().x-0.004), 1e-7);

	file.close();
}

TEST(CVoxelyze, deformableMaterialPossions) //5x3x3
{
	std::ofstream file("output-CVoxelyze-deformableMaterialPoissons.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setModelBilinear(1e6, 5e5, 1e5);
	pMat1->setPoissonsRatio(0.3f);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.2f);


	for (int i=0; i<5; i++){ 
		for (int j=0; j<3; j++){ 
			for (int k=0; k<3; k++){ 
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				//if (i==4) pV->setExternalForce(Vec3D<float>(0.2f, 0.0f, 0.0f));

			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int i=0; i<300; i++){
		if (i<200){
			for (int j=0; j<3; j++){ 
				for (int k=0; k<3; k++){ 
					Sim.voxel(4,j,k)->external()->setDisplacementAll(Vec3D<>(i*7.5e-6f, 0.0f, 0.0f));
				}
			}
		}
		Sim.doTimeStep(ts);
		file << Sim.voxel(4,1,1)->position().x-0.004 << "\t" << Sim.voxel(2,1,1)->position().x-0.002 << "\n";
	}


	//erase the forces
	for (int j=0; j<3; j++){ 
		for (int k=0; k<3; k++){ 
			Sim.voxel(4,j,k)->external()->setFixedAll(false);
		}
	}

	//let it relax
	for (int i=0; i<400; i++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(4,1,1)->position().x-0.004 << "\t" << Sim.voxel(2,1,1)->position().x-0.002 << "\n";
	}

	EXPECT_NEAR(7.16e-4, (float)(Sim.voxel(4,1,1)->position().x-0.004), 1e-7);

	file.close();
}

TEST(CVoxelyze, replaceMaterial)
{
	std::ofstream file("output-CVoxelyze-replaceMaterial.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.08f);
	CVX_Material* pMat2 = Sim.addMaterial(1e7, 1e3);
	pMat2->setInternalDamping(1.0f);
	pMat2->setGlobalDamping(0.08f);

	for (int i=0; i<5; i++){
		for (int j=0; j<2; j++){
			for (int k=0; k<2; k++){
				CVX_Voxel* pV = Sim.setVoxel(pMat1,i,j,k);
				if (i==0) pV->external()->setFixedAll();
				else if (i==4) pV->external()->setForce(0,0,1e-6f);
			}
		}
	}

	float ts = Sim.recommendedTimeStep();
	for (int l=0; l<2000; l++){
		Sim.doTimeStep(ts);
		if (l==150){ //switch it up!
			for (int i=0; i<5; i++){
				for (int j=0; j<2; j++){
					for (int k=0; k<2; k++){
						CVX_Voxel* pV = Sim.voxel(i,j,k);
						if (i%2 == 1)
							Sim.setVoxel(pMat2, i, j, k);
					}
				}
			}
			ts = Sim.recommendedTimeStep();
		}
		file << Sim.voxel(4,0,0)->position().z << "\n";
	}
	EXPECT_NEAR(3.8167e-8, Sim.voxel(4,0,0)->position().z, 1e-10);
}


TEST(CVoxelyze, temperature)
{
	std::ofstream file("output-CVoxelyze-temperature.txt");

	CVoxelyze Sim(0.001);
	CVX_Material* pMat1 = Sim.addMaterial(1e6, 1e3);
	pMat1->setInternalDamping(1.0f);
	pMat1->setGlobalDamping(0.15f);
	pMat1->setCte(0.01f);
	CVX_Material* pMat2 = Sim.addMaterial(1e7f, 1e3f);
	pMat2->setInternalDamping(1.0f);
	pMat2->setGlobalDamping(0.15f);


	for (int i=0; i<3; i++){
		CVX_Voxel* pV1 = Sim.setVoxel(pMat1,i,0,0);
		CVX_Voxel* pV2 = Sim.setVoxel(pMat2,i,0,1);

		if (i==0){
			pV1->external()->setFixedAll();
			pV2->external()->setFixedAll();		
		}
	}

	//direction 1
	Sim.setAmbientTemperature(5, true);

	float ts = Sim.recommendedTimeStep();
	for (int l=0; l<500; l++){
		Sim.doTimeStep(ts);
	//	file << Sim.voxel(2,0,0)->position().z << "\n";
	}
	EXPECT_NEAR(2.55e-5, Sim.voxel(2,0,0)->position().z, 1e-8);

	//direction 2
	Sim.resetTime();
	Sim.setAmbientTemperature(-5, true);

	for (int l=0; l<500; l++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(2,0,0)->position().z << "\n";
	}
	EXPECT_NEAR(-2.591e-5, Sim.voxel(2,0,0)->position().z, 1e-8);
}

TEST(CVoxelyze, staticFriction)
{
	std::ofstream file("output-CVoxelyze-staticFriction.txt");

	double vSize = 0.001;
	float density = 1e3f;
	CVoxelyze Sim(vSize);
	Sim.enableFloor(true);
	Sim.setGravity();
	CVX_Material* pMat1 = Sim.addMaterial(1e6, density);
	float normalForce = (float)(density*vSize*vSize*vSize*9.80665); //mg

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	
	pMat1->setStaticFriction(1.0f);
	pMat1->setKineticFriction(0.1f); //normal force = 1e3*0.001
	pMat1->setGlobalDamping(1.0f);

	float ts = Sim.recommendedTimeStep();

	//baseline
	//just below break of static friction
	for (int l=0; l<50; l++){
		file << Sim.voxel(0,0,0)->position().z << "\n";
		Sim.doTimeStep(ts); //let it settle down...
	}
	pV1->external()->setForce(0.9f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_EQ(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();

	//just above break of static friction
	for (int l=0; l<50; l++) Sim.doTimeStep(ts);
	pV1->external()->setForce(1.1f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_NE(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();

	//increase gravity
	Sim.setGravity(2.0); //2g gravity

	//just below break of static friction
	for (int l=0; l<50; l++) Sim.doTimeStep(ts); //let it settle down...
	pV1->external()->setForce(1.9f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_EQ(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();

	//just above break of static friction
	for (int l=0; l<50; l++) Sim.doTimeStep(ts);
	pV1->external()->setForce(2.1f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_NE(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();
	
	//increase static friction
	Sim.setGravity(); //1g gravity
	pMat1->setStaticFriction(2.0f);

	//just below break of static friction
	for (int l=0; l<50; l++) Sim.doTimeStep(ts); //let it settle down...
	pV1->external()->setForce(1.9f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_EQ(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();

	//just above break of static friction
	for (int l=0; l<50; l++) Sim.doTimeStep(ts);
	pV1->external()->setForce(2.1f*normalForce, 0.0f, 0.0f);
	for (int l=0; l<10; l++) Sim.doTimeStep(ts);
	EXPECT_NE(0.0, pV1->position().x);
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	Sim.resetTime();
}



TEST(CVoxelyze, kineticFriction)
{
	std::ofstream file("output-CVoxelyze-kineticFriction.txt");

	double vSize = 0.001;
	float density = 1e3f;
	CVoxelyze Sim(vSize);
	Sim.enableFloor(true);
	Sim.setGravity();
	CVX_Material* pMat1 = Sim.addMaterial(1e6, density);
	double mass = (density*vSize*vSize*vSize);
	float normalForce = (float)(mass*9.80665); //mg

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	
	pMat1->setStaticFriction(1.0f);
	pMat1->setKineticFriction(0.1f); //normal force = 1e3*0.001
	pMat1->setGlobalDamping(1.0f);

	float ts = Sim.recommendedTimeStep();

	//just above break of static friction
	for (int l=0; l<50; l++){
		Sim.doTimeStep(ts);
	//	file << Sim.voxel(0,0,0)->position().x << "\t" << Sim.voxel(0,0,0)->position().z << "\n";
	}
	pMat1->setGlobalDamping(0.0001f);
	float hForce = 2.0f*normalForce;
	pV1->external()->setForce(hForce, 0.0f, 0.0f);
	double lastPos = 0;
	double vel = 0;
	double energy = 0;
	for (int l=0; l<10; l++){
		Sim.doTimeStep(ts);
		double curPos = Sim.voxel(0,0,0)->position().x;
		vel = (curPos - lastPos)/ts;
//		energy += hForce*(curPos - lastPos); //F*ds
		float factor = l==9 ? 0.5f : 1.0f; //only add half the energy last time to make discrete energy calc work
		energy += factor*(hForce - pMat1->kineticFriction()*normalForce)*(curPos - lastPos); //F*ds
		file << curPos << "\t" << vel << "\t" << Sim.voxel(0,0,0)->position().z << "\n";
		lastPos = curPos;
	}
	ASSERT_NEAR(energy, 0.5*mass*vel*vel, 2e-16); //check energy (not necessary, but kinda fun
	pV1->external()->setForce(0.0f, 0.0f, 0.0f);
	for (int l=0; l<200; l++){
		Sim.doTimeStep(ts);
		double curPos = Sim.voxel(0,0,0)->position().x;
		file << curPos << "\t" << (curPos - lastPos)/ts << "\t" << Sim.voxel(0,0,0)->position().z << "\n";
		lastPos = curPos;
	}

}

TEST(CVoxelyze, collisions) //could be expanded significantly
{
	std::ofstream file("output-CVoxelyze-collisions.txt");

	double vSize = 0.001;
	float density = 1e6f;
	CVoxelyze Sim(vSize);
	Sim.enableFloor(true);
	Sim.setGravity();
	CVX_Material* pMat1 = Sim.addMaterial(1e6, density);
	pMat1->setGlobalDamping(0.0f);

	CVX_Voxel* pV1 = Sim.setVoxel(pMat1,0,0,0);
	pV1->external()->setFixedAll();

	CVX_Voxel* pV2 = Sim.setVoxel(pMat1,0,0,2);

	Sim.enableCollisions();

	float ts = Sim.recommendedTimeStep();

	//just above break of static friction
	for (int l=0; l<150; l++){
		Sim.doTimeStep(ts);
		file << Sim.voxel(0,0,2)->position().z << "\n";
	}

	EXPECT_GT(Sim.voxel(0,0,2)->position().z, 0.001);
}

//timestep calc with wide varying density and stiffness

