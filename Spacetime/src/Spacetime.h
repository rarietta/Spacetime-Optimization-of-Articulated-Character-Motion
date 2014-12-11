//-------------------------------------------------------------------------------------------------//
// Spacetime.h
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#define PVD_HOST "127.0.0.1"

#include "PxPhysicsAPI.h"
#include "SvzAlgebra.h"
#include "SvzMatrix.h"
#include <iostream>
#include <assert.h>
#include <ctype.h>
#include <vector>
#include <adolc/adolc.h>
#include <adolc/adouble.h>

using namespace std;
using namespace physx;

class Spacetime
{
public:

	// for accessing X, Y, and Z components
	#define DOF 3
	enum {X, Y, Z};
	
	// constructor
	Spacetime(void);
	Spacetime(matrix<PxReal> startPose, matrix<PxReal> endPose, PxU32 numTimeSteps);
	matrix<PxReal> state_0;
	matrix<PxReal> state_d;
	PxU32 numTimeSteps;
	PxReal deltaT;
	
	// Init functions (SpacetimeInit.cpp)
	void initPhysics(void);

	// Optimization functions (SpacetimeOptimization.cpp)
	matrix<PxReal> Optimize(void);
	
	// State functions (SpacetimeState.cpp)
	void switchPause(void);

	// Kinematics functions (SpacetimeKinematics.cpp)
	void stepPhysics(void);

	// Cleanup functions (SpacetimeCleanup.cpp)
	void cleanupPhysics(void);

private:
	
	// is the program running?
	bool pause;

	// list of actor components
	enum {BASE, LEG1, LEG2, LEG3, HEAD};

	// Required physX runtime variable initializations
	PxScene* gScene;
	PxPhysics* gPhysics;
	PxMaterial*	gMaterial;
	PxFoundation* gFoundation;
	PxDefaultCpuDispatcher*	gDispatcher;
	PxVisualDebuggerConnection*	gConnection;
	PxDefaultAllocator gAllocator;
	PxDefaultErrorCallback gErrorCallback;

	// list of actors in character system
	std::vector<PxRigidStatic*> static_actors;
	std::vector<PxRigidDynamic*> dynamic_actors;

	// list of joints and their positions relative to attached bodies	
	std::vector<PxJoint*> joints;
	std::vector<PxVec3> joint_local_positions;

	// vectors for saving/restoring system
	std::vector<PxVec3> linearVelocityVector;
	std::vector<PxVec3> angularVelocityVector;
	std::vector<PxTransform> globalPoseVector;

	// Init functions (SpacetimeInit.cpp)
	void addStaticActors(void);
	void addDynamicActors(void);
	void addJoints(void);

	// State functions (SpacetimeState.cpp)
	void saveState(void);
	void restoreState(void);
	void setState(matrix<PxReal> stateVector);

	// Kinematics functions (SpacetimeKinematics.cpp)
	void applyTorqueVector(matrix<PxReal> T);
	matrix<PxReal> buildJacobian(void);
	matrix<PxReal> buildForceVector(void);
	matrix<PxReal> calculateAngularVelocity(void);
	matrix<PxReal> calculateAngularPosition(void);
	matrix<PxReal> computeInverseMassMatrix(matrix<PxReal> T);
	matrix<PxReal> computeCVector(matrix<PxReal> T, matrix<PxReal> M);
	matrix<PxReal> compute_dfdx(PxU32 t);
};