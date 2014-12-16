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

#define T_DEBUG 1

using namespace std;
using namespace physx;

enum {G_TAG, C_TAG, M_TAG, MINV_TAG};

class Spacetime
{
public:

	// for accessing X, Y, and Z components
	#define DOF 1
	enum {X, Y, Z};
	
	// constructor
	Spacetime(void);
	Spacetime(matrix<double> startPose, matrix<double> endPose, PxU32 numTimeSteps);
	matrix<double> state_0;
	matrix<double> state_d;
	std::vector<matrix<double>> uSequence;
	PxU32 numTimeSteps;
	double deltaT;
	PxVec3 root;

	// Init functions (SpacetimeInit.cpp)
	void initPhysics(void);

	// Optimization functions (SpacetimeOptimization.cpp)
	void makeInitialGuess(void);
	matrix<double> Optimize(void);
	
	// State functions (SpacetimeState.cpp)
	void switchPause(void);
	void setState(matrix<double> stateVector);

	// Kinematics functions (SpacetimeKinematics.cpp)
	void stepPhysics(void);
	void applyTorqueVector(matrix<double> T);

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
	matrix<double> getState(void);

	// Kinematics functions (SpacetimeKinematics.cpp)
	void debug(void);
	matrix<double> buildJacobian(void);
	matrix<double> computeGVector(void);
	matrix<double> calculateAngularVelocity(void);
	matrix<double> calculateAngularPosition(void);
	matrix<double> computeInverseMassMatrix(matrix<double> T);
	matrix<double> computeCVector(matrix<double> T, matrix<double> M);

	// Partial derivative functions (SpacetimeDerivatives.cpp)
	matrix<double> compute_dLdx(PxU32 t);
	matrix<double> compute_dLdu(PxU32 t);
	matrix<double> compute_dfdx(PxU32 t);
	matrix<double> compute_dfdu(PxU32 t);
};