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
#include <algorithm>
#include <adolc/adolc.h>

//#define T_DEBUG 1

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

	// simulation variables
	matrix<double> state_0;					// initial pose
	matrix<double> state_d;					// desired pose
	std::vector<matrix<double>> uSequence;	// sequence of input torque vectors
	PxU32 numTimeSteps;						// number of time steps in simulation
	double deltaT;							// length of each time step
	PxVec3 root;							// position of root (static)
	double uThreshold;

	// Init functions (SpacetimeInit.cpp)
	void initPhysics(void);

	// Optimization functions (SpacetimeOptimization.cpp)
	void makeInitialGuess(void);
	double IterateOptimization(void);
	
	// State functions (SpacetimeState.cpp)
	void switchPause(void);
	void setState(matrix<double> stateVector);
	void restoreState(void);

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
	matrix<double> getState(void);

	// Kinematics functions (SpacetimeKinematics.cpp)
	matrix<double> buildJacobian(void);
	matrix<double> computeGVector(void);
	matrix<double> calculateAngularVelocity(void);
	matrix<double> calculateAngularPosition(void);
	matrix<double> computeInverseMassMatrix(matrix<double> G);
	matrix<double> computeCVector(matrix<double> G, matrix<double> M);

	// Optimization sequences
	std::vector<matrix<double>> GSequence;
	std::vector<matrix<double>> CSequence;
	std::vector<matrix<double>> MSequence;
	std::vector<matrix<double>> MInvSequence;
	std::vector<matrix<double>> stateSequence;
	std::vector<matrix<double>> costateSequence;

	// Numerical derivative functions (SpacetimeDerivatives.cpp)
	// Utilizes ADOL-C library
	matrix<double> compute_dLdx_numerical(PxU32 t);
	matrix<double> compute_dLdu_numerical(PxU32 t);
	matrix<double> compute_dfdx_numerical(PxU32 t);
	matrix<double> compute_dfdu_numerical(PxU32 t);
	matrix<double> compute_dG_dtheta1_numerical(PxU32 t);
	matrix<double> compute_dG_dtheta2_numerical(PxU32 t);
	matrix<double> compute_dG_dthetaDot1_numerical(PxU32 t);
	matrix<double> compute_dG_dthetaDot2_numerical(PxU32 t);
	matrix<double> compute_dC_dtheta1_numerical(PxU32 t);
	matrix<double> compute_dC_dtheta2_numerical(PxU32 t);
	matrix<double> compute_dC_dthetaDot1_numerical(PxU32 t);
	matrix<double> compute_dC_dthetaDot2_numerical(PxU32 t);
	matrix<double> compute_dMInv_dtheta1_numerical(PxU32 t);
	matrix<double> compute_dMInv_dtheta2_numerical(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot1_numerical(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot2_numerical(PxU32 t);

	// Analytical derivative functions (SpacetimeAnalytical.cpp)
	// Only valid as ADOL-C alternative for 2 joint system
	matrix<double> compute_dLdx_analytical(PxU32 t);
	matrix<double> compute_dLdu_analytical(PxU32 t);
	matrix<double> compute_dfdx_analytical(PxU32 t);
	matrix<double> compute_dfdu_analytical(PxU32 t);
	matrix<double> compute_dG_dtheta1_analytical(PxU32 t);
	matrix<double> compute_dG_dtheta2_analytical(PxU32 t);
	matrix<double> compute_dG_dthetaDot1_analytical(PxU32 t);
	matrix<double> compute_dG_dthetaDot2_analytical(PxU32 t);
	matrix<double> compute_dC_dtheta1_analytical(PxU32 t);
	matrix<double> compute_dC_dtheta2_analytical(PxU32 t);
	matrix<double> compute_dC_dthetaDot1_analytical(PxU32 t);
	matrix<double> compute_dC_dthetaDot2_analytical(PxU32 t);
	matrix<double> compute_dMInv_dtheta1_analytical(PxU32 t);
	matrix<double> compute_dMInv_dtheta2_analytical(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot1_analytical(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot2_analytical(PxU32 t);

	// Math3D functions
	PxVec3 QuaternionToEuler(PxQuat q);
	PxReal SSDmatrix(matrix<double> A, matrix<double> B);
	PxReal SSDvector(std::vector<matrix<double>> A, std::vector<matrix<double>> B);
};