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

using namespace std;
using namespace physx;

#define R3 3
enum {G_TAG, C_TAG, M_TAG, MINV_TAG};

class Spacetime {
public:

	// for accessing X, Y, and Z components
	#define DOF 1
	enum {X, Y, Z};

	// constructor
	Spacetime(void);
	Spacetime(matrix<double> startPose, matrix<double> endPose, PxU32 numTimeSteps);

	// simulation variables
	bool ANALYTIC;
	matrix<double> state_0;					// initial pose
	matrix<double> state_d;					// desired pose
	std::vector<matrix<double>> uSequence;	// sequence of input torque vectors
	PxU32 numTimeSteps;						// number of time steps in simulation
	double deltaT;							// length of each time step
	PxVec3 root;							// position of root (static)
	double uThreshold;						// threshold on difference btwn successive input sequences before convergence

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

	// global variables for use in analytic derivatives
	double g, m1, m2, lc1, lc2, l1, l2;
	matrix<double> state;
	double theta1, theta2;
	double thetaDot1, thetaDot2;
	
	// Init functions (SpacetimeInit.cpp)
	void initPhysics(void);
	void addStaticActors(void);
	void addDynamicActors(void);
	void addJoints(void);

	// Cleanup functions (SpacetimeCleanup.cpp)
	void cleanupPhysics(void);

	// Optimization functions (SpacetimeOptimization*.cpp)
	void makeInitialGuess_analytic(void);
	void makeInitialGuess_discrete(void);
	void makeInitialGuess_numeric(void);
	double IterateOptimization_analytic(void);
	double IterateOptimization_discrete(void);
	double IterateOptimization_numeric(void);
	
	// State functions (SpacetimeState.cpp)
	void setState(matrix<double> stateVector);
	void restoreState(void);
	void saveState(void);
	matrix<double> getState(void);

	// Numeric dynamics functions (SpacetimeDynamics_Numeric.cpp)
	// Utilizes ADOL-C library
	matrix<double> computeG_numeric(void);
	matrix<double> computeC_numeric(matrix<double> G, matrix<double> M);
	matrix<double> computeMInv_numeric(matrix<double> G);
	void stepPhysics_numeric(matrix<double> u);
	void applyTorqueVector(matrix<double> T);
	matrix<double> buildJacobian(void);
	matrix<double> calculateAngularVelocity(void);
	matrix<double> calculateAngularPosition(void);
	
	// Analytics dynamics functions (SpacetimeDynamics_Analytic.cpp)
	// Only valid as ADOL-C alternative for 2 joint system
	matrix<double> computeG_analytic(void);
	matrix<double> computeC_analytic(void);
	matrix<double> computeM_analytic(void);
	void stepPhysics_analytic(matrix<double> u);
	void reversePhysics_analytic(matrix<double> u);
	
	// Discrete dynamics functions (SpacetimeDynamics_Discrete.cpp)
	// Only valid as ADOL-C alternative for 2 joint system
	matrix<double> computeG_discrete(void);
	matrix<double> computeC_discrete(void);
	matrix<double> computeM_discrete(void);
	void stepPhysics_discrete(matrix<double> u);
	matrix<double> F(matrix<double> x, matrix<double>);

	// Optimization sequences
	std::vector<matrix<double>> GSequence;
	std::vector<matrix<double>> CSequence;
	std::vector<matrix<double>> MSequence;
	std::vector<matrix<double>> MInvSequence;
	std::vector<matrix<double>> stateSequence;
	std::vector<matrix<double>> costateSequence;
	std::vector<double> valueSequence;
	std::vector<matrix<double>> kSequence;
	std::vector<matrix<double>> KSequence;

	// Numeric derivative functions (SpacetimeDerivatives_Numeric.cpp)
	// Should utilize ADOL-C library
	matrix<double> compute_dLdx_numeric(PxU32 t);
	matrix<double> compute_dLdu_numeric(PxU32 t);
	matrix<double> compute_dfdx_numeric(PxU32 t);
	matrix<double> compute_dfdu_numeric(PxU32 t);
	matrix<double> compute_dG_dtheta1_numeric(PxU32 t);
	matrix<double> compute_dG_dtheta2_numeric(PxU32 t);
	matrix<double> compute_dG_dthetaDot1_numeric(PxU32 t);
	matrix<double> compute_dG_dthetaDot2_numeric(PxU32 t);
	matrix<double> compute_dC_dtheta1_numeric(PxU32 t);
	matrix<double> compute_dC_dtheta2_numeric(PxU32 t);
	matrix<double> compute_dC_dthetaDot1_numeric(PxU32 t);
	matrix<double> compute_dC_dthetaDot2_numeric(PxU32 t);
	matrix<double> compute_dMInv_dtheta1_numeric(PxU32 t);
	matrix<double> compute_dMInv_dtheta2_numeric(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot1_numeric(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot2_numeric(PxU32 t);

	// Analytic derivative functions (SpacetimeDerivatives_Analytic.cpp)
	// Only valid as ADOL-C alternative for 2 joint system
	matrix<double> compute_dfdu_analytic(void);
	matrix<double> compute_dLdx_analytic(PxU32 t);
	matrix<double> compute_dLdu_analytic(PxU32 t);
	matrix<double> compute_dfdx_analytic(PxU32 t);
	matrix<double> compute_dfdu_analytic(PxU32 t);
	matrix<double> compute_dG_dtheta1_analytic(PxU32 t);
	matrix<double> compute_dG_dtheta2_analytic(PxU32 t);
	matrix<double> compute_dG_dthetaDot1_analytic(PxU32 t);
	matrix<double> compute_dG_dthetaDot2_analytic(PxU32 t);
	matrix<double> compute_dC_dtheta1_analytic(PxU32 t);
	matrix<double> compute_dC_dtheta2_analytic(PxU32 t);
	matrix<double> compute_dC_dthetaDot1_analytic(PxU32 t);
	matrix<double> compute_dC_dthetaDot2_analytic(PxU32 t);
	matrix<double> compute_dMInv_dtheta1_analytic(PxU32 t);
	matrix<double> compute_dMInv_dtheta2_analytic(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot1_analytic(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot2_analytic(PxU32 t);
	
	// Dsicrete derivative functions (SpacetimeDerivatives_Discrete.cpp)
	// Only valid as ADOL-C alternative for 2 joint system
	matrix<double> compute_Lu(PxU32 t);
	matrix<double> compute_Lx(PxU32 t);
	matrix<double> compute_Fx(PxU32 t);
	matrix<double> compute_Fu(PxU32 t);
	matrix<double> compute_Luu(PxU32 t);
	matrix<double> compute_Lxx(PxU32 t);
	matrix<double> compute_Lux(PxU32 t);
	matrix<double> compute_Lxu(PxU32 t);
	std::vector<matrix<double>> compute_Fxx(PxU32 t);
	std::vector<matrix<double>> compute_Fxu(PxU32 t);
	std::vector<matrix<double>> compute_Fux(PxU32 t);
	std::vector<matrix<double>> compute_Fuu(PxU32 t);
	matrix<double> compute_dG_dtheta1(PxU32 t);
	matrix<double> compute_dG_dtheta2(PxU32 t);
	matrix<double> compute_dG_dthetaDot1(PxU32 t);
	matrix<double> compute_dG_dthetaDot2(PxU32 t);
	std::vector<matrix<double>> compute_dG_dX_dtheta1(PxU32 t);
	std::vector<matrix<double>> compute_dG_dX_dtheta2(PxU32 t);
	std::vector<matrix<double>> compute_dG_dX_dthetaDot1(PxU32 t);
	std::vector<matrix<double>> compute_dG_dX_dthetaDot2(PxU32 t);
	matrix<double> compute_dC_dtheta1(PxU32 t);
	matrix<double> compute_dC_dtheta2(PxU32 t);
	matrix<double> compute_dC_dthetaDot1(PxU32 t);
	matrix<double> compute_dC_dthetaDot2(PxU32 t);
	std::vector<matrix<double>> compute_dC_dX_dtheta1(PxU32 t);
	std::vector<matrix<double>> compute_dC_dX_dtheta2(PxU32 t);
	std::vector<matrix<double>> compute_dC_dX_dthetaDot1(PxU32 t);
	std::vector<matrix<double>> compute_dC_dX_dthetaDot2(PxU32 t);
	matrix<double> compute_dMInv_dtheta1(PxU32 t);
	matrix<double> compute_dMInv_dtheta2(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot1(PxU32 t);
	matrix<double> compute_dMInv_dthetaDot2(PxU32 t);
	std::vector<matrix<double>> compute_dMInv_dX_dtheta1(PxU32 t);
	std::vector<matrix<double>> compute_dMInv_dX_dtheta2(PxU32 t);
	std::vector<matrix<double>> compute_dMInv_dX_dthetaDot1(PxU32 t);
	std::vector<matrix<double>> compute_dMInv_dX_dthetaDot2(PxU32 t);

	// Math utility functions
	PxVec3 QuaternionToEuler(PxQuat q);
	PxReal SSDmatrix(matrix<double> A, matrix<double> B);
	PxReal SSDvector(std::vector<matrix<double>> A, std::vector<matrix<double>> B);
	matrix<double> clamp(matrix<double> theta);
	matrix<double> I(PxU32 x);
	std::vector<matrix<double>> vectorTranspose(std::vector<matrix<double>> vec);
	std::vector<matrix<double>> vectorSum(std::vector<matrix<double>> A, std::vector<matrix<double>> B);
	std::vector<matrix<double>> vectorDifference(std::vector<matrix<double>> A, std::vector<matrix<double>> B);
	std::vector<matrix<double>> vectorVectorProduct(std::vector<matrix<double>> A, std::vector<matrix<double>> B);
	std::vector<matrix<double>> vectorMatrixProduct(std::vector<matrix<double>> vec, matrix<double> mat);
	std::vector<matrix<double>> matrixVectorProduct(matrix<double> mat, std::vector<matrix<double>> vec);
	matrix<double> vec2mat(std::vector<matrix<double>> vec);
};