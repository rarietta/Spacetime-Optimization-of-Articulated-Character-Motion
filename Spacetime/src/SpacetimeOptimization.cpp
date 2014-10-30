//======================================================================================================================//
// SpacetimeOptimization.cpp																							//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//																														//
// This code...																											//
//======================================================================================================================//

#define PVD_HOST "127.0.0.1"

#include "PxPhysicsAPI.h"
#include "SvzAlgebra.h"
#include "SvzMatrix.h"
#include <iostream>
#include <assert.h>
#include <ctype.h>
#include <vector>

using namespace std;
using namespace physx;

//======================================================================================================================//
// Define global variables for spacetime optimization																	//
//======================================================================================================================//

// change this to suppress verbose debugging output
#define V_DEBUG 1

// for accessing X, Y, and Z components
#define DOF 3
enum {X, Y, Z};

// list of actor components
enum {BASE, LEG1, LEG2, HEAD};

// is the program running?
bool pause = false;

// Required physX runtime variable initializations
PxScene* gScene = NULL;
PxPhysics* gPhysics = NULL;
PxMaterial*	gMaterial	= NULL;
PxFoundation* gFoundation = NULL;
PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxVisualDebuggerConnection*	gConnection	= NULL;
PxDefaultAllocator gAllocator;
PxDefaultErrorCallback gErrorCallback;

// list of joint positions relative to attached bodies
std::vector<PxRigidStatic*> static_actors;
std::vector<PxRigidDynamic*> dynamic_actors;
std::vector<PxVec3> joint_local_positions;

//======================================================================================================================//
// Hard-coded function to create custom static environment actors (called by initPhysics() on load)						//
//======================================================================================================================//

void addStaticActors(void)
{
	//--------------------------------------------------------------------------//
	// ground plane actor														//
	//--------------------------------------------------------------------------//

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	static_actors.push_back(groundPlane);
}

//======================================================================================================================//
// Hard-coded function to create custom dynamic actors in character (called by initPhysics() on load)					//
//======================================================================================================================//

void addDynamicActors(void)
{
	// universal material properties
	PxMaterial* mMaterial = gPhysics->createMaterial(0.5,0.5,0.5);
	PxReal density = 1.0f; 

	// reusable transform
	PxTransform transform(PxVec3(0,0,0), PxQuat::createIdentity()); 

	//--------------------------------------------------------------------------//
	// lamp base actor															//
	//--------------------------------------------------------------------------//

	// lamp base geometry
	PxBoxGeometry base_geometry(4.0, 0.5, 4.0);

	// lamp base transformation
	PxVec3 base_translation(0,0.6f,0);
	PxQuat base_rotation(PxQuat::createIdentity());
	PxTransform base_transform(base_translation, base_rotation);

	// lamp base creation
	PxRigidDynamic *base = PxCreateDynamic(*gPhysics, base_transform, 
		base_geometry, *mMaterial, density);
	if (!base)
		cerr << "create actor failed!" << endl;
	
	// lamp base properties
	base->setAngularDamping(0.75);
    base->setLinearVelocity(PxVec3(0,0,0)); 
	base->setMass(PxReal(200));
	base->setName("base");

	// lamp base addition
	gScene->addActor(*base);
	dynamic_actors.push_back(base);

	//--------------------------------------------------------------------------//
	// lamp first leg actor														//
	//--------------------------------------------------------------------------//

	// first leg geometry
	PxBoxGeometry leg1_geometry(0.5, 10.0, 0.5);

	// first leg transformation
	PxVec3 leg1_translation(0.0f, 10.0f*sin(PxPi/4.0f)+1.1f, -10.0f*cos(PxPi/4.0f));
	PxQuat leg1_rotation(-PxPi/4.0f, PxVec3(1,0,0));
	PxTransform leg1_transform(leg1_translation, leg1_rotation);
	
	// first leg creation
	PxRigidDynamic *leg1 = PxCreateDynamic(*gPhysics, leg1_transform, 
		leg1_geometry, *mMaterial, density);
	if (!leg1)
		cerr << "create actor failed!" << endl;

	// first leg properties
    leg1->setAngularDamping(0.75);
    leg1->setLinearVelocity(PxVec3(0,0,0));
	leg1->setMass(10.0f);
	leg1->setName("leg1");

	// first leg addition
	gScene->addActor(*leg1);
	dynamic_actors.push_back(leg1);

	//--------------------------------------------------------------------------//
	// lamp second leg actor													//
	//--------------------------------------------------------------------------//

	// second leg geometry
	PxBoxGeometry leg2_geometry(0.5, 10.0, 0.5);

	// second leg transformation
	transform.p = PxVec3(0,20.0f*sin(PxPi/4.0f)+10.0f*sin(PxPi/4.0f)+1.1f, -20.0f*cos(PxPi/4.0f)+10.0f*cos(PxPi/4.0f));
	transform.q = PxQuat(PxPi/4.0f, PxVec3(1,0,0));

	// second leg creation
	PxRigidDynamic *leg2 = PxCreateDynamic(*gPhysics, transform, 
		leg2_geometry, *mMaterial, density);
	if (!leg2)
		cerr << "create actor failed!" << endl;

	// second leg properties
	leg2->setAngularDamping(0.75);
	leg2->setMass(10);
	leg2->setName("leg2");

	// second leg addition
	gScene->addActor(*leg2);
	dynamic_actors.push_back(leg2);

	//--------------------------------------------------------------------------//
	// lamp head actor															//
	//--------------------------------------------------------------------------//

	// lamp head geometry
	PxSphereGeometry bulb_geometry(PxReal(2.0));

	// lamp head transformation
	transform.p = PxVec3(0,18.6f,0);

	// lamp head creation
	PxRigidDynamic *bulb = PxCreateDynamic(*gPhysics, transform, 
		bulb_geometry, *mMaterial, density);
	if (!bulb)
		cerr << "create actor failed!" << endl;
	
	// lamp head properties
	bulb->setAngularDamping(0.75);
	bulb->setMass(5);
	bulb->setName("bulb");

	// lamp head addition
	//gScene->addActor(*bulb);
	//dynamic_actors.push_back(bulb);
}

//======================================================================================================================//
// Hard-coded function to create custom joints in character (called by initPhysics() on load)							//
//======================================================================================================================//

void addJoints(void)
{
	// reusable transforms
	PxTransform body0_transform;
	PxTransform body1_transform;

	// Create spherical joint at base
	PxQuat base_quat1(PxPi/2.0f, PxVec3(0,0,1));
	PxQuat base_quat2(PxPi/8.0f, PxVec3(0,1,0));
	body0_transform = PxTransform(PxVec3(0,  0.5f, 0), base_quat1*base_quat2);
	body1_transform = PxTransform(PxVec3(0, -10.0f, 0), base_quat1);
	PxD6Joint* base_joint = PxD6JointCreate(*gPhysics, 
		dynamic_actors[BASE], body0_transform, dynamic_actors[LEG1], body1_transform);
	base_joint->setSwingLimit(PxJointLimitCone(PxPi/2, PxPi/2));
	base_joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
	base_joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	base_joint->setConstraintFlag(PxConstraintFlag::eREPORTING, true);
	base_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
	joint_local_positions.push_back(PxVec3(0,-10,0));

	// Create hinge joint between legs
	body0_transform = PxTransform(PxVec3(0,  10.0, 0));
	body1_transform = PxTransform(PxVec3(0, -10.0, 0), PxQuat(-PxPi/2.0, PxVec3(1,0,0)));
	PxRevoluteJoint* leg_joint = PxRevoluteJointCreate(*gPhysics, 
		dynamic_actors[LEG1], body0_transform, dynamic_actors[LEG2], body1_transform);
	leg_joint->setLimit(PxJointAngularLimitPair(3.0*PxPi/4.0, -1.0*PxPi/4.0));
	leg_joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
	leg_joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
	joint_local_positions.push_back(PxVec3(0,-10,0));

	/*
	// Create bulb joint
	body0_transform = PxTransform(PxVec3(0, 4.0,0));
	body1_transform = PxTransform(PxVec3(0,-2.0,0));
	PxSphericalJoint* bulb_joint = PxSphericalJointCreate(*gPhysics,
		dynamic_actors[LEG2],body0_transform, dynamic_actors[HEAD], body1_transform);
	bulb_joint->setLimitCone(PxJointLimitCone(0.5,0.5,PxReal(1.0f)));
	bulb_joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	*/
}

//======================================================================================================================//
// Initialize physics setup and call hard-coded functions to create custom actors and joints							//
//======================================================================================================================//

void initPhysics(void)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, profileZoneManager);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity		= PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher				= PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene					= gPhysics->createScene(sceneDesc);

	#ifdef V_DEBUG
		gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
		gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 5.0f);
		gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 50.0f);
		gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 20.0f);
		gScene->setVisualizationParameter(PxVisualizationParameter::eBODY_ANG_VELOCITY, 50.0f);
	#endif

	addStaticActors();
	addDynamicActors();
	addJoints();
}

//======================================================================================================================//
// Build the Jacobian matrix for the system at the current timestep														//
//======================================================================================================================//

matrix<PxReal> buildJacobian(void)
{
	// get all joints in scene
	PxRigidActor *a1[2], *a2[2];
	PxU32 jointCount = gScene->getNbConstraints();
	PxConstraint** jointBuffer1 = new PxConstraint*[jointCount];
	PxConstraint** jointBuffer2 = new PxConstraint*[jointCount];
	gScene->getConstraints(jointBuffer1, jointCount);
	gScene->getConstraints(jointBuffer2, jointCount);
		
	// calculate arm lengths r
	matrix<vec3> R(jointCount, jointCount);
	for (PxU32 i = 0; i < jointCount; i++) {
		for (PxU32 j = 0; j < jointCount; j++)
		{
			if (i >= j) {
				// get global position of center of mass
				jointBuffer1[i]->getActors(a1[0], a1[1]);
				PxRigidBody *body1 = (PxRigidBody *) a1[1];
				PxTransform xform1 = body1->getGlobalPose();
				PxVec3 global_com = xform1.transform(body1->getCMassLocalPose().p);

				// get global position of joint
				jointBuffer2[j]->getActors(a2[0], a2[1]);
				PxRigidBody *body2 = (PxRigidBody *) a2[1];
				PxTransform xform2 = body2->getGlobalPose();
				PxVec3 global_jnt = xform2.transform(joint_local_positions[j]);

				// get global difference vector r
				PxVec3 diff = global_com - global_jnt;
				R(i,j) = vec3(diff.x, diff.y, diff.z);

				#ifdef V_DEBUG_DIFF
					printf("global_jnt[%d] = <%f,%f,%f>\n", j, global_jnt.x, global_jnt.y, global_jnt.z);
					printf("global_com[%d] = <%f,%f,%f>\n", i, global_com.x, global_com.y, global_com.z);
					cout << "diff[" << j << "->" << i << "] = <" << diff.x << ", " << diff.y << ", " << diff.z << ">" << endl;
				#endif
			}
		}
	}

	matrix<PxReal> J(jointCount*DOF, jointCount*DOF);
	for (PxU32 i = 0; i < jointCount; i++)
	{
		// get joint axes a_x, a_y, a_z
		std::vector<vec3> axes;
		vec3 a_xi(1,0,0); axes.push_back(a_xi);
		vec3 a_yi(0,1,0); axes.push_back(a_yi);
		vec3 a_zi(0,0,1); axes.push_back(a_zi);

		// build individual J_i matrix of size jointCount*DOF x DOF
		for (int n = 0; n < DOF; n++) {
			vec3 axis_n = axes[n];
			for (PxU32 j = 0; j < jointCount; j++) {
				if (j < i) {
					// if the current body is unaffected by the joint
					// the values in the row of the J_i matrix are 0
					J(j*DOF+X, i*DOF+n) = 0.0f;
					J(j*DOF+Y, i*DOF+n) = 0;
					J(j*DOF+Z, i*DOF+n) = 0;
				} else {
					// populate matrix with 3-vector cross product 
					// of joint's n-th axis of rotation and r_cm_j
					// (vector from the joint to the j-th center of mass)
					vec3 cross_product = axis_n.Cross(R(j,i));
					J(j*DOF+X, i*DOF+n) = cross_product[X];
					J(j*DOF+Y, i*DOF+n) = cross_product[Y];
					J(j*DOF+Z, i*DOF+n) = cross_product[Z];
				}
			}
		}
	}

	delete(jointBuffer1);
	delete(jointBuffer2);
	return J;
}

//======================================================================================================================//
// Build the force vector due to gravity at the current timestep														//
//======================================================================================================================//

matrix<PxReal> buildForceVector(void)
{
	// get all joints in scene
	PxRigidActor *a[2];
	PxU32 jointCount = gScene->getNbConstraints();
	PxConstraint** jointBuffer = new PxConstraint*[jointCount];
	gScene->getConstraints(jointBuffer, jointCount);

	matrix<PxReal> F(jointCount*DOF, 1);
	for (PxU32 i = 0; i < jointCount; i++) {
		jointBuffer[i]->getActors(a[0], a[1]);
		PxRigidBody *body1 = (PxRigidBody *) a[1];
		PxVec3 gravity = gScene->getGravity();
		PxReal mass = body1->getMass();
		F(i*DOF+X, 0) = (mass * gravity).x;
		F(i*DOF+Y, 0) = (mass * gravity).y;
		F(i*DOF+Z, 0) = (mass * gravity).z;
	}

	delete(jointBuffer);
	return F;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

void stepPhysics(void)
{
	if (pause) return;

	#ifdef V_DEBUG
		cout << "-----------------------------------------------------------------------------------------------" << endl;
		cout << "time = " << gScene->getTimestamp() << endl;
		cout << endl;
	#endif
	
	// get all joints in scene
	PxRigidActor *a[2];
	PxU32 jointCount = gScene->getNbConstraints();
	PxConstraint** jointBuffer = new PxConstraint*[jointCount];
	gScene->getConstraints(jointBuffer, jointCount);
		
	// virtual work calculation
	matrix<PxReal> J = buildJacobian();
	matrix<PxReal> F = buildForceVector();
	matrix<PxReal> T = ~J*F;

	// apply torques to maintain static equilibrium
	for (unsigned int i = 0; i < jointCount; i++)
	{
		jointBuffer[i]->getActors(a[0], a[1]);
		PxRigidBody *body0 = (PxRigidBody *) a[0];
		PxRigidBody *body1 = (PxRigidBody *) a[1];
		body0->addTorque( 1.0127*PxVec3(T(i*DOF+X,0), T(i*DOF+Y,0), T(i*DOF+Z,0)));
		body1->addTorque(-1.0127*PxVec3(T(i*DOF+X,0), T(i*DOF+Y,0), T(i*DOF+Z,0)));

		#ifdef V_DEBUG
			cout << "J = \n" << J << endl;
			cout << "F = \n" << F << endl;
			printf("The torque on the joint between %s and %s is equal to \n\t<%f,%f,%f>\n\n\n", 
				a[0]->getName(), a[1]->getName(), -T(i*DOF+X,0), -T(i*DOF+Y,0), -T(i*DOF+Z,0));
		#endif
	}

	// free memory
	delete(jointBuffer);

	// simulate and return
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}

//======================================================================================================================//
// Required physX function for terminating physics engine																//
//======================================================================================================================//

void cleanupPhysics(void)
{
	gScene->release();
	gDispatcher->release();
	PxProfileZoneManager* profileZoneManager = gPhysics->getProfileZoneManager();
	if(gConnection != NULL)
		gConnection->release();
	gPhysics->release();	
	profileZoneManager->release();
	gFoundation->release();
}

//======================================================================================================================//
// Process user input -- only supports pausing in current state															//
//======================================================================================================================//

void keyPress(const char key)
{
	switch(toupper(key)) {
		case 'P': pause = !pause; break;
	}
}

//======================================================================================================================//
// Main render loop for physics simulation																				//
//======================================================================================================================//

#if defined(PX_XBOXONE) || defined(PX_WINMODERN)
int main(Platform::Array<Platform::String^>^)
#else
int main(int, char**)
#endif
{
	extern void renderLoop();
	renderLoop();
}
