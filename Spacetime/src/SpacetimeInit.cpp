//-------------------------------------------------------------------------------------------------//
// Spacetime.cpp
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#include "Spacetime.h"

//======================================================================================================================//
// Hard-coded function to create custom static environment actors (called by initPhysics() on load)						//
//======================================================================================================================//

void 
Spacetime::addStaticActors(void)
{
	// ground plane actor
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	static_actors.push_back(groundPlane);
}

//======================================================================================================================//
// Hard-coded function to create custom dynamic actors in character (called by initPhysics() on load)					//
//======================================================================================================================//

void 
Spacetime::addDynamicActors(void)
{
	// universal material properties
	PxMaterial* mMaterial = gPhysics->createMaterial(0.5,0.5,0.5);
	PxReal density = 1.0f; 

	// base geometry
	PxBoxGeometry base_geometry(4.0, 0.5, 4.0);
	PxVec3 base_translation(0,0.6f,0); root = base_translation;
	PxQuat base_rotation(PxQuat::createIdentity());
	PxTransform base_transform(base_translation, base_rotation);
	PxRigidDynamic *base = PxCreateDynamic(*gPhysics, base_transform, 
		base_geometry, *mMaterial, density);
	if (!base) cerr << "create actor failed!" << endl;
	base->setAngularDamping(0.75);
    base->setLinearVelocity(PxVec3(0,0,0));
	base->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*base);
	dynamic_actors.push_back(base);
	
	// first leg geometry
	PxBoxGeometry leg1_geometry(0.5, 0.5, 10.0);
	PxVec3 leg1_translation(0,0,0);
	PxQuat leg1_rotation = PxQuat::createIdentity();
	PxTransform leg1_transform(leg1_translation, leg1_rotation);
	PxRigidDynamic *leg1 = PxCreateDynamic(*gPhysics, leg1_transform, 
		leg1_geometry, *mMaterial, density);
	if (!leg1) cerr << "create actor failed!" << endl;
    leg1->setAngularDamping(0.75);
	leg1->setMass(10.0f);
	gScene->addActor(*leg1);
	dynamic_actors.push_back(leg1);

	// second leg geometry
	PxBoxGeometry leg2_geometry(0.5, 0.5, 10.0);
	PxVec3 leg2_translation = PxVec3(0,0,0);
	PxQuat leg2_rotation = PxQuat::createIdentity();
	PxTransform leg2_transform(leg2_translation, leg2_rotation);
	PxRigidDynamic *leg2 = PxCreateDynamic(*gPhysics, leg2_transform, 
		leg2_geometry, *mMaterial, density);
	if (!leg2) cerr << "create actor failed!" << endl;
	leg2->setAngularDamping(0.75);
	leg2->setMass(10.0f);
	gScene->addActor(*leg2);
	dynamic_actors.push_back(leg2);
}

//======================================================================================================================//
// Hard-coded function to create custom joints in character (called by initPhysics() on load)							//
//======================================================================================================================//

void 
Spacetime::addJoints(void)
{
	// reusable transforms
	PxTransform body0_transform;
	PxTransform body1_transform;

	// Create hinge joint 1
	body0_transform = PxTransform(PxVec3(0, 0.5, 0));
	body1_transform = PxTransform(PxVec3(0, 0, 10.0));
	PxRevoluteJoint* joint1 = PxRevoluteJointCreate(*gPhysics, 
		dynamic_actors[BASE], body0_transform, dynamic_actors[LEG1], body1_transform);
	joint_local_positions.push_back(body1_transform.p);
	joints.push_back((PxJoint*)joint1);

	// Create hinge joint 2
	body0_transform = PxTransform(PxVec3(0, 0, -10.0));
	body1_transform = PxTransform(PxVec3(0, 0, 10.0));
	PxRevoluteJoint* joint2 = PxRevoluteJointCreate(*gPhysics, 
		dynamic_actors[LEG1], body0_transform, dynamic_actors[LEG2], body1_transform);
	joint_local_positions.push_back(body1_transform.p);
	joints.push_back((PxJoint*)joint2);
}

//======================================================================================================================//
// Initialize physics setup and call hard-coded functions to create custom actors and joints							//
//======================================================================================================================//

void 
Spacetime::initPhysics(void)
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

	addStaticActors();
	addDynamicActors();
	addJoints();
}