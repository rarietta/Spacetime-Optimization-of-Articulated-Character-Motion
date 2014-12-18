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

void 
Spacetime::addDynamicActors(void)
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
	root = PxVec3(0,0.6,0);
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
	base->setMass(PxReal(5000));
	base->setName("base");
	base->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

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
	leg1->setMass(0.10f);
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
	leg2->setMass(0.10f);
	leg2->setName("leg2");

	// second leg addition
	gScene->addActor(*leg2);
	dynamic_actors.push_back(leg2);

	/*
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
	// lamp third leg actor													//
	//--------------------------------------------------------------------------//

	// third leg geometry
	PxBoxGeometry leg3_geometry(0.5, 10.0, 0.5);

	// second leg transformation
	transform.p = PxVec3(0,20.0f*sin(PxPi/4.0f)+20.0f*sin(PxPi/4.0f)+1.1f, -20.0f*cos(PxPi/4.0f)+20.0f*cos(PxPi/4.0f)+10);
	transform.q = PxQuat(PxPi/2.0f, PxVec3(1,0,0));

	// second leg creation
	PxRigidDynamic *leg3 = PxCreateDynamic(*gPhysics, transform, 
		leg3_geometry, *mMaterial, density);
	if (!leg3)
		cerr << "create actor failed!" << endl;

	// second leg properties
	leg3->setAngularDamping(0.75);
	leg3->setMass(10);
	leg3->setName("leg3");

	// second leg addition
	gScene->addActor(*leg3);
	dynamic_actors.push_back(leg3);

	//--------------------------------------------------------------------------//
	// lamp head actor															//
	//--------------------------------------------------------------------------//

	// lamp head geometry
	PxSphereGeometry bulb_geometry(PxReal(2.0));

	// lamp head transformation
	transform.p = PxVec3(0,20.0f*sin(PxPi/4.0f)+20.0f*sin(PxPi/4.0f)+1.1f, -20.0f*cos(PxPi/4.0f)+20.0f*cos(PxPi/4.0f)+22);
	transform.q = PxQuat(PxPi/2.0f, PxVec3(1,0,0));

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
	gScene->addActor(*bulb);
	dynamic_actors.push_back(bulb);
	*/
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

	// Create spherical joint at base
	PxQuat base_quat1(PxPi/2.0f, PxVec3(0,0,1));
	body0_transform = PxTransform(PxVec3(0,  0.5f, 0), base_quat1);
	body1_transform = PxTransform(PxVec3(0, -10.0f, 0), base_quat1);
	PxD6Joint* base_joint = PxD6JointCreate(*gPhysics, 
		dynamic_actors[BASE], body0_transform, dynamic_actors[LEG1], body1_transform);
	base_joint->setSwingLimit(PxJointLimitCone(PxPi/2, PxPi/2));
	base_joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
	base_joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	joint_local_positions.push_back(PxVec3(0,-10,0));
	joints.push_back((PxJoint*)base_joint);

	// Create hinge joint between legs
	body0_transform = PxTransform(PxVec3(0,  10.0, 0));
	body1_transform = PxTransform(PxVec3(0, -10.0, 0), PxQuat(-PxPi/2.0, PxVec3(1,0,0)));
	PxRevoluteJoint* leg_joint = PxRevoluteJointCreate(*gPhysics, 
		dynamic_actors[LEG1], body0_transform, dynamic_actors[LEG2], body1_transform);
	joint_local_positions.push_back(PxVec3(0,-10,0));
	joints.push_back((PxJoint*)leg_joint);

	/*
	// Create hinge joint between legs
	body0_transform = PxTransform(PxVec3(0,  10.0, 0));
	body1_transform = PxTransform(PxVec3(0, -10.0, 0), PxQuat(-PxPi/4.0, PxVec3(1,0,0)));
	PxRevoluteJoint* leg_joint2 = PxRevoluteJointCreate(*gPhysics, 
		dynamic_actors[LEG2], body0_transform, dynamic_actors[LEG3], body1_transform);
	leg_joint2->setLimit(PxJointAngularLimitPair(3.0*PxPi/4.0, -1.0*PxPi/4.0));
	leg_joint2->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
	leg_joint2->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
	joint_local_positions.push_back(PxVec3(0,-10,0));
	joints.push_back((PxJoint*)leg_joint2);

	// Create bulb joint
	body0_transform = PxTransform(PxVec3(0, 10.0, 0));
	body1_transform = PxTransform(PxVec3(0, -2.0, 0));
	PxSphericalJoint* bulb_joint = PxSphericalJointCreate(*gPhysics,
		dynamic_actors[LEG3],body0_transform, dynamic_actors[HEAD], body1_transform);
	bulb_joint->setLimitCone(PxJointLimitCone(0.5,0.5,PxReal(1.0f)));
	bulb_joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	joint_local_positions.push_back(PxVec3(0,-2,0));
	joints.push_back((PxJoint*)bulb_joint);
	*/
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