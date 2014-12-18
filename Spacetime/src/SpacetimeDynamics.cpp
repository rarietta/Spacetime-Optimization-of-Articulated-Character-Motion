//----------------------------------------------------------------------------------------------------------------------//
// SpacetimeKinematics.cpp																								//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//																														//
// This code...																											//
//----------------------------------------------------------------------------------------------------------------------//

#include "Spacetime.h"

#define R3 3
	
//======================================================================================================================//
// Build the Jacobian matrix for the system at the current timestep														//
//======================================================================================================================//

matrix<double> 
Spacetime::buildJacobian(void)
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
				PxVec3 global_com = xform1.transform(PxVec3(0,0,0));//body1->getCMassLocalPose().p);

				// get global position of joint
				jointBuffer2[j]->getActors(a2[0], a2[1]);
				PxRigidBody *body2 = (PxRigidBody *) a2[1];
				PxTransform xform2 = body2->getGlobalPose();
				PxVec3 global_jnt = xform2.transform(joint_local_positions[j]);

				// get global difference vector r
				PxVec3 diff = global_com - global_jnt;
				R(i,j) = vec3(diff.x, diff.y, diff.z);
			}
		}
	}

	matrix<double> J(R3*jointCount, DOF*jointCount);
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
					J(R3*j+X, i*DOF+n) = 0.0;
					J(R3*j+Y, i*DOF+n) = 0.0;
					J(R3*j+Z, i*DOF+n) = 0.0;
				} else {
					// populate matrix with 3-vector cross product 
					// of joint's n-th axis of rotation and r_cm_j
					// (vector from the joint to the j-th center of mass)
					vec3 cross_product = axis_n.Cross(R(j,i));
					J(R3*j+X, i*DOF+n) = cross_product[X];
					J(R3*j+Y, i*DOF+n) = cross_product[Y];
					J(R3*j+Z, i*DOF+n) = cross_product[Z];
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

matrix<double> 
Spacetime::computeGVector(void)
{
	// get all joints in scene
	PxRigidActor *a[2];
	PxU32 jointCount = gScene->getNbConstraints();
	PxConstraint** jointBuffer = new PxConstraint*[jointCount];
	gScene->getConstraints(jointBuffer, jointCount);
	
	matrix<double> G(R3*jointCount, 1);
	for (PxU32 i = 0; i < jointCount; i++) {
		jointBuffer[i]->getActors(a[0], a[1]);
		PxRigidBody *body1 = (PxRigidBody *) a[1];
		PxVec3 gravity = gScene->getGravity();
		PxReal mass = body1->getMass();
		G(R3*i+X, 0) = mass * gravity[X];
		G(R3*i+Y, 0) = mass * gravity[Y];
		G(R3*i+Z, 0) = mass * gravity[Z];
	}
	delete(jointBuffer);
	return ~buildJacobian()*G;
}

//======================================================================================================================//
// Apply a specified torque vector to the system																		//
//======================================================================================================================//

void 
Spacetime::applyTorqueVector(matrix<double> T)
{
	// get all joints in scene
	PxRigidActor *a[2];
	PxU32 jointCount = gScene->getNbConstraints();
	PxConstraint** jointBuffer = new PxConstraint*[jointCount];
	gScene->getConstraints(jointBuffer, jointCount);

	// apply torque to joints
	for (unsigned int i = 0; i < jointCount; i++) {
		jointBuffer[i]->getActors(a[PxJointActorIndex::eACTOR0], a[PxJointActorIndex::eACTOR1]);
		PxRigidBody *body0 = (PxRigidBody *) a[PxJointActorIndex::eACTOR0];
		PxRigidBody *body1 = (PxRigidBody *) a[PxJointActorIndex::eACTOR1];
		PxVec3 torque_i;
		if (DOF > X) { torque_i[X] = T(i*DOF+X,0); } else { torque_i[X] = 0.0f; }
		if (DOF > Y) { torque_i[Y] = T(i*DOF+Y,0); } else { torque_i[Y] = 0.0f; }
		if (DOF > Z) { torque_i[Z] = T(i*DOF+Z,0); } else { torque_i[Z] = 0.0f; }
		body0->addTorque( 1.0127 * torque_i);
		body1->addTorque(-1.0127 * torque_i);
	}

	// free memory
	delete(jointBuffer);
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

matrix<double> 
Spacetime::calculateAngularVelocity(void) 
{
	PxRigidActor *a[2];
	matrix<double> angularVelocityVector(DOF*joints.size(), 1);

	// find global angular velocity of each joints child body
	for (int i = 0; i < joints.size(); i++) {
		joints[i]->getActors(a[PxJointActorIndex::eACTOR0], a[PxJointActorIndex::eACTOR1]);
		PxRigidBody *body1 = (PxRigidBody *) a[PxJointActorIndex::eACTOR1];
		PxVec3 angularVelocity = body1->getAngularVelocity();
		if (DOF > X) { angularVelocityVector(i*DOF+X, 0) = angularVelocity[X]; }
		if (DOF > Y) { angularVelocityVector(i*DOF+Y, 0) = angularVelocity[Y]; }
		if (DOF > Z) { angularVelocityVector(i*DOF+Z, 0) = angularVelocity[Z]; }
	}

	return angularVelocityVector;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

matrix<double> 
Spacetime::calculateAngularPosition(void) 
{
	PxRigidActor *a[2];
	matrix<double> angularPositionVector(DOF*joints.size(), 1);

	for (unsigned int i = 0; i < joints.size(); i++) {
		joints[i]->getActors(a[PxJointActorIndex::eACTOR0], a[PxJointActorIndex::eACTOR1]);
		PxRigidBody *body1 = (PxRigidBody *) a[PxJointActorIndex::eACTOR1];
		PxTransform globalPose = body1->getGlobalPose();
		PxVec3 theta = QuaternionToEuler(globalPose.q);
		if (DOF > X) { angularPositionVector(i*DOF+X, 0) = theta[X]; }
		if (DOF > Y) { angularPositionVector(i*DOF+Y, 0) = theta[Y]; }
		if (DOF > Z) { angularPositionVector(i*DOF+Z, 0) = theta[Z]; }
	}

	return angularPositionVector;
}

//======================================================================================================================//
// Calculate the inverse mass matrix for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeInverseMassMatrix(matrix<double> G)
{
	matrix<double> M_inv(DOF*joints.size(), DOF*joints.size());
	for (int i = 0; i < DOF*joints.size(); i++) {

		// save state to restore after each calculation
		saveState();

		// zero out current velocities to eliminate C term
		for (int j = 0; j < dynamic_actors.size(); j++) {
			PxRigidDynamic *actor = dynamic_actors[j];
			actor->setAngularVelocity(PxVec3(0,0,0));
			actor->setLinearVelocity(PxVec3(0,0,0));
		}

		// solve for acceleration with modified input torque
		matrix<double> velocityBefore = calculateAngularVelocity();
		matrix<double> G_new(G);
		G_new(i, 0) += 1.0;
		applyTorqueVector(G_new);
		stepPhysics();
		matrix<double> velocityAfter = calculateAngularVelocity();
		matrix<double> angularAcceleration = (velocityAfter - velocityBefore) / (deltaT);

		// save acceleration as column of inverse mass matrix
		for (int j = 0; j < DOF*joints.size(); j++)
			M_inv(j,i) = angularAcceleration(j,0);

		// restore the state
		restoreState();
	}
	return M_inv;
}

//======================================================================================================================//
// Calculate the centripetal/Coriolis term for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeCVector(matrix<double> G, matrix<double> M)
{
	matrix<double> C(DOF*joints.size(), 1);

	saveState();
	matrix<double> velocityBefore = calculateAngularVelocity();
	applyTorqueVector(G);
	stepPhysics();
	matrix<double> velocityAfter = calculateAngularVelocity();
	matrix<double> angularAcceleration = (velocityAfter - velocityBefore) / (deltaT);
	C = M * angularAcceleration;
	restoreState();

	return C;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

void 
Spacetime::stepPhysics(void)
{
	// simulate and return
	gScene->simulate(deltaT);
	gScene->fetchResults(true);
}