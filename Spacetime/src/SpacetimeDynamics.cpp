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

//======================================================================================================================//
// Build the Jacobian matrix for the system at the current timestep														//
//======================================================================================================================//

matrix<PxReal> 
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
				PxVec3 global_com = xform1.transform(body1->getCMassLocalPose().p);

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

matrix<PxReal> 
Spacetime::buildForceVector(void)
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
// Apply a specified torque vector to the system																		//
//======================================================================================================================//

void 
Spacetime::applyTorqueVector(matrix<PxReal> T)
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
		body0->addTorque( 1.0127*PxVec3(T(i*DOF+X,0), T(i*DOF+Y,0), T(i*DOF+Z,0)));
		body1->addTorque(-1.0127*PxVec3(T(i*DOF+X,0), T(i*DOF+Y,0), T(i*DOF+Z,0)));
	}

	// free memory
	delete(jointBuffer);
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

matrix<PxReal> 
Spacetime::calculateAngularVelocity(void) 
{
	// create matrix
	matrix<PxReal> angularVelocityVector(DOF*joints.size(), 1);

	// apply torque to joints
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxVec3 angularVelocity = joints[i]->getRelativeAngularVelocity();
		angularVelocityVector(i*DOF+X, 0) = angularVelocity[X];
		angularVelocityVector(i*DOF+Y, 0) = angularVelocity[Y];
		angularVelocityVector(i*DOF+Z, 0) = angularVelocity[Z];
	}

	return angularVelocityVector;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

matrix<PxReal> 
Spacetime::calculateAngularPosition(void) 
{
	// create matrix
	matrix<PxReal> angularPositionVector(DOF*joints.size(), 1);

	// apply torque to joints
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxReal thetaX; PxVec3 axisX; 
		PxTransform localPose = joints[i]->getLocalPose(PxJointActorIndex::eACTOR1);
		localPose.q.toRadiansAndUnitAxis(thetaX, axisX);
		angularPositionVector(i*DOF+X, 0) = thetaX;
		angularPositionVector(i*DOF+Y, 0) = 0;
		angularPositionVector(i*DOF+Z, 0) = 0;
	}

	return angularPositionVector;
}

//======================================================================================================================//
// Calculate the inverse mass matrix for the system at the current state
//======================================================================================================================//

matrix<PxReal> 
Spacetime::computeInverseMassMatrix(matrix<PxReal> T)
{
	matrix<PxReal> M_inv(DOF*joints.size(), DOF*joints.size());
	for (int i = 0; i < DOF*joints.size(); i++) {
		saveState();
		matrix<PxReal> velocityBefore = calculateAngularVelocity();
		matrix<PxReal> T_new(T);
		T_new(i, 0) += 1;
		applyTorqueVector(T_new);
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
		matrix<PxReal> velocityAfter = calculateAngularVelocity();
		matrix<PxReal> angularAcceleration = (velocityAfter - velocityBefore) / (1.0f/60.0f);
		for (int j = 0; j < DOF*joints.size(); j++)
			M_inv(j,i) = angularAcceleration(j,0);
		restoreState();
	}
	return M_inv;
}

//======================================================================================================================//
// Calculate the centripetal/Coriolis term for the system at the current state
//======================================================================================================================//

matrix<PxReal> 
Spacetime::computeCVector(matrix<PxReal> T, matrix<PxReal> M)
{
	matrix<PxReal> C(DOF*joints.size(), 1);

	saveState();
	matrix<PxReal> velocityBefore = calculateAngularVelocity();
	applyTorqueVector(T);
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
	matrix<PxReal> velocityAfter = calculateAngularVelocity();
	matrix<PxReal> angularAcceleration = (velocityAfter - velocityBefore) / (1.0f/60.0f);
	C = M * angularAcceleration;
	restoreState();

	return C;
}

//======================================================================================================================//
// Calculate the derivative of the dynamics equation f with respect to the state vector X								//
//======================================================================================================================//

matrix<PxReal> 
Spacetime::compute_dfdx(PxU32 t)
{
	// TODO
	matrix<PxReal> dfdx(1, 1);
	return dfdx;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

void 
Spacetime::stepPhysics(void)
{
	// NOTE:
	// Inverse Kinematics Formula
	// ddTheta = M_inv * (T - C(Theta, dTheta) - G(Theta))

	// do not simulate if the user has paused
	if (pause) return;

	// simulate and return
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}