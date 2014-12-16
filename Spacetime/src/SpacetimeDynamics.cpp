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

	matrix<double> J(jointCount*3, jointCount*DOF);
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
					J(j*3+X, i*DOF+n) = 0.0;
					J(j*3+Y, i*DOF+n) = 0.0;
					J(j*3+Z, i*DOF+n) = 0.0;
				} else {
					// populate matrix with 3-vector cross product 
					// of joint's n-th axis of rotation and r_cm_j
					// (vector from the joint to the j-th center of mass)
					vec3 cross_product = axis_n.Cross(R(j,i));
					if (!pause) {
						cout << "R(j,i) = <" << R(j,i)[X] << ", "  << R(j,i)[Y] << ", " << R(j,i)[Z] << ">" << endl;
						cout << cross_product[X] << endl;
					}
					J(j*3+X, i*DOF+n) = cross_product[X];
					J(j*3+Y, i*DOF+n) = cross_product[Y];
					J(j*3+Z, i*DOF+n) = cross_product[Z];
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
	
	//short int tag = 0;
	//trace_on(tag);
	
	// declaration of dependent and independent variables
	// for use in ADOL-C derivative calculations
	//adouble *_X, *_dGdX;
	//_X = new adouble[jointCount*DOF*2];
	//_dGdX = new adouble[jointCount*DOF];

	//matrix<double> state = buildStateVector();
	//for (int i = 0; i < jointCount*DOF*2; i++)
	//	_X[i] <<= state(i,1);

	matrix<double> G(3*jointCount, 1);
	for (PxU32 i = 0; i < jointCount; i++) {
		jointBuffer[i]->getActors(a[0], a[1]);
		PxRigidBody *body1 = (PxRigidBody *) a[1];
		PxVec3 gravity = gScene->getGravity();
		PxReal mass = body1->getMass();
		//_dGdX[i*DOF+X] = (mass * gravity).x;
		//_dGdX[i*DOF+Y] = (mass * gravity).y;
		//_dGdX[i*DOF+Z] = (mass * gravity).z;
		G(i*3+X, 0) = mass * gravity[X];
		G(i*3+Y, 0) = mass * gravity[Y];
		G(i*3+Z, 0) = mass * gravity[Z];
	}

	//for (int i = 0; i < jointCount*DOF; i++)
	//	_dGdX[i] >>= G(i,1);

	delete(jointBuffer);
	//delete(_dGdX);
	//delete(_X);

	//trace_off();

	//cout << "G = \n" << G << endl;
	return G;
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
	// create matrix
	matrix<double> angularVelocityVector(DOF*joints.size(), 1);

	// apply torque to joints
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxVec3 angularVelocity = joints[i]->getRelativeAngularVelocity();
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
	// create matrix
	matrix<double> angularPositionVector(DOF*joints.size(), 1);

	// apply torque to joints
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxReal thetaX; PxVec3 axisX;
		PxReal thetaY; PxVec3 axisY;
		PxReal thetaZ; PxVec3 axisZ; 
		PxTransform localPose = joints[i]->getLocalPose(PxJointActorIndex::eACTOR1);
		localPose.q.toRadiansAndUnitAxis(thetaX, axisX);
		localPose.q.toRadiansAndUnitAxis(thetaY, axisY);
		localPose.q.toRadiansAndUnitAxis(thetaZ, axisZ);
		cout << "thetaX = " << thetaX << endl;
		cout << "axisX = <" << axisX[X] << ", " << axisX[Y] << ", " << axisX[Z] << ">" << endl;
		if (DOF > X) { angularPositionVector(i*DOF+X, 0) = thetaX; }
		if (DOF > Y) { angularPositionVector(i*DOF+Y, 0) = thetaY; }
		if (DOF > Z) { angularPositionVector(i*DOF+Z, 0) = thetaZ; }
	}

	return angularPositionVector;
}

//======================================================================================================================//
// Calculate the inverse mass matrix for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeInverseMassMatrix(matrix<double> T)
{
	matrix<double> M_inv(DOF*joints.size(), DOF*joints.size());
	for (int i = 0; i < DOF*joints.size(); i++) {

		// save state to restore after each calculation
		saveState();

		// zero out current velocities to eliminate C term
		for (int i = 0; i < dynamic_actors.size(); i++) {
			PxRigidDynamic *actor = dynamic_actors[i];
			actor->setAngularVelocity(PxVec3(0,0,0));
			actor->setLinearVelocity(PxVec3(0,0,0));
		}

		// solve for acceleration with modified input torque
		matrix<double> velocityBefore = calculateAngularVelocity();
		matrix<double> T_new(T);
		T_new(i, 0) += 1.0;
		applyTorqueVector(T_new);
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
		matrix<double> velocityAfter = calculateAngularVelocity();
		matrix<double> angularAcceleration = (velocityAfter - velocityBefore) / (1.0/60.0);

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
Spacetime::computeCVector(matrix<double> T, matrix<double> M)
{
	matrix<double> C(DOF*joints.size(), 1);

	saveState();
	matrix<double> velocityBefore = calculateAngularVelocity();
	applyTorqueVector(T);
	gScene->simulate(1.0/60.0);
	gScene->fetchResults(true);
	matrix<double> velocityAfter = calculateAngularVelocity();
	matrix<double> angularAcceleration = (velocityAfter - velocityBefore) / (1.0/60.0);
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
	// NOTE:
	// Inverse Kinematics Formula
	// ddTheta = M_inv * (T - C(Theta, dTheta) - G(Theta))

	// do not simulate if the user has paused
	if (pause) return;

	// simulate and return
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}