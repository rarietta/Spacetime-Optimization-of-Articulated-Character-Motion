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
	// calculate arm lengths r
	matrix<vec3> R(joints.size(), joints.size());
	for (PxU32 j = 0; j < joints.size(); j++) {
		for (PxU32 i = j; i < joints.size(); i++)
		{
			// get global position of center of mass
			PxTransform xform1 = dynamic_actors[i+1]->getGlobalPose();
			PxVec3 global_com = xform1.transform(PxVec3(0,0,0));

			// get global position of joint
			PxTransform xform2 = dynamic_actors[j+1]->getGlobalPose();
			PxVec3 global_jnt = xform2.transform(joint_local_positions[j]);

			// get global difference vector r
			PxVec3 diff = global_com - global_jnt;
			R(i,j) = vec3(diff.x, diff.y, diff.z);
		}
	}

	matrix<double> J(R3*joints.size(), DOF*joints.size());
	for (PxU32 i = 0; i < joints.size(); i++)
	{
		// get joint axes a_x, a_y, a_z
		std::vector<vec3> axes;
		vec3 a_xi(1,0,0); axes.push_back(a_xi);
		vec3 a_yi(0,1,0); axes.push_back(a_yi);
		vec3 a_zi(0,0,1); axes.push_back(a_zi);

		// build individual J_i matrix of size jointCount*DOF x DOF
		for (int n = 0; n < DOF; n++) {
			vec3 axis_n = axes[n];
			for (PxU32 j = 0; j < joints.size(); j++) {
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
	return J;
}

//======================================================================================================================//
// Build the force vector due to gravity at the current timestep														//
//======================================================================================================================//

matrix<double> 
Spacetime::computeG_numeric(void)
{
	matrix<double> G(R3*joints.size(), 1);
	for (PxU32 i = 0; i < joints.size(); i++) {
		PxVec3 gravity = gScene->getGravity();
		PxReal mass = dynamic_actors[i+1]->getMass();
		G(R3*i+X, 0) = mass * gravity[X];
		G(R3*i+Y, 0) = mass * gravity[Y];
		G(R3*i+Z, 0) = mass * gravity[Z];
	}
	matrix<double> J = buildJacobian();
	return ~J*G;
}

//======================================================================================================================//
// Apply a specified torque vector to the system																		//
//======================================================================================================================//

void 
Spacetime::applyTorqueVector(matrix<double> T)
{
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxVec3 torque_i;
		if (DOF > X) { torque_i[X] = T(i*DOF+X,0); } else { torque_i[X] = 0.0f; }
		if (DOF > Y) { torque_i[Y] = T(i*DOF+Y,0); } else { torque_i[Y] = 0.0f; }
		if (DOF > Z) { torque_i[Z] = T(i*DOF+Z,0); } else { torque_i[Z] = 0.0f; }
		dynamic_actors[ i ]->addTorque( 1.0127 * torque_i);
		dynamic_actors[i+1]->addTorque(-1.0127 * torque_i);
	}
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

matrix<double> 
Spacetime::calculateAngularVelocity(void) 
{
	matrix<double> angularVelocityVector(DOF*joints.size(), 1);
	for (int i = 0; i < joints.size(); i++) {
		PxVec3 angularVelocity = dynamic_actors[i+1]->getAngularVelocity();
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
	matrix<double> angularPositionVector(DOF*joints.size(), 1);
	for (unsigned int i = 0; i < joints.size(); i++) {
		PxVec3 theta = QuaternionToEuler(dynamic_actors[i+1]->getGlobalPose().q);
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
Spacetime::computeMInv_numeric(matrix<double> G)
{
	matrix<double> MInv(DOF*joints.size(), DOF*joints.size());
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
		stepPhysics_numeric(G_new);
		matrix<double> velocityAfter = calculateAngularVelocity();
		matrix<double> angularAcceleration = (velocityAfter - velocityBefore) / deltaT;

		// save acceleration as column of inverse mass matrix
		for (int j = 0; j < DOF*joints.size(); j++)
			MInv(j,i) = angularAcceleration(j,0);

		// restore the state
		restoreState();
	}
	return MInv;
}

//======================================================================================================================//
// Calculate the centripetal/Coriolis term for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeC_numeric(matrix<double> G, matrix<double> M)
{
	matrix<double> C(DOF*joints.size(), 1);

	saveState();
	matrix<double> velocityBefore = calculateAngularVelocity();
	stepPhysics_numeric(G);
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
Spacetime::stepPhysics_numeric(matrix<double> u)
{
	// simulate and return
	applyTorqueVector(u);
	gScene->simulate(deltaT);
	gScene->fetchResults(true);
}