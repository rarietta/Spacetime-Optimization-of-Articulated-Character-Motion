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
// Calculate the gravitational torque term G for the system at the current timestep										//
//======================================================================================================================//

matrix<double> 
Spacetime::computeG_analytic(void)
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();
	double I1  = (1.0/12.0)*m1*l1*l1;
	double I2  = (1.0/12.0)*m1*l2*l2;

	// state dependent variables
	matrix<double> state = getState();
	double theta1 = state(0,0);
	double theta2 = state(1,0);
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	matrix<double> G(DOF*joints.size(), 1);
	G(0,0) = m1*lc1*g*cos(theta1) + m2*g*(lc2*cos(theta1+theta2) + l1*cos(theta1));
	G(1,0) = m2*g*lc2*cos(theta1+theta2);
	return G;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//
/*
matrix<double> 
Spacetime::calculateAngularVelocity_analytic(void) 
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
Spacetime::calculateAngularPosition_analytic(void) 
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
*/
//======================================================================================================================//
// Calculate the mass matrix for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeM_analytic(void)
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();
	double I1  = (1.0/12.0)*m1*l1*l1;
	double I2  = (1.0/12.0)*m1*l2*l2;

	// state dependent variables
	matrix<double> state = getState();
	double theta1 = state(0,0);
	double theta2 = state(1,0);
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	matrix<double> M(DOF*joints.size(), DOF*joints.size());
	M(0,0) = m1*lc1*lc1 + I1 + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*cos(theta2)) + I2;
	M(0,1) = m2*(lc2*lc2 + l1*lc2*cos(theta2)) + I2;
	M(1,0) = m2*(lc2*lc2 + l1*lc2*cos(theta2)) + I2;
	M(1,1) = m2*lc2*lc2 + I2;
	return M;
}

//======================================================================================================================//
// Calculate the centripetal/Coriolis term for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeC_analytic(void)
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();
	double I1  = (1.0/12.0)*m1*l1*l1;
	double I2  = (1.0/12.0)*m1*l2*l2;

	// state dependent variables
	matrix<double> state = getState();
	double theta1 = state(0,0);
	double theta2 = state(1,0);
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	matrix<double> C(DOF*joints.size(), 1);
	double h = m2*l1*lc2*sin(theta2);
	C(0,0) = -h*thetaDot2*thetaDot2 - 2*h*thetaDot1*thetaDot2;
	C(1,0) = h*thetaDot1*thetaDot1;
	return C;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
//======================================================================================================================//

void 
Spacetime::stepPhysics_analytic(matrix<double> MInv, matrix<double> u, matrix<double> C, matrix<double> G)
{
	matrix<double> state = getState();
	matrix<double> stateDot(4,1);

	matrix<double> thetaDot(2,1); 
	thetaDot(0,0) = state(2,0); 
	thetaDot(1,0) = state(3,0);
	
	matrix<double> thetaDotDot = MInv * (u - C - G);
	
	stateDot(0,0) = thetaDot(0,0);
	stateDot(1,0) = thetaDot(1,0);
	stateDot(2,0) = thetaDotDot(0,0);
	stateDot(3,0) = thetaDotDot(1,0);
	
	setState(state + stateDot*deltaT);

	//for (int i = 0; i < dynamic_actors.size(); i++)
	//	dynamic_actors[i]->setLinearVelocity(PxVec3(0,0,0));
}