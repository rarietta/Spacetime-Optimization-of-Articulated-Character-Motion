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
// Calculate the gravitational torque term G for the system at the current timestep										//
//======================================================================================================================//

matrix<double> 
Spacetime::computeG_discrete(void)
{
	// system dependent variables
	double g   = abs(gScene->getGravity().y);
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

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
// Calculate the mass matrix for the system at the current state
//======================================================================================================================//

matrix<double> 
Spacetime::computeM_discrete(void)
{
	// system dependent variables
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
Spacetime::computeC_discrete(void)
{
	// system dependent variables
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = getState();
	double theta1 = state(0,0);
	double theta2 = state(1,0);
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	matrix<double> C(DOF*joints.size(), 1);
	double h = m2*l1*lc2*sin(theta2);
	C(0,0) = -h * (thetaDot2*thetaDot2 + 2*thetaDot1*thetaDot2);
	C(1,0) = h*thetaDot1*thetaDot1;
	return C;
}

//======================================================================================================================//
// Simulate the physics engine forward by one timestep																	//
// Uses RK2 integration method																							//
//======================================================================================================================//

void 
Spacetime::stepPhysics_discrete(matrix<double> u)
{
	matrix<double> state_1 = getState();
	matrix<double> C_1 = computeC_discrete();
	matrix<double> G_1 = computeG_discrete();
	matrix<double> MInv_1 = !computeM_discrete();

	matrix<double> damping = G_1;
	damping(0,0) *= abs(state_1(2,0));
	damping(1,0) *= abs(state_1(3,0));

	matrix<double> thetaDotDot_1 =  (MInv_1 * (u - C_1 - G_1));
	
	matrix<double> stateDot_1(4,1);
	stateDot_1(0,0) = state_1(2,0);
	stateDot_1(1,0) = state_1(3,0);
	stateDot_1(2,0) = thetaDotDot_1(0,0);
	stateDot_1(3,0) = thetaDotDot_1(1,0);
	
	setState(state_1 + stateDot_1*deltaT/2.0);
	
	matrix<double> state_2 = getState();
	matrix<double> C_2 = computeC_discrete();
	matrix<double> G_2 = computeG_discrete();
	matrix<double> MInv_2 = !computeM_discrete();
	
	damping = G_2;
	damping(0,0) *= abs(state_2(2,0));
	damping(1,0) *= abs(state_2(3,0));

	matrix<double> thetaDotDot_2 = (MInv_2 * (u - C_2 - G_2));
	
	matrix<double> stateDot_2(4,1);
	stateDot_2(0,0) = state_2(2,0);
	stateDot_2(1,0) = state_2(3,0);
	stateDot_2(2,0) = thetaDotDot_2(0,0);
	stateDot_2(3,0) = thetaDotDot_2(1,0);
	
	setState(state_1 + stateDot_2*deltaT);
}

matrix<double>
Spacetime::F(matrix<double> x, matrix<double> u) {
	
	matrix<double> C = computeC_discrete();
	matrix<double> G = computeG_discrete();
	matrix<double> MInv = !computeM_discrete();

	matrix<double> thetaDotDot =  (MInv * (u - C - G));
	
	matrix<double> xDot(4,1);
	xDot(0,0) = x(2,0);
	xDot(1,0) = x(3,0);
	xDot(2,0) = thetaDotDot(0,0);
	xDot(3,0) = thetaDotDot(1,0);

	return x + deltaT * xDot;
}