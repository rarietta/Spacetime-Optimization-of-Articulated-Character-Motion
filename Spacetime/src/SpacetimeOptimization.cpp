//==================================================================================================//
//																									//
// SpacetimeOptimization.cpp																		//
// (c) Ricky Arietta 2014																			//
// CIS 599 Masters Independent Study																//
// University of Pennsylvania																		//
// Computer Graphics and Gaming Technology Program													//
//																									//
//==================================================================================================//

#include "Spacetime.h"
bool ANALYTIC = true;

//==================================================================================================//
// Function:																						//
//		Estimate next input torque vector sequence before optimization using						//
//		simple PD controller																		//
// Returns:																							//
//		Void, input torque sequence is stored in spacetime instance									//
// Called from:																						//
//		Currently called from render loop															//
//==================================================================================================//

void
Spacetime::makeInitialGuess(void)
{
	// compute G, C, M, and MInv state matrices
	matrix<double> G = computeGVector();
	matrix<double> MInv = computeMInv(G);
	matrix<double> M = !MInv;
	matrix<double> C = computeCVector(G, M);

	// compute torque with PD controller
	double Kp = 1.5, Kv = 1.5;
	matrix<double> theta = calculateAngularPosition();
	matrix<double> thetad(state_d); thetad.SetSize(DOF*joints.size(), 1);
	matrix<double> thetaDot = calculateAngularVelocity();
	matrix<double> u = C + G + M*(Kp*(thetad - theta) - Kv*thetaDot);
	uSequence.push_back(u);
	applyTorqueVector(u);
	stepPhysics();
}

//==================================================================================================//
// Function:																						//
//		Perform one iteration of the numeric solver used to compute the							// 
//		optimal input torque sequence																//
// Returns:																							//
//		SSD between newly computed input torque vector sequence and									// 
//		previously computed sequence for use in convergence calculations							//
// Called from:																						//
//		Currently called from render loop															//
//==================================================================================================//

double
Spacetime::IterateOptimization(void)
{	
	//----------------------------------------------------------------------------------------------//
	// clear the state and costate sequences from the last solver iteration							//
	//----------------------------------------------------------------------------------------------//

	setState(state_0);
	GSequence.clear();
	CSequence.clear();
	MSequence.clear();
	MInvSequence.clear();
	stateSequence.clear();
	costateSequence.clear();

	//----------------------------------------------------------------------------------------------//
	// solve forward for the state sequence given the current u vector sequence						//
	//----------------------------------------------------------------------------------------------//

	stateSequence.push_back(state_0);
	for (int t = 0; t < numTimeSteps; t++) 
	{	
		// system dependent variables
		PxReal g   = gScene->getGravity().y;
		PxReal m1  = dynamic_actors[1]->getMass();
		PxReal m2  = dynamic_actors[2]->getMass();
		PxReal lc1 = joint_local_positions[0].magnitude();
		PxReal lc2 = joint_local_positions[1].magnitude();
		PxReal l1  = 2.0 * joint_local_positions[0].magnitude();
		PxReal l2  = 2.0 * joint_local_positions[1].magnitude();
		PxReal I1 = (1.0/12.0)*m1*l1*l1;
		PxReal I2 = (1.0/12.0)*m2*l2*l2;

		// state dependent variables
		matrix<double> state = getState();
		PxReal theta1 = state(0,0) + PxPi/2.0f;
		PxReal theta2 = state(1,0) + PxPi/2.0f - theta1;
		PxReal thetaDot1 = state(2,0);
		PxReal thetaDot2 = state(3,0);

		// compute and store G, C, M, and MInv state matrices
		matrix<double> G = computeGVector();					// THIS MATCHES THE ANALYTIC METHOD - GOOD
			
		//matrix<double> MInv = computeMInv(G);					// THIS DOES NOT MATCH THE ANALYTIC METHOD
		//matrix<double> M = !MInv;								// THIS DOES NOT MATCH THE ANALYTIC METHOD
		matrix<double> M(2,2);
		M(0,0) = m1*lc1*lc1 + I1 + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*cos(theta2)) + I2;
		M(0,1) = m2*(lc2*lc2 + l1*lc2*cos(theta2)) + I2;
		M(1,0) = m2*(lc2*lc2 + l1*lc2*cos(theta2)) + I2;
		M(1,1) = m2*lc2*lc2 + I2;
		matrix<double> MInv = !M;

		//matrix<double> C = computeCVector(G, M);				// THIS DOES NOT MATCH THE ANALYTIC METHOD
		double h = m2*l1*lc2*sin(theta2);
		matrix<double> C(2,1);
		C(0,0) = -h*thetaDot2*thetaDot2 - 2*h*thetaDot1*thetaDot2;
		C(1,0) = h*thetaDot1*thetaDot1;

		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);

		// apply computed input torque vector u and simulate one step
		applyTorqueVector(uSequence[t]);
		stepPhysics();
		stateSequence.push_back(getState());
	}

	//----------------------------------------------------------------------------------------------//
	// simulate lagrangian costate sequence backwards from final state using x, u					//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> dfdx;
	matrix<double> lambda_N = state_d - stateSequence[numTimeSteps-1];
	costateSequence.push_back(lambda_N);
	for (int t = 0; t < numTimeSteps; t++) {
		if (ANALYTIC) dfdx = compute_dfdx_analytic(numTimeSteps-t-1); 
		else			dfdx = compute_dfdx_numeric (numTimeSteps-t-1);
		matrix<double> lambdaDot = ~((~costateSequence[t])*dfdx);
		costateSequence.push_back(costateSequence[t] - deltaT*lambdaDot);
	} std::reverse(costateSequence.begin(), costateSequence.end());

	//----------------------------------------------------------------------------------------------//
	// update u from constraint formula																//
	//----------------------------------------------------------------------------------------------//
	
	std::vector<matrix<double>> new_uSequence;
	for (int t = 0; t < numTimeSteps; t++) {
		if (ANALYTIC) new_uSequence.push_back(-1.0 * ~(~costateSequence[t]*compute_dfdu_analytic(t)));
		else			new_uSequence.push_back(-1.0 * ~(~costateSequence[t]*compute_dfdu_numeric(t)));
	} PxReal uDiff = SSDvector(uSequence, new_uSequence);
	uSequence.clear(); uSequence = new_uSequence;
	return uDiff;
}