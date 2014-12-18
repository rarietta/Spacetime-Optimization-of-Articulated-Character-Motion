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
bool ANALYTICAL = true;

//==================================================================================================//
// Function:																						//
//		Estimate input torque vector sequence before optimization using simple PD controller		//
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
	matrix<double> MInv = computeInverseMassMatrix(G);
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
//		Perform one iteration of the numerical solver used to compute the							// 
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

	GSequence.clear();
	CSequence.clear();
	MSequence.clear();
	MInvSequence.clear();
	stateSequence.clear();
	costateSequence.clear();

	//----------------------------------------------------------------------------------------------//
	// solve forward for the state sequence given the current u vector sequence						//
	//----------------------------------------------------------------------------------------------//

	setState(state_0);
	stateSequence.push_back(state_0);
	for (int t = 1; t <= numTimeSteps; t++) 
	{	
		// compute and store G, C, M, and MInv state matrices
		matrix<double> G = computeGVector();
		matrix<double> MInv = computeInverseMassMatrix(G);
		matrix<double> M = !MInv;
		matrix<double> C = computeCVector(G, M);
		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);
		stateSequence.push_back(getState());

		// apply computed input torque vector u and simulate one step
		applyTorqueVector(uSequence[t-1]);
		stepPhysics();
	}

	//----------------------------------------------------------------------------------------------//
	// simulate lagrangian costate sequence backwards from final state using x, u					//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> lambda_N = state_d - stateSequence[numTimeSteps-1];
	costateSequence.push_back(lambda_N);
	for (int t = 0; t < numTimeSteps; t++) {
		matrix<double> dfdx;
		if (ANALYTICAL) dfdx = compute_dfdx_analytical(numTimeSteps-t-1); 
		else			dfdx = compute_dfdx(numTimeSteps-t-1);
		matrix<double> lambdaDot = ~((~costateSequence[t])*dfdx);
		costateSequence.push_back(costateSequence[t] - deltaT*lambdaDot);
	} std::reverse(costateSequence.begin(), costateSequence.end());

	//----------------------------------------------------------------------------------------------//
	// update u from constraint formula																//
	//----------------------------------------------------------------------------------------------//
	
	std::vector<matrix<double>> new_uSequence;
	for (int t = 0; t < numTimeSteps; t++)
		new_uSequence.push_back(-1.0 * ~(~costateSequence[t]*compute_dfdu_analytical(t)));
	PxReal uDiff = SSDvector(uSequence, new_uSequence);
	uSequence = new_uSequence;
	return uDiff;
}