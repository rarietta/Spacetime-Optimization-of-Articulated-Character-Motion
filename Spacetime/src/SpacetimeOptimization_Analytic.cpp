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
Spacetime::makeInitialGuess_analytic(void)
{
	/// compute G, C, M, and MInv state matrices
	matrix<double> G, C, M, MInv;
	G = computeG_analytic();
	M = computeM_analytic();
	C = computeC_analytic();
	MInv = !M;

	// compute torque with PD controller
	double Kp = 1.5, Kv = 1.5;
	matrix<double> state = getState();
	
	matrix<double> theta(2,1);
	double theta1 = state(0,0);
	double theta2 = state(1,0);
	theta(0,0) = theta1;
	theta(1,0) = theta2;
	
	matrix<double> thetad(state_d); 
	thetad.SetSize(DOF*joints.size(), 1);
	
	matrix<double> thetaDot(2,1);
	thetaDot(0,0) = state(2,0);
	thetaDot(1,0) = state(3,0);
	
	matrix<double> u = C + G + M*(Kp*(thetad - theta) - Kv*thetaDot);
	uSequence.push_back(u);

	// apply torque and advance system
	stepPhysics_analytic(u);
}

//==================================================================================================//
// Function:																						//
//		Perform one iteration of the numeric solver used to compute the								// 
//		optimal input torque sequence																//
// Returns:																							//
//		SSD between newly computed input torque vector sequence and									// 
//		previously computed sequence for use in convergence calculations							//
// Called from:																						//
//		Currently called from render loop															//
//==================================================================================================//

double
Spacetime::IterateOptimization_analytic(void)
{	
	//return Synthesis();

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
		// compute G, C, M, and MInv state matrices
		matrix<double> G, C, M, MInv;
		G = computeG_analytic();
		M = computeM_analytic();
		C = computeC_analytic();
		MInv = !M;
		
		// store G, C, M, and MInv state matrices
		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);
		
		// apply torque and advance system
		stepPhysics_analytic(uSequence[t]);
		
		// store current state
		stateSequence.push_back(getState());
	}

	//----------------------------------------------------------------------------------------------//
	// simulate lagrangian costate sequence backwards from final state using x, u					//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> lambdaDot, lambdaDotTranspose, dfdx, dLdx; 
	matrix<double> transpose_dfdu = ~compute_dfdu_analytic(numTimeSteps-1);
	matrix<double> inverse_transpose_dfdu = ~transpose_dfdu * !(transpose_dfdu * ~transpose_dfdu);
	matrix<double> lambda_N = -inverse_transpose_dfdu * uSequence[numTimeSteps-1];
	costateSequence.push_back(lambda_N);
	for (int t = 0; t < numTimeSteps; t++) {
		dLdx = compute_dLdx_analytic(numTimeSteps-t-1);
		dfdx = compute_dfdx_analytic(numTimeSteps-t-1); 
		lambdaDotTranspose = -dLdx + ~costateSequence[t]*dfdx;
		lambdaDot = ~lambdaDotTranspose;
		costateSequence.push_back(costateSequence[t] - deltaT*lambdaDot);
	} std::reverse(costateSequence.begin(), costateSequence.end());

	//----------------------------------------------------------------------------------------------//
	// update u from constraint formula																//
	//----------------------------------------------------------------------------------------------//
	
	setState(state_0);
	matrix<double> dfdu, u;
	std::vector<matrix<double>> new_uSequence;
	for (int t = 0; t < numTimeSteps; t++) {
		dfdu = compute_dfdu_analytic();
		u = -(~(~(-costateSequence[t])*dfdu));
		new_uSequence.push_back(u);
		stepPhysics_analytic(u);
	} 
	PxReal uDiff = SSDvector(uSequence, new_uSequence);
	uSequence.clear(); uSequence = new_uSequence;
	return uDiff;
}