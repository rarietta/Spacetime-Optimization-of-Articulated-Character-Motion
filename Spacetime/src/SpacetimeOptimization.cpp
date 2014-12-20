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
Spacetime::makeInitialGuess(void)
{
	/// compute G, C, M, and MInv state matrices
	matrix<double> G, C, M, MInv;
	if (ANALYTIC) {
		G = computeG_analytic();
		M = computeM_analytic();
		C = computeC_analytic();
		MInv = !M;
	} else {
		G = computeG_numeric();
		MInv = computeMInv_numeric(G); 
		M = !M;
		C = computeC_numeric(G,M);
	}

	// compute torque with PD controller
	double Kp = 1.5, Kv = 1.5;
	matrix<double> theta = calculateAngularPosition();
	matrix<double> thetad(state_d); thetad.SetSize(DOF*joints.size(), 1);
	matrix<double> thetaDot = calculateAngularVelocity();
	matrix<double> u = C + G + M*(Kp*(thetad - theta) - Kv*thetaDot);
	uSequence.push_back(u);

	// apply torque and advance system
	if (ANALYTIC) stepPhysics_analytic(MInv,u,C,G);
	else		  stepPhysics_numeric(u);
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

	for (int t = 0; t < numTimeSteps; t++) 
	{	
		// store current state
		stateSequence.push_back(getState());

		// compute G, C, M, and MInv state matrices
		matrix<double> G, C, M, MInv;
		if (ANALYTIC) {
			G = computeG_analytic();
			M = computeM_analytic();
			C = computeC_analytic();
			MInv = !M;
		} else {
			G = computeG_numeric();
			MInv = computeMInv_numeric(G);
			M = !MInv;
			C = computeC_numeric(G,!MInv);
		}

		// store G, C, M, and MInv state matrices
		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);
		
		// apply torque and advance system
		if (ANALYTIC) stepPhysics_analytic(MInv,uSequence[t],C,G);
		else		  stepPhysics_numeric(uSequence[t]);
	}

	//----------------------------------------------------------------------------------------------//
	// simulate lagrangian costate sequence backwards from final state using x, u					//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> lambdaDot, dfdx, dLdx;
	matrix<double> lambda_N = state_d - stateSequence[numTimeSteps-1];
	costateSequence.push_back(lambda_N);
	for (int t = 0; t < numTimeSteps; t++) {
		if (ANALYTIC) {
			dLdx = compute_dLdx_analytic(numTimeSteps-t-1);
			dfdx = compute_dfdx_analytic(numTimeSteps-t-1); 
		} else {
			dLdx = compute_dLdx_numeric(numTimeSteps-t-1);
			dfdx = compute_dfdx_numeric(numTimeSteps-t-1);
		} lambdaDot = ~(-dLdx + (~costateSequence[t])*dfdx);
		costateSequence.push_back(costateSequence[t] + deltaT*lambdaDot);
	} std::reverse(costateSequence.begin(), costateSequence.end());

	//----------------------------------------------------------------------------------------------//
	// update u from constraint formula																//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> dfdu, u;
	std::vector<matrix<double>> new_uSequence;
	for (int t = 0; t < numTimeSteps; t++) {
		if (ANALYTIC) dfdu = compute_dfdu_analytic(t);
		else		  dfdu = compute_dfdu_numeric(t);
		u = ~(~costateSequence[t]*dfdu);
		new_uSequence.push_back(u);
		cout << "new u = \n" << u << endl;
	} 
	PxReal uDiff = SSDvector(uSequence, new_uSequence);
	uSequence.clear(); uSequence = new_uSequence;
	return uDiff;
}