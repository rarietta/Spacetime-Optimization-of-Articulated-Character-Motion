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
Spacetime::makeInitialGuess_discrete(void)
{
	/// compute G, C, M, and MInv state matrices
	matrix<double> G, C, M, MInv;
	G = computeG_discrete();
	M = computeM_discrete();
	C = computeC_discrete();
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
	stepPhysics_discrete(u);
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
Spacetime::IterateOptimization_discrete(void)
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
	// compute the G, C, and M matrices for each timestep before the backwards sweep				//
	//----------------------------------------------------------------------------------------------//

	stateSequence.push_back(state_0);
	for (int t = 0; t < numTimeSteps; t++) 
	{	
		// compute G, C, M, and MInv state matrices
		matrix<double> G = computeG_discrete();
		matrix<double> M = computeM_discrete();
		matrix<double> C = computeC_discrete();
		matrix<double> MInv = !M;
		
		// store G, C, M, and MInv state matrices
		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);
		
		// apply torque and advance system
		if (ANALYTIC) stepPhysics_analytic(uSequence[t]);
		else		  stepPhysics_numeric (uSequence[t]);
		
		// store current state
		stateSequence.push_back(getState());
	}

	//----------------------------------------------------------------------------------------------//
	// backwards sweep																				//
	//----------------------------------------------------------------------------------------------//

	// clear values from previous pass
	kSequence.clear();
	KSequence.clear();
	valueSequence.clear();

	// declare V derivative values
	matrix<double> Vx;
	matrix<double> Vxx;
	/*
	// backward pass
	double final_error = SSDmatrix(state_d, stateSequence[numTimeSteps-1]);
	valueSequence.push_back(final_error);
	for (int t = numTimeSteps-1; t >= 0; t--) {
		// compute value V = cost-to-go function = 
		// sum of u^2 values for all successive timesteps + final error
		matrix<double> cost_t = ~uSequence[t]*uSequence[t];
		valueSequence.push_back(cost_t(0,0) + valueSequence[t]);

		// compute first derivative values at current timestep
		matrix<double> Lu = compute_Lu(t);
		matrix<double> Lx = compute_Lx(t);
		matrix<double> Fx = compute_Fx(t);
		matrix<double> Fu = compute_Fu(t);

		// compute second derivative values at current timestep
		matrix<double> Luu = compute_Luu(t);
		matrix<double> Lxx = compute_Lxx(t);
		matrix<double> Lux = compute_Lux(t);
		matrix<double> Lxu = compute_Lxu(t);
		std::vector<matrix<double>> Fxx = compute_Fxx(t);
		std::vector<matrix<double>> Fux = compute_Fux(t);
		std::vector<matrix<double>> Fxu = compute_Fxu(t);
		std::vector<matrix<double>> Fuu = compute_Fuu(t);

		// compute expansion coefficients for current timestep
		matrix<double> Qu = Lx + ~Fx*Vx;
		matrix<double> Qx = Lu + ~Fu*Vx;
		matrix<double> Qxx = Lxx + ~Fx*Vxx*Fx + Vx*Fxx;
		matrix<double> Quu = Luu + ~Fu*Vxx*Fu + Vx*Fuu;
		matrix<double> Qux = Lux + ~Fu*Vxx*Fx + Vx*Fux;
		matrix<double> Qxu = Lxu + ~Fx*Vxx*Fu + Vx*Fxu;

		// compute k and K terms for current timestep
		kSequence.push_back(-(!Quu)*Qu);
		KSequence.push_back(-(!Quu)*Qux);

		// compute Vx and Vxx terms for current timestep
		Vx  = Qx  - Qu*!Quu*Qux;
		Vxx = Qxx - Qxu*!Quu*Qux;
		
	}

	// reverse sequences for forward pass
	std::reverse(kSequence.begin(), KSequence.end());
	std::reverse(KSequence.begin(), KSequence.end());
	std::reverse(valueSequence.begin(), valueSequence.end());

	// forward pass
	std::vector<matrix<double>> xHatSequence;
	std::vector<matrix<double>> uHatSequence;
	xHatSequence.push_back(state_0);
	for (int t = 0; t < numTimeSteps; t++) {

		// compute uHat
		matrix<double> uHat = uSequence[t] + kSequence[t] + KSequence[t]*(xHatSequence[t] - stateSequence[t]);
		uHatSequence.push_back(uHat);
		
		// compute next xHat
		matrix<double> xHat = F(xHatSequence[t], uHat);
		xHatSequence.push_back(xHat);
	}

	double uDiff = SSDvector(uSequence, uHatSequence);
	stateSequence = xHatSequence;
	uSequence = uHatSequence;
	return uDiff;
	*/
	return 0.5;
}