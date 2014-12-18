//======================================================================================================================//
// SpacetimeOptimization.cpp																							//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//																														//
// This code...																											//
//======================================================================================================================//

#include "Spacetime.h"

bool ANALYTICAL = true;

//======================================================================================================================//
// Define global variables for spacetime optimization																	//
//======================================================================================================================//

//#define T_DEBUG 1  // comment this to suppress textual debugging output
//#define V_DEBUG 1  // comment this to suppress visual  debugging output

PxReal SSDmatrix(matrix<double> A, matrix<double> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.RowNo(); i++) {
		for (int j = 0; j < A.ColNo(); j++) {
			SSD += (A(i,j)-B(i,j))*(A(i,j)-B(i,j));
		}
	}
	return SSD;
}

PxReal SSDvector(std::vector<matrix<double>> A, std::vector<matrix<double>> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.size(); i++)
		SSD += SSDmatrix(A[i], B[i]);
	return SSD;
}

//======================================================================================================================//
// Main optimization calculation loop																					//
//======================================================================================================================//

void
Spacetime::makeInitialGuess(void)
{
	// virtual work calculation yields gravitational torque vector
	matrix<double> G = computeGVector();

	// compute inverse mass matrix by altering rows of input torque
	matrix<double> M_inv = computeInverseMassMatrix(G);
	matrix<double> M(M_inv); M = M.Inv();

	// compute C term of kinematics formula
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

//------------------------------------------------------------------------------------------------------------------//
// perform one iteration of the numerical solver used to compute the optimal input torque sequence					//
//------------------------------------------------------------------------------------------------------------------//

double
Spacetime::IterateOptimization(void)
{	
	// save initial state
	saveState();
	PxReal uDiff;

	// clear the state and costate sequences from the last solver iteration
	GSequence.clear();
	CSequence.clear();
	MSequence.clear();
	MInvSequence.clear();
	stateSequence.clear();

	//----------------------------------------------------------------------------------------------//
	// 1) solve for the state sequence given the current u vector sequence							//
	//----------------------------------------------------------------------------------------------//

	setState(state_0);
	stateSequence.push_back(state_0);
	for (int t = 1; t <= numTimeSteps; t++) 
	{	
		// virtual work calculation yields gravitational torque vector
		matrix<double> G = computeGVector();

		// compute inverse mass matrix by altering rows of input torque
		matrix<double> MInv = computeInverseMassMatrix(G);
		matrix<double> M(MInv); M = (matrix<double>) M.Inv();

		// compute C term of kinematics formula
		matrix<double> C = computeCVector(G, M);
			
		// store computed state matrices
		GSequence.push_back(G);
		CSequence.push_back(C);
		MSequence.push_back(M);
		MInvSequence.push_back(MInv);
		stateSequence.push_back(getState());
		//cout << "state[" << t << "] = \n" << getState() << endl;

		// apply computed input torque vector u and simulate
		applyTorqueVector(uSequence[t-1]);
		stepPhysics();
	}

	//----------------------------------------------------------------------------------------------//
	// 2) simulate lagrangian costate sequence backwards from final state using x, u				//
	//----------------------------------------------------------------------------------------------//
	
	costateSequence.clear();
	matrix<double> lambda_n = state_d - stateSequence[numTimeSteps-1];
	cout << "stateSequence[numTimeSteps-1] = \n" << stateSequence[numTimeSteps-1] << endl;
	cout << "state_d = \n" << state_d << endl;
	cout << "state_d - stateSequence[numTimeSteps-1] = \n" << state_d - stateSequence[numTimeSteps-1] << endl;
	costateSequence.push_back(lambda_n);
	for (int t = 0; t < numTimeSteps; t++) 
	{
		matrix<double> dfdx= compute_dfdx_analytical(numTimeSteps-t-1); 
		matrix<double> lambdaDot;
		if (ANALYTICAL) {
			lambdaDot = (~costateSequence[t])*dfdx;
			lambdaDot = ~lambdaDot;
			cout << "lambda[" << numTimeSteps-t-1 << "] = " << costateSequence[t] << endl;
			cout << "lambdaDot[" << numTimeSteps-t-1 << "] = " << lambdaDot << endl;
		} else { 
			lambdaDot = -(~costateSequence[t])*dfdx;
			lambdaDot = ~lambdaDot;
		}
		costateSequence.push_back(costateSequence[t] - deltaT*lambdaDot);
	} std::reverse(costateSequence.begin(), costateSequence.end());

	//----------------------------------------------------------------------------------------------//
	// 3) update u from constraint formula															//
	//----------------------------------------------------------------------------------------------//

	std::vector<matrix<double>> new_uSequence;
	for (int t = 0; t < numTimeSteps; t++)
		new_uSequence.push_back(-1.0 * ~(~costateSequence[t]*compute_dfdu_analytical(t)));
	uDiff = SSDvector(uSequence, new_uSequence);
	cout << "uDiff = " << uDiff << endl;
		
	uSequence = new_uSequence;

	return uDiff;
}
