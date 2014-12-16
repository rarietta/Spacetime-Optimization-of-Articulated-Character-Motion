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
#define UDIFF_THRESHOLD 0.01

//======================================================================================================================//
// Debug																												//
//======================================================================================================================//
void
Spacetime::debug(void)
{
	//------------------------------------------------------------------------------------------------------------------//
	// compute M, C, G terms from kinematics equation																	//
	// also compute derivatives of M, C, G terms with ADOL-C															//
	//------------------------------------------------------------------------------------------------------------------//

	// virtual work calculation yields gravitational torque vector
	matrix<double> J = buildJacobian();
	matrix<double> G = computeGVector();
	matrix<double> T = ~J*G;

	// compute inverse mass matrix by altering rows of input torque
	matrix<double> M_inv = computeInverseMassMatrix(T);
	matrix<double> M(M_inv); M = (matrix<double>) M.Inv();
	matrix<double> eye = M_inv * M;
	
	// compute C term of kinematics formula
	matrix<double> C = computeCVector(T, M);

	if (!pause) {
		cout << "-----------------------------------------------------------------------------------------------" << endl;
		cout << "time = " << gScene->getTimestamp() << endl;
		cout << endl;
		cout << "J = \n" << J << endl;
		cout << "G = \n" << G << endl;
		cout << "T = \n" << T << endl;
		cout << "M_inv = \n" << M_inv << endl;
		cout << "M = \n" << M << endl;
		cout << "eye test: \n" << eye << endl;
		cout << "C = \n" << C << endl;
	}
}

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
	saveState();
	for (int i = 0; i < numTimeSteps; i++) 
	{
		// virtual work calculation yields gravitational torque vector
		matrix<double> J = buildJacobian();
		matrix<double> G = computeGVector();
		G = -(~J*G);

		// compute inverse mass matrix by altering rows of input torque
		matrix<double> M_inv = computeInverseMassMatrix(G);
		matrix<double> M(M_inv); M = (matrix<double>) M.Inv();

		// compute C term of kinematics formula
		matrix<double> C = computeCVector(G, M);

		double Kp = .0010;
		double Kv = .0010;
		matrix<double> theta = calculateAngularPosition();
		matrix<double> thetad(state_d); thetad.SetSize(DOF*joints.size(), 1);
		matrix<double> thetaDot = calculateAngularVelocity();
		matrix<double> u = C + G + M*(Kp*(thetad - theta) - Kv*thetaDot);
		uSequence.push_back(u);
	}

	for (int i = 0; i < numTimeSteps; i++)
		cout << "u[" << i << "] = \n" << uSequence[i] << endl;

	return;
}

matrix<double>
Spacetime::Optimize(void)
{	
	matrix<double> T;
	return T;

	//------------------------------------------------------------------------------------------------------------------//
	// iteratively solve for optimal input torque sequence																//
	//------------------------------------------------------------------------------------------------------------------//
	/*
	// save initial state
	saveState();

	PxU32 jointCount = gScene->getNbConstraints();
	std::vector<matrix<double>> GSequence;
	std::vector<matrix<double>> CSequence;
	std::vector<matrix<double>> MSequence;
	std::vector<matrix<double>> M_invSequence;
	std::vector<matrix<double>> stateSequence;
	std::vector<matrix<double>> costateSequence;

	PxReal uDiff;
	do 
	{
		// clear the state and costate sequences from the last solver iteration
		GSequence.clear();
		CSequence.clear();
		MSequence.clear();
		M_invSequence.clear();
		stateSequence.clear();
		costateSequence.clear();

		// 1) solve for the state sequence given the current u vector sequence
		saveState();
		stateSequence.push_back(state_0);
		for (int i = 0; i < numTimeSteps-1; i++) 
		{	
			// virtual work calculation yields gravitational torque vector
			matrix<double> J = buildJacobian();
			matrix<double> G = computeGVector();
			matrix<double> T = ~J*F;

			// compute inverse mass matrix by altering rows of input torque
			matrix<double> M_inv = computeInverseMassMatrix(T);
			matrix<double> M(M_inv); M = (matrix<double>) M.Inv();

			// compute C term of kinematics formula
			matrix<double> C = computeCVector(T, M);
			
			// compute current state vector derivative with computed
			// matrix values and the current input torque vector u
			matrix<double> xDot = M_inv * (T + C + G);
			stateSequence.push_back(stateSequence[i] + xDot*deltaT);
			stepPhysics();
		}
		restoreState();

		// 2) simulate lagrangian costate sequence backwards from final state using x, u
		// TODO: initialize lambda vector at end state
		matrix<double> lambda_n;
		costateSequence.insert(costateSequence.begin(), lambda_n);
		for (int i = numTimeSteps-1; i > 0; i--) 
		{
			// lambdaDot = -dL/dX + transpose(lambda)*df/dX
			matrix<double> dLdx = compute_dLdx(i);
			matrix<double> dfdx = compute_dfdx(i);
			matrix<double> lambdaDot = -dLdx + ~costateSequence.front()*dfdx;
			costateSequence.insert(costateSequence.begin(), costateSequence.front()-deltaT*lambdaDot);
		}


		// 3) update u from constraint formula
		std::vector<matrix<double>> new_uSequence;
		for (int i = 0; i < numTimeSteps; i++)
			new_uSequence.push_back(~costateSequence[i]*M_invSequence[i]);
		uDiff = SSDvector(uSequence, new_uSequence);
		uSequence = new_uSequence;

	} while (uDiff > UDIFF_THRESHOLD);
	
	//------------------------------------------------------------------------------------------------------------------//
	// compute optimal pose/position sequence from optimal u															//
	//------------------------------------------------------------------------------------------------------------------//

	// TODO
	matrix<double> Opt(1,1);
	return Opt;
	*/
}
