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
// Define global variables for spacetime optimization																	//
//======================================================================================================================//

//#define T_DEBUG 1  // comment this to suppress textual debugging output
//#define V_DEBUG 1  // comment this to suppress visual  debugging output

PxReal SSDmatrix(matrix<PxReal> A, matrix<PxReal> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.RowNo(); i++) {
		for (int j = 0; j < A.ColNo(); j++) {
			SSD += (A(i,j)-B(i,j))*(A(i,j)-B(i,j));
		}
	}
	return SSD;
}

PxReal SSDvector(std::vector<matrix<PxReal>> A, std::vector<matrix<PxReal>> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.size(); i++)
		SSD += SSDmatrix(A[i], B[i]);
	return SSD;
}

//======================================================================================================================//
// Main optimization calculation loop																					//
//======================================================================================================================//

matrix<PxReal>
Spacetime::Optimize(void)
{	
	//------------------------------------------------------------------------------------------------------------------//
	// compute M, C, G terms from kinematics equation																	//
	// also compute derivatives of M, C, G terms with ADOL-C															//
	//------------------------------------------------------------------------------------------------------------------//

	// virtual work calculation yields gravitational torque vector
	matrix<PxReal> J = buildJacobian();
	matrix<PxReal> F = buildForceVector();
	matrix<PxReal> T = ~J*F;

	// compute inverse mass matrix by altering rows of input torque
	matrix<PxReal> M_inv = computeInverseMassMatrix(T);
	matrix<PxReal> M(M_inv); //M = (matrix<PxReal>) M.Inv();

	// compute C term of kinematics formula
	matrix<PxReal> C = computeCVector(T, M);

	#ifdef T_DEBUG
		cout << "-----------------------------------------------------------------------------------------------" << endl;
		cout << "time = " << gScene->getTimestamp() << endl;
		cout << endl;
		cout << "J = \n" << J << endl;
		cout << "F = \n" << F << endl;
		cout << "T = \n" << T << endl;
		cout << "M_inv = \n" << M_inv << endl;
		cout << "M = \n" << M << endl;
		cout << "C = \n" << C << endl;
		switchPause();
	#endif

	//------------------------------------------------------------------------------------------------------------------//
	// u = initial estimate on input torque sequence w/ PD controller													//
	//------------------------------------------------------------------------------------------------------------------//

	saveState();
	std::vector<matrix<PxReal>> uSequence;
	for (int i = 0; i < numTimeSteps; i++) 
	{
		// virtual work calculation yields gravitational torque vector
		matrix<PxReal> J = buildJacobian();
		matrix<PxReal> G = buildForceVector();
		matrix<PxReal> T = ~J*F;

		// compute inverse mass matrix by altering rows of input torque
		matrix<PxReal> M_inv = computeInverseMassMatrix(T);
		matrix<PxReal> M(M_inv); M = (matrix<PxReal>) M.Inv();

		// compute C term of kinematics formula
		matrix<PxReal> C = computeCVector(T, M);

		PxReal Kp = 10;
		PxReal Kv = 10;
		matrix<PxReal> theta = calculateAngularPosition();
		matrix<PxReal> thetad(state_d); thetad.SetSize(DOF*joints.size(), 1);
		matrix<PxReal> thetaDot = calculateAngularVelocity();
		matrix<PxReal> u = C + G + M*(Kp*(thetad - theta) - Kv*thetaDot);
		uSequence.push_back(u);

		applyTorqueVector(u);
		stepPhysics();
	}
	restoreState();

	//------------------------------------------------------------------------------------------------------------------//
	// iteratively solve for optimal input torque sequence																//
	//------------------------------------------------------------------------------------------------------------------//
		
	// save initial state
	saveState();

	PxU32 jointCount = gScene->getNbConstraints();
	std::vector<matrix<PxReal>> GSequence;
	std::vector<matrix<PxReal>> CSequence;
	std::vector<matrix<PxReal>> MSequence;
	std::vector<matrix<PxReal>> M_invSequence;
	std::vector<matrix<PxReal>> stateSequence;
	std::vector<matrix<PxReal>> costateSequence;

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
			matrix<PxReal> J = buildJacobian();
			matrix<PxReal> G = buildForceVector();
			matrix<PxReal> T = ~J*F;

			// compute inverse mass matrix by altering rows of input torque
			matrix<PxReal> M_inv = computeInverseMassMatrix(T);
			matrix<PxReal> M(M_inv); M = (matrix<PxReal>) M.Inv();

			// compute C term of kinematics formula
			matrix<PxReal> C = computeCVector(T, M);
			
			// compute current state vector derivative with computed
			// matrix values and the current input torque vector u
			matrix<PxReal> xDot = M_inv * (T + C + G);
			stateSequence.push_back(stateSequence[i] + xDot*deltaT);
			stepPhysics();
		}
		restoreState();

		// 2) simulate lagrangian costate sequence backwards from final state using x, u
		// TODO: initialize lambda vector at end state
		matrix<PxReal> lambda_n;
		costateSequence.insert(costateSequence.begin(), lambda_n);
		for (int i = numTimeSteps-1; i > 0; i--) 
		{
			// lambdaDot = -dL/dX + transpose(lambda)*df/dX
			matrix<PxReal> dfdx = compute_dfdx(i);
			matrix<PxReal> lambdaDot = ~costateSequence.front()*dfdx;
			costateSequence.insert(costateSequence.begin(), costateSequence.front()-deltaT*lambdaDot);
		}


		// 3) update u from constraint formula
		std::vector<matrix<PxReal>> new_uSequence;
		for (int i = 0; i < numTimeSteps; i++)
			new_uSequence.push_back(~costateSequence[i]*M_invSequence[i]);
		uDiff = SSDvector(uSequence, new_uSequence);
		uSequence = new_uSequence;

	} while (uDiff > UDIFF_THRESHOLD);
	
	//------------------------------------------------------------------------------------------------------------------//
	// compute optimal pose/position sequence from optimal u															//
	//------------------------------------------------------------------------------------------------------------------//

	// TODO
	matrix<PxReal> Opt(1,1);
	return Opt;
}
