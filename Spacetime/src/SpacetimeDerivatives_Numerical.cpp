//======================================================================================================================//
// SpacetimeDerivatives.cpp																								//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//======================================================================================================================//

#include "Spacetime.h"

//======================================================================================================================//
// numerical G derivatives																								//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dG_dtheta1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dG_dtheta1(2,1);
	
	// return numerical derivative
	return dG_dtheta1;
}

matrix<double>  
Spacetime::compute_dG_dtheta2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dG_dtheta2(2,1);
	
	// return numerical derivative
	return dG_dtheta2;
}

matrix<double>  
Spacetime::compute_dG_dthetaDot1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dG_dthetaDot1(2,1);
	
	// return numerical derivative
	return dG_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dG_dthetaDot2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dG_dthetaDot2(2,1);
	
	// return numerical derivative
	return dG_dthetaDot2;
}

//======================================================================================================================//
// numerical C derivatives																								//
//======================================================================================================================//

matrix<double>  
Spacetime::compute_dC_dtheta1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dC_dtheta1(2,1);
	
	// return numerical derivative
	return dC_dtheta1;
}

matrix<double>  
Spacetime::compute_dC_dtheta2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dC_dtheta2(2,1);
	
	// return numerical derivative
	return dC_dtheta2;
}

matrix<double>  
Spacetime::compute_dC_dthetaDot1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dC_dthetaDot1(2,1);
	
	// return numerical 
	return dC_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dC_dthetaDot2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dC_dthetaDot2(2,1);

	// return numerical derivative
	return dC_dthetaDot2;
}

//======================================================================================================================//
// numerical MInv derivatives																								//
//======================================================================================================================//

matrix<double>  
Spacetime::compute_dMInv_dtheta1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dMInv_dtheta1(2,2);
	
	// return numerical derivative
	return dMInv_dtheta1;
}

matrix<double>  
Spacetime::compute_dMInv_dtheta2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dM_dtheta2(2,2);
	
	// return numerical derivative
	return -MInvSequence[t] * dM_dtheta2 * MInvSequence[t];
}

matrix<double>  
Spacetime::compute_dMInv_dthetaDot1_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dMInv_dthetaDot1(2,2);
	
	// return numerical derivative
	return dMInv_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dMInv_dthetaDot2_numerical(PxU32 t) 
{
	// TODO 

	// compute values
	matrix<double> dMInv_dthetaDot2(2,2);
	
	// return numerical derivative
	return dMInv_dthetaDot2;
}

//======================================================================================================================//
// Calculate the derivative of the cost function L with respect to the state vector X									//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dLdx_numerical(PxU32 t)
{
	// REPLACE WITH ADOL-C?
	matrix<double> dLdx(DOF*joints.size(), 1);
	for (int i = 0; i < DOF*joints.size(); i++)
		dLdx(i,0) = 0.0;
	return dLdx;
}

//======================================================================================================================//
// Calculate the derivative of the cost function L with respect to the input torque vector u							//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dLdu_numerical(PxU32 t)
{
	// REPLACE WITH ADOL-C?
	return uSequence[t];
}

//======================================================================================================================//
// Calculate the derivative of the dynamics equation f with respect to the state vector X								//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dfdx_numerical(PxU32 t)
{
	//----------------------------------------------------------------------------------------------//
	// compute dG_dX, the derivative of the gravitational torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	// compute gradients numerically
	matrix<double> dG_dtheta1    = compute_dG_dtheta1_numerical(t);
	matrix<double> dG_dtheta2    = compute_dG_dtheta2_numerical(t);
	matrix<double> dG_dthetaDot1 = compute_dG_dthetaDot1_numerical(t);
	matrix<double> dG_dthetaDot2 = compute_dG_dthetaDot2_numerical(t);

	std::vector<matrix<double>> dG_dX;
	dG_dX.push_back(dG_dtheta1);
	dG_dX.push_back(dG_dtheta2);
	dG_dX.push_back(dG_dthetaDot1);
	dG_dX.push_back(dG_dthetaDot2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dC_dX, the derivative of the centripetal torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	// compute gradients numerically
	matrix<double> dC_dtheta1    = compute_dC_dtheta1_numerical(t);
	matrix<double> dC_dtheta2    = compute_dC_dtheta2_numerical(t);
	matrix<double> dC_dthetaDot1 = compute_dC_dthetaDot1_numerical(t);
	matrix<double> dC_dthetaDot2 = compute_dC_dthetaDot2_numerical(t);

	std::vector<matrix<double>> dC_dX;
	dC_dX.push_back(dC_dtheta1);
	dC_dX.push_back(dC_dtheta2);
	dC_dX.push_back(dC_dthetaDot1);
	dC_dX.push_back(dC_dthetaDot2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dMinv_dx, the derivative of the inverse mass matrix w.r.t. to the state vector		//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	// compute gradients numerically
	matrix<double> dMInv_dtheta1    = compute_dMInv_dtheta1_numerical(t);
	matrix<double> dMInv_dtheta2    = compute_dMInv_dtheta2_numerical(t);
	matrix<double> dMInv_dthetaDot1 = compute_dMInv_dthetaDot1_numerical(t);
	matrix<double> dMInv_dthetaDot2 = compute_dMInv_dthetaDot2_numerical(t);

	std::vector<matrix<double>> dMInv_dX;
	dMInv_dX.push_back(dMInv_dtheta1);
	dMInv_dX.push_back(dMInv_dtheta2);
	dMInv_dX.push_back(dMInv_dthetaDot1);
	dMInv_dX.push_back(dMInv_dthetaDot2);

	//----------------------------------------------------------------------------------------------//
	// Perform calculation of df2/dx = dMInv_dX*(u+C+G) - Minv*(dC_dx+dG_dx);						//
	//----------------------------------------------------------------------------------------------//

	matrix<double> u = uSequence[t];
	matrix<double> C = CSequence[t];
	matrix<double> G = GSequence[t];
	matrix<double> MInv = MInvSequence[t];
	
	// dMInv_dX*(u+C+G)
	matrix<double> term1(DOF*joints.size(), DOF*joints.size()*2);
	matrix<double> sum = (u - C - G);
	for (int i = 0; i < DOF*joints.size()*2; i++) {
		matrix<double> column_i = dMInv_dX[i] * sum;
		for (int j = 0; j < DOF*joints.size(); j++) {
			term1(j,i) = column_i(j,0);
		}
	}

	// Minv*(dC_dx+dG_dx);
	matrix<double> term2(DOF*joints.size(), DOF*joints.size()*2);
	for (int i = 0; i < DOF*joints.size()*2; i++) {
		matrix<double> sum = dC_dX[i] + dG_dX[i];
		matrix<double> column_i = MInv * sum;
		for (int j = 0; j < DOF*joints.size(); j++) {
			term2(j,i) = column_i(j,0);
		}
	}
	matrix<double> dfdx_lower = term1 - term2;
	
	//----------------------------------------------------------------------------------------------//
	//																								//
	// dfdx = |df1/dx| = |        0         |         I          |									//
	//		  |df2/dx|	 | dMInv_dX*(u+C+G) - Minv*(dC_dx+dG_dx) |									//
	//																								//
	//----------------------------------------------------------------------------------------------//

	matrix<double> dfdx(DOF*joints.size()*2, DOF*joints.size()*2);
	for (int i = 0; i < DOF*joints.size(); i++) {
		for (int j = 0; j < DOF*joints.size(); j++) {
			dfdx(i,j) = 0;
			if (i == j) dfdx(i,j+DOF*joints.size()) = 1;
			else		dfdx(i,j+DOF*joints.size()) = 0;
		}
	} for (int i = DOF*joints.size(); i < DOF*joints.size()*2; i++) {
		for (int j = 0; j < DOF*joints.size()*2; j++) {
			dfdx(i,j) = dfdx_lower(i-DOF*joints.size(), j);
		}
	}
	
	cout << "dfdx = \n" << dfdx << endl;
	return dfdx;
	
	/*
	
	cout << "------------------------------------------------------\n" << endl;
	cout << "dG_dtheta1 = \n" << dG_dtheta1 << endl;
	cout << "dG_dtheta2 = \n" << dG_dtheta2 << endl;
	cout << "dG_dthetaDot1 = \n" << dG_dthetaDot1 << endl;
	cout << "dG_dthetaDot2 = \n" << dG_dthetaDot2 << endl;
	cout << "dC_dtheta1 = \n" << dC_dtheta1 << endl;
	cout << "dC_dtheta2 = \n" << dC_dtheta2 << endl;
	cout << "dC_dthetaDot1 = \n" << dC_dthetaDot1 << endl;
	cout << "dC_dthetaDot2 = \n" << dC_dthetaDot2 << endl;
	cout << "dMInv_dtheta2 = \n" << dMInv_dtheta1 << endl;
	cout << "dMInv_dtheta2 = \n" << dMInv_dtheta2 << endl;
	cout << "dMInv_dthetaDot1 = \n" << dMInv_dthetaDot1 << endl;
	cout << "dMInv_dthetaDot2 = \n" << dMInv_dthetaDot2 << endl;


	// vector of angular rotations and velocities
	std::vector<double*> theta;
	std::vector<double*> omega;
	for (int i = 0; i < joints.size(); i++) {
		double *theta_i = new double[DOF];
		double *omega_i = new double[DOF];
		theta.push_back(theta_i);
		omega.push_back(theta_i);
	}
	
	//----------------------------------------------------------------------------------------------//
	// compute dC_dX, the derivative of the gravitational torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	// declare all necessary arrays, matrices, and vectors
	double *dGdtheta1_d, *dGdtheta2_d, *dGdomega1_d, *dGdomega2_d;
	dGdtheta1_d = new double[joints.size()];
	dGdtheta2_d = new double[joints.size()];
	dGdomega1_d = new double[joints.size()];
	dGdomega2_d = new double[joints.size()];
	matrix<double> dGdtheta1(DOF*joints.size(),1);
	matrix<double> dGdtheta2(DOF*joints.size(),1);
	matrix<double> dGdomega1(DOF*joints.size(),1);
	matrix<double> dGdomega2(DOF*joints.size(),1);
	std::vector<matrix<double>> dG_dX;

	// compute gradients with ADOL-C drivers
	gradient(0,joints.size(),theta[0],dGdtheta1_d);
	gradient(0,joints.size(),theta[1],dGdtheta2_d);
	gradient(0,joints.size(),omega[0],dGdomega1_d);
	gradient(0,joints.size(),omega[1],dGdomega2_d);

	// convert 1D double arrays returned by ADOL-C to correctly defined matrices
	for (int i = 0; i < DOF*joints.size(); i++) {
		dGdtheta1(i,0) = dGdtheta1_d[i];
		dGdtheta2(i,0) = dGdtheta2_d[i];
		dGdomega1(i,0) = dGdomega1_d[i];
		dGdomega2(i,0) = dGdomega2_d[i];
	}
	dG_dX.push_back(dGdtheta1);
	dG_dX.push_back(dGdtheta2);
	dG_dX.push_back(dGdomega1);
	dG_dX.push_back(dGdomega2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dC_dX, the derivative of the centripetal torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	// declare all necessary arrays, matrices, and vectors
	double *dCdtheta1_d, *dCdtheta2_d, *dCdomega1_d, *dCdomega2_d;
	dCdtheta1_d = new double[joints.size()];
	dCdtheta2_d = new double[joints.size()];
	dCdomega1_d = new double[joints.size()];
	dCdomega2_d = new double[joints.size()];
	matrix<double> dCdtheta1(DOF*joints.size(),1);
	matrix<double> dCdtheta2(DOF*joints.size(),1);
	matrix<double> dCdomega1(DOF*joints.size(),1);
	matrix<double> dCdomega2(DOF*joints.size(),1);
	std::vector<matrix<double>> dC_dX;

	// compute gradients with ADOL-C drivers
	gradient(0,joints.size(),theta[0],dCdtheta1_d);
	gradient(0,joints.size(),theta[1],dCdtheta2_d);
	gradient(0,joints.size(),omega[0],dCdomega1_d);
	gradient(0,joints.size(),omega[1],dCdomega2_d);

	// convert 1D double arrays returned by ADOL-C to correctly defined matrices
	for (int i = 0; i < DOF*joints.size(); i++) {
		dCdtheta1(i,0) = dCdtheta1_d[i];
		dCdtheta2(i,0) = dCdtheta2_d[i];
		dCdomega1(i,0) = dCdomega1_d[i];
		dCdomega2(i,0) = dCdomega2_d[i];
	}
	dC_dX.push_back(dGdtheta1);
	dG_dX.push_back(dGdtheta2);
	dG_dX.push_back(dGdomega1);
	dG_dX.push_back(dGdomega2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dMinv_dx, the derivative of the inverse mass matrix w.r.t. to the state vector		//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//

	// declare all necessary arrays, matrices, and vectors
	double *dMinvdtheta1_d, *dMinvdtheta2_d, *dMinvdomega1_d, *dMinvdomega2_d;
	dMinvdtheta1_d = new double[joints.size()];
	dMinvdtheta2_d = new double[joints.size()];
	dMinvdomega1_d = new double[joints.size()];
	dMinvdomega2_d = new double[joints.size()];
	matrix<double> dMinvdtheta1(joints.size(), joints.size());
	matrix<double> dMinvdtheta2(joints.size(), joints.size());
	matrix<double> dMinvdomega1(joints.size(), joints.size());
	matrix<double> dMinvdomega2(joints.size(), joints.size());
	std::vector<matrix<double>> dMInv_dX;

	// compute gradients with ADOL-C drivers
	gradient(0,joints.size(),theta[0],dMinvdtheta1_d);
	gradient(0,joints.size(),theta[1],dMinvdtheta2_d);
	gradient(0,joints.size(),omega[0],dMinvdomega1_d);
	gradient(0,joints.size(),omega[1],dMinvdomega2_d);

	// convert 1D double arrays returned by ADOL-C to correctly defined matrices
	int index = 0;
	for (int i = 0; i < joints.size(); i++) {
		for (int j = 0; j < joints.size(); j++) {
			dMinvdtheta1(i,j) = dMinvdtheta1_d[index];
			dMinvdtheta2(i,j) = dMinvdtheta2_d[index];
			dMinvdomega1(i,j) = dMinvdomega1_d[index];
			dMinvdomega2(i,j) = dMinvdomega2_d[index];
			index++;
		}
	}
	dMInv_dX.push_back(dMinvdtheta1);
	dMInv_dX.push_back(dMinvdtheta2);
	dMInv_dX.push_back(dMinvdomega1);
	dMInv_dX.push_back(dMinvdomega2);
	*/
}

//======================================================================================================================//
// Calculate the derivative of the dynamics equation f with respect to the input torque vector u						//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dfdu_numerical(PxU32 t)
{
	// REPLACE WITH ADOL-C?
	matrix<double> u = uSequence[t];
	matrix<double> C = CSequence[t];
	matrix<double> G = GSequence[t];
	matrix<double> MInv = MSequence[t];

	matrix<double> dfdu(DOF*joints.size()*2, DOF*joints.size());
	for (int i = 0; i < DOF*joints.size(); i++) {
		for (int j = 0; j < DOF*joints.size(); j++) {
			dfdu(i,j) = 0.0;
			dfdu(i+DOF*joints.size(),j) = MInv(i,j);
		}
	} return dfdu;
}