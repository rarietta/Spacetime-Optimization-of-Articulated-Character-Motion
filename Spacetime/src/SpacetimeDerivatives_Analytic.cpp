//======================================================================================================================//
// SpacetimeDerivatives.cpp																								//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//======================================================================================================================//

#include "Spacetime.h"

//======================================================================================================================//
// Analytic G derivatives																								//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dG_dtheta1_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dG_dtheta1(2,1);
	dG_dtheta1(0,0) = -(m1*g*lc1*sin(theta1)) - (m2*g*lc2*sin(theta1)*cos(theta2)) - (m2*g*lc2*cos(theta1)*sin(theta2)) - (m2*g*l1*sin(theta1));
	dG_dtheta1(1,0) = -(m2*g*lc2*sin(theta1)*cos(theta2)) - (m2*g*lc2*cos(theta1)*sin(theta2));
	
	// return analytic derivative
	return dG_dtheta1;
}

matrix<double>  
Spacetime::compute_dG_dtheta2_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dG_dtheta2(2,1);
	dG_dtheta2(0,0) = -(m2*g*lc2*cos(theta1)*sin(theta2)) - (m2*g*lc2*sin(theta1)*cos(theta2));
	dG_dtheta2(1,0) = -(m2*g*lc2*cos(theta1)*sin(theta2)) - (m2*g*lc2*sin(theta1)*cos(theta2));
	
	// return analytic derivative
	return dG_dtheta2;
}

matrix<double>  
Spacetime::compute_dG_dthetaDot1_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dG_dthetaDot1(2,1);
	dG_dthetaDot1(0,0) = 0.0;
	dG_dthetaDot1(1,0) = 0.0;
	
	// return analytic derivative
	return dG_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dG_dthetaDot2_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dG_dthetaDot2(2,1);
	dG_dthetaDot2(0,0) = 0.0;
	dG_dthetaDot2(1,0) = 0.0;
	
	// return analytic derivative
	return dG_dthetaDot2;
}

//======================================================================================================================//
// Analytic C derivatives																								//
//======================================================================================================================//

matrix<double>  
Spacetime::compute_dC_dtheta1_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dC_dtheta1(2,1);
	dC_dtheta1(0,0) = 0.0;
	dC_dtheta1(1,0) = 0.0;
	
	// return analytic derivative
	return dC_dtheta1;
}

matrix<double>  
Spacetime::compute_dC_dtheta2_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dC_dtheta2(2,1);
	dC_dtheta2(0,0) = -(m2*l1*lc2*cos(theta2)*thetaDot2*thetaDot2) - 2*(m2*l1*lc2*cos(theta2)*thetaDot1*thetaDot2);
	dC_dtheta2(1,0) = m2*l1*lc2*cos(theta2)*thetaDot1*thetaDot1;
	
	// return analytic derivative
	return dC_dtheta2;
}

matrix<double>  
Spacetime::compute_dC_dthetaDot1_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dC_dthetaDot1(2,1);
	dC_dthetaDot1(0,0) = -2*m2*l1*lc2*sin(theta2)*thetaDot2;
	dC_dthetaDot1(1,0) =  2*m2*l1*lc2*sin(theta2)*thetaDot1;
	
	// return analytic 
	return dC_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dC_dthetaDot2_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dC_dthetaDot2(2,1);
	dC_dthetaDot2(0,0) = -2*(m2*l1*lc2*sin(theta2)*thetaDot2) - 2*(m2*l1*lc2*sin(theta2)*thetaDot1);
	dC_dthetaDot2(1,0) =  0;
	
	// return analytic derivative
	return dC_dthetaDot2;
}

//======================================================================================================================//
// Analytic MInv derivatives																								//
//======================================================================================================================//

matrix<double>  
Spacetime::compute_dMInv_dtheta1_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dMInv_dtheta1(2,2);
	dMInv_dtheta1(0,0) = 0.0; dMInv_dtheta1(0,1) = 0.0;
	dMInv_dtheta1(1,0) = 0.0; dMInv_dtheta1(1,1) = 0.0;
	
	// return analytic derivative
	return dMInv_dtheta1;
}

matrix<double>  
Spacetime::compute_dMInv_dtheta2_analytic(PxU32 t) 
{
	// system dependent variables
	double g   = gScene->getGravity().y;
	double m1  = dynamic_actors[1]->getMass();
	double m2  = dynamic_actors[2]->getMass();
	double lc1 = joint_local_positions[0].magnitude();
	double lc2 = joint_local_positions[1].magnitude();
	double l1  = 2.0 * joint_local_positions[0].magnitude();
	double l2  = 2.0 * joint_local_positions[1].magnitude();

	// state dependent variables
	matrix<double> state = stateSequence[t];
	double theta1 = state(0,0) + PxPi/2.0f;
	double theta2 = state(1,0) + PxPi/2.0f - theta1;
	double thetaDot1 = state(2,0);
	double thetaDot2 = state(3,0);

	// compute values
	matrix<double> dM_dtheta2(2,2);
	dM_dtheta2(0,0) = -2*(m2*l1*lc2*sin(theta2)); dM_dtheta2(0,1) = -(m2*l1*lc2*sin(theta2));
	dM_dtheta2(1,0) =   -(m2*l1*lc2*sin(theta2)); dM_dtheta2(1,1) = 0.0;
	
	// return analytic derivative
	return -MInvSequence[t] * dM_dtheta2 * MInvSequence[t];
}

matrix<double>  
Spacetime::compute_dMInv_dthetaDot1_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dMInv_dthetaDot1(2,2);
	dMInv_dthetaDot1(0,0) = 0.0; dMInv_dthetaDot1(0,1) = 0.0;
	dMInv_dthetaDot1(1,0) = 0.0; dMInv_dthetaDot1(1,1) = 0.0;
	
	// return analytic derivative
	return dMInv_dthetaDot1;
}

matrix<double>  
Spacetime::compute_dMInv_dthetaDot2_analytic(PxU32 t) 
{
	// compute values
	matrix<double> dMInv_dthetaDot2(2,2);
	dMInv_dthetaDot2(0,0) = 0.0; dMInv_dthetaDot2(0,1) = 0.0;
	dMInv_dthetaDot2(1,0) = 0.0; dMInv_dthetaDot2(1,1) = 0.0;
	
	// return analytic derivative
	return dMInv_dthetaDot2;
}

//======================================================================================================================//
// Calculate the derivative of the cost function L with respect to the input torque vector u							//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dLdu_analytic(PxU32 t)
{
	return uSequence[t];
}

//======================================================================================================================//
// Calculate the derivative of the cost function L with respect to the state vector X									//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dLdx_analytic(PxU32 t)
{
	matrix<double> dLdx(DOF*joints.size(), 1);
	for (int i = 0; i < DOF*joints.size(); i++)
		dLdx(i,0) = 0.0;
	return dLdx;
}

//======================================================================================================================//
// Calculate the derivative of the dynamics equation f with respect to the input torque vector u						//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dfdu_analytic(PxU32 t)
{
	matrix<double> MInv = MSequence[t];
	matrix<double> dfdu(DOF*joints.size()*2, DOF*joints.size());
	for (int i = 0; i < DOF*joints.size(); i++) {
		for (int j = 0; j < DOF*joints.size(); j++) {
			dfdu(i,j) = 0.0;
			dfdu(i+DOF*joints.size(),j) = MInv(i,j);
		}
	} return dfdu;
}

//======================================================================================================================//
// Calculate the derivative of the dynamics equation f with respect to the state vector X								//
//======================================================================================================================//

matrix<double> 
Spacetime::compute_dfdx_analytic(PxU32 t)
{	
	//----------------------------------------------------------------------------------------------//
	// compute dG_dX, the derivative of the gravitational torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> dG_dtheta1    = compute_dG_dtheta1_analytic(t);
	matrix<double> dG_dtheta2    = compute_dG_dtheta2_analytic(t);
	matrix<double> dG_dthetaDot1 = compute_dG_dthetaDot1_analytic(t);
	matrix<double> dG_dthetaDot2 = compute_dG_dthetaDot2_analytic(t);

	std::vector<matrix<double>> dG_dX;
	dG_dX.push_back(dG_dtheta1);
	dG_dX.push_back(dG_dtheta2);
	dG_dX.push_back(dG_dthetaDot1);
	dG_dX.push_back(dG_dthetaDot2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dC_dX, the derivative of the centripetal torque matrix w.r.t. to the state vector	//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> dC_dtheta1    = compute_dC_dtheta1_analytic(t);
	matrix<double> dC_dtheta2    = compute_dC_dtheta2_analytic(t);
	matrix<double> dC_dthetaDot1 = compute_dC_dthetaDot1_analytic(t);
	matrix<double> dC_dthetaDot2 = compute_dC_dthetaDot2_analytic(t);

	std::vector<matrix<double>> dC_dX;
	dC_dX.push_back(dC_dtheta1);
	dC_dX.push_back(dC_dtheta2);
	dC_dX.push_back(dC_dthetaDot1);
	dC_dX.push_back(dC_dthetaDot2);
	
	//----------------------------------------------------------------------------------------------//
	// compute dMinv_dx, the derivative of the inverse mass matrix w.r.t. to the state vector		//
	// and build full 3D derivative matrix from partials											//
	//----------------------------------------------------------------------------------------------//
	
	matrix<double> dMInv_dtheta1    = compute_dMInv_dtheta1_analytic(t);
	matrix<double> dMInv_dtheta2    = compute_dMInv_dtheta2_analytic(t);
	matrix<double> dMInv_dthetaDot1 = compute_dMInv_dthetaDot1_analytic(t);
	matrix<double> dMInv_dthetaDot2 = compute_dMInv_dthetaDot2_analytic(t);

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

	matrix<double> term1(DOF*joints.size(), DOF*joints.size()*2);
	for (int i = 0; i < DOF*joints.size()*2; i++) {
		matrix<double> sum = (u - C - G);
		matrix<double> column_i = dMInv_dX[i] * sum;
		for (int j = 0; j < DOF*joints.size(); j++) {
			term1(j,i) = column_i(j,0);
		}
	}

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
	// dfdx = |df1/dx| = |        0         |         I          |									//
	//		  |df2/dx|	 | dMInv_dX*(u+C+G) - Minv*(dC_dx+dG_dx) |									//
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
	
	return dfdx;
}
