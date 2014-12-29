#include "Spacetime.h"

//==================================================================================================//
// Convert Physx quaternions to individual Euler Angles												//
//==================================================================================================//

PxVec3
Spacetime::QuaternionToEuler(PxQuat q)
{
	float sqw = q.w * q.w;
	float sqx = q.x * q.x;
	float sqy = q.y * q.y;
	float sqz = q.z * q.z;
	float unit = sqx + sqy + sqz + sqw;
	float test = q.x * q.y + q.z * q.w;

	float heading, attitude, bank;

	// check for singularity at north pole
	if (test > 0.499f * unit) { 
		heading = 2.0f * atan2(q.x, q.w);
		attitude = PxPi / 2.0f;
		bank = 0.0f;
	}

	// check for singularity at south pole
	else if (test < -0.499 * unit) {
		heading = -2.0f * atan2(q.x, q.w);
		attitude = -PxPi / 2.0f;
		bank = 0.0f;
	}

	else {
		attitude =  asin (2.0f * test / unit);
		heading  = atan2 (2.0f * q.y * q.w - 2.0f * q.x * q.z,  sqx - sqy - sqz + sqw);
		bank	 = atan2 (2.0f * q.x * q.w - 2.0f * q.y * q.z, -sqx + sqy - sqz + sqw);
	}

	return PxVec3(bank, heading, attitude);
}


//======================================================================================================================//
// Math utility functions for u input torque vector convergence															//
//======================================================================================================================//

PxReal 
Spacetime::SSDmatrix(matrix<double> A, matrix<double> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.RowNo(); i++) {
		for (int j = 0; j < A.ColNo(); j++) {
			SSD += (A(i,j)-B(i,j))*(A(i,j)-B(i,j));
		}
	}
	return SSD;
}

PxReal 
Spacetime::SSDvector(std::vector<matrix<double>> A, std::vector<matrix<double>> B)
{
	PxReal SSD = 0;
	for (int i = 0; i < A.size(); i++)
		SSD += SSDmatrix(A[i], B[i]);
	return SSD;
}

matrix<double>
Spacetime::clamp(matrix<double> theta)
{
	matrix<double> clamped(theta.RowNo(), theta.ColNo());
	for (int i = 0; i < theta.RowNo(); i++) {
		clamped(i,0) = theta(i,0);
		while (clamped(i,0) >  PxPi) clamped(i,0) -= PxPi;
		while (clamped(i,0) < -PxPi) clamped(i,0) += PxPi;
	}
	return clamped;
}

matrix<double> 
Spacetime::I(PxU32 x) {
	matrix<double> I(x,x);
	for (int i = 0; i < x; i++) {
		for (int j = 0; j < x; j++) {
			if (i==j) I(i,j) = 1.0;
			else	  I(i,j) = 0.0;
		}
	}
}
