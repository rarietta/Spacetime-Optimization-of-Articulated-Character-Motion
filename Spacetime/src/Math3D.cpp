#include "Spacetime.h"

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