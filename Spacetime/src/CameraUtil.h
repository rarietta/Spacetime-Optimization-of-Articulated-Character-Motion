//-------------------------------------------------------------------------------------------------//
// CameraUtil.h
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#ifndef PHYSX_SPACETIME_CAMERA_H
#define PHYSX_SPACETIME_CAMERA_H

#include "foundation/PxTransform.h"

namespace RenderUtil
{
class Camera
{
public:
	Camera(const physx::PxVec3 &eye, const physx::PxVec3& dir);

	void				handleMouse(int button, int state, int x, int y);
	bool				handleKey(unsigned char key, int x, int y, float speed = 1.0f);
	void				handleMotion(int x, int y);
	void				handleAnalogMove(float x, float y);

	physx::PxVec3		getEye()	const;
	physx::PxVec3		getDir()	const;
	physx::PxTransform	getTransform() const;
private:
	physx::PxVec3	mEye;
	physx::PxVec3	mDir;
	int				mMouseX;
	int				mMouseY;
};


}


#endif //PHYSX_SPACETIME_CAMERA_H