//-------------------------------------------------------------------------------------------------//
// RenderUtil.h
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#ifndef PHYSX_SPACETIME_RENDER_H
#define PHYSX_SPACETIME_RENDER_H

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"

#if defined(PX_WINDOWS)
#include <windows.h>
#pragma warning(disable: 4505)
#include <glut.h>
#elif defined(PX_LINUX)
#include <GL/glut.h>
#elif defined(PX_APPLE) && !defined(PX_APPLE_IOS)
#include <GLUT/glut.h>
#elif defined(PX_PS3)
#include <GL/glut.h>
#else
#error platform not supported.
#endif

namespace RenderUtil
{
void setupDefaultWindow(const char* name);
void setupDefaultRenderState();

void startRender(const physx::PxVec3& cameraEye, const physx::PxVec3& cameraDir);
void renderActors(physx::PxRigidActor** actors, const physx::PxU32 numActors, bool shadows = false, const physx::PxVec3 & color = physx::PxVec3(0.0f, 0.75f, 0.0f));
void finishRender();
}

#define MAX_NUM_ACTOR_SHAPES 128

#endif //PHYSX_SPACETIME_RENDER_H
