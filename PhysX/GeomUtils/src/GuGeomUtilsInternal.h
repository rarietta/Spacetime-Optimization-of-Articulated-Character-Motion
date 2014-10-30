// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2013 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_GEOM_UTILS_INTERNAL_H
#define GU_GEOM_UTILS_INTERNAL_H

#include "CmPhysXCommon.h"
#include "PxBounds3.h"
#include "GuCapsule.h"
#include "GuBox.h"
#include "PxCapsuleGeometry.h"
#include "PxBoxGeometry.h"
#include "PsMathUtils.h"
#include "PsUtilities.h"

namespace physx
{
namespace Gu
{
	class Plane;
	class Capsule;
	class Box;
	class Segment;

	PX_PHYSX_COMMON_API const PxU8*		getBoxEdges();

	PX_PHYSX_COMMON_API void			computeBoxPoints(const PxBounds3& bounds, PxVec3* PX_RESTRICT pts);
	PX_PHYSX_COMMON_API void			computeBoundsAroundVertices(PxBounds3& bounds, PxU32 nbVerts, const PxVec3* PX_RESTRICT verts);

	PX_PHYSX_COMMON_API void			computeBoxAroundCapsule(const Capsule& capsule, Box& box);
	PX_FORCE_INLINE		void			computePxBoxAroundCapsule(const PxCapsuleGeometry& capsuleGeom, PxBoxGeometry& box)
	{
		box.halfExtents = PxVec3(capsuleGeom.radius + (capsuleGeom.halfHeight), capsuleGeom.radius, capsuleGeom.radius);
	}

	PX_PHYSX_COMMON_API PxPlane			getPlane(const PxTransform& pose);
	PX_PHYSX_COMMON_API PxTransform		getCapsuleTransform(const Gu::Capsule& capsule, PxReal& halfHeight);

	PX_FORCE_INLINE void getCapsuleSegment(const PxTransform& transform, const PxCapsuleGeometry& capsuleGeom, Gu::Segment& segment)
	{
		const PxVec3 tmp = transform.q.getBasisVector0() * capsuleGeom.halfHeight;
		segment.p0 = transform.p + tmp;
		segment.p1 = transform.p - tmp;
	}

	PX_FORCE_INLINE	void getCapsule(Gu::Capsule& capsule, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose)
	{
		getCapsuleSegment(pose, capsuleGeom, capsule);
		capsule.radius = capsuleGeom.radius;
	}

	// AP: common api prefix is needed for use in PxcSweepConvexMesh
	PX_PHYSX_COMMON_API void computeSweptBox(
		Gu::Box& box, const PxVec3& extents, const PxVec3& center, const PxMat33& rot, const PxVec3& unitDir, const PxReal distance);

}  // namespace Gu

}

#endif
