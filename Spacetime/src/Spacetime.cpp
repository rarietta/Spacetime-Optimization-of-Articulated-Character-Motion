//-------------------------------------------------------------------------------------------------//
// Spacetime.cpp
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#include "Spacetime.h"

Spacetime::Spacetime(void) 
{
	// is the program running?
	pause = false;

	// Required physX runtime variable initializations
	gScene = NULL;
	gPhysics = NULL;
	gMaterial	= NULL;
	gFoundation = NULL;
	gDispatcher = NULL;
	gConnection	= NULL;
	deltaT = 1.0 / 60.0;
}

Spacetime::Spacetime(matrix<PxReal> startPose, matrix<PxReal> endPose, PxU32 numTimeSteps) 
{
	// is the program running?
	pause = false;

	// Required physX runtime variable initializations
	gScene = NULL;
	gPhysics = NULL;
	gMaterial	= NULL;
	gFoundation = NULL;
	gDispatcher = NULL;
	gConnection	= NULL;
	deltaT = 1.0 / 60.0;

	this->state_0 = startPose;
	this->state_d = endPose;
	this->numTimeSteps = numTimeSteps;
}