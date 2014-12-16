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

	numTimeSteps = 10000;

	state_0 = matrix<double>(4,1);
	state_0(0,0) = -PxPi/4.0f;
	state_0(1,0) =  PxPi/4.0f;
	state_0(2,0) =  0.0f;
	state_0(3,0) =  0.0f;

	state_d = matrix<double>(4,1);
	state_d(0,0) = 0.0f;
	state_d(1,0) = 0.0f;
	state_d(2,0) = 0.0f;
	state_d(3,0) = 0.0f;
}

Spacetime::Spacetime(matrix<double> startPose, matrix<double> endPose, PxU32 numTimeSteps) 
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