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
	deltaT = 1.0f / 600.0f;

	numTimeSteps = 10000;
	uThreshold = 10;

	initPhysics();

	state_0 = matrix<double>(DOF*joints.size()*2,1);
	state_d = matrix<double>(DOF*joints.size()*2,1);
		
	if (DOF == 1) {
		state_0(0,0) = -PxPi/4.0f;
		state_0(1,0) =  PxPi/4.0f;
		state_0(2,0) =  0.0f;
		state_0(3,0) =  0.0f;
		
		state_d(0,0) = 0.0f;
		state_d(1,0) = 0.0f;
		state_d(2,0) = 0.0f;
		state_d(3,0) = 0.0f;
	} 
	else if (DOF == 3) {
		state_0(0,0) = -PxPi/4.0f;
		state_0(1,0) =  0.0f;
		state_0(2,0) =  0.0f;
		state_0(3,0) =  PxPi/4.0f;
		state_0(4,0) =  0.0f;
		state_0(5,0) =  0.0f;
		state_0(6,0) =  0.0f;
		state_0(7,0) =  0.0f;
		state_0(8,0) =  0.0f;
		state_0(9,0) =  0.0f;
		state_0(10,0) = 0.0f;
		state_0(11,0) = 0.0f;

		state_d(0,0) = 0.0f;
		state_d(1,0) = 0.0f;
		state_d(2,0) = 0.0f;
		state_d(3,0) = 0.0f;
		state_d(4,0) = 0.0f;
		state_d(5,0) = 0.0f;
		state_d(6,0) = 0.0f;
		state_d(7,0) = 0.0f;
		state_d(8,0) = 0.0f;
		state_d(9,0) = 0.0f;
		state_d(10,0) = 0.0f;
		state_d(11,0) = 0.0f;
	}
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
	deltaT = 1.0 / 600.0f;

	this->state_0 = startPose;
	this->state_d = endPose;
	this->numTimeSteps = numTimeSteps;
	
	initPhysics();
}