//--------------------------------------------------------------------------------------------------//
// SpacetimeApplication.cpp																			//
// (c) Ricky Arietta 2014																			//
// CIS 599 Masters Independent Study																//
// University of Pennsylvania																		//
// Computer Graphics and Gaming Technology Program													//
//																									//
// This file is the main program stub for the spacetime optimization project						//
//--------------------------------------------------------------------------------------------------//

#include "Spacetime.h"
#include "PxPhysicsAPI.h"

#ifndef _HAS_EXCEPTIONS
#define _HAS_EXCEPTIONS 0
#endif

using namespace std;
extern void renderLoop(Spacetime *sys);

void main(int argc, char** argv) {
	float theta1 = atof(argv[1]);
	float theta2 = atof(argv[2]);
	matrix<double> state_0(4,1), state_d(4,1);
	state_0(0,0) = theta1; state_0(1,0) = theta2;
	state_0(2,0) = 0.0; state_0(3,0) = 0.0;
	state_d(0,0) = PxPi/4.0; state_d(1,0) = -3.0*PxPi/4.0;
	state_d(2,0) = 0.0; state_d(3,0) = 0.0;
	int numTimeSteps = 10000;
	Spacetime *render_system = new Spacetime(state_0, state_d, numTimeSteps);
	renderLoop(render_system);
}