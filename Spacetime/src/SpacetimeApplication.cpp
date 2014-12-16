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

void main(int, char**) {
	Spacetime *render_system = new Spacetime();
	render_system->initPhysics();
	matrix<double> optimized = render_system->Optimize();
	renderLoop(render_system);
}