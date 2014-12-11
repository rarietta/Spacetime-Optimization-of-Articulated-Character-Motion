//-------------------------------------------------------------------------------------------------//
// Spacetime.h
// (c) Ricky Arietta 2014
// CIS 599 Masters Independent Study
// University of Pennsylvania
// Computer Graphics and Gaming Technology Program
//
// This code... 
//-------------------------------------------------------------------------------------------------//

#include <vector>
#include <iostream>
#include "Spacetime.h"
#include "RenderUtil.h"
#include "CameraUtil.h"
#include "PxPhysicsAPI.h"

#ifndef _HAS_EXCEPTIONS
#define _HAS_EXCEPTIONS 0
#endif

using namespace std;

void motionCallback(int x, int y);
void keyPress(const char key);
void keyboardCallback(unsigned char key, int x, int y);
void mouseCallback(int button, int state, int x, int y);
void idleCallback(void);
void renderCallback(void);
void exitCallback(void);
void renderLoop(Spacetime *sys);