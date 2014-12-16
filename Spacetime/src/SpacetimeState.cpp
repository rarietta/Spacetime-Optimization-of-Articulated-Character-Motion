//======================================================================================================================//
// SpacetimeState.cpp																									//
// (c) Ricky Arietta 2014																								//
// CIS 599 Masters Independent Study																					//
// University of Pennsylvania																							//
// Computer Graphics and Gaming Technology Program																		//
//======================================================================================================================//

#include "Spacetime.h"

//======================================================================================================================//
// Utility functions for saving and restoring the state of the system since physX has no builtin funcionality			//
//======================================================================================================================//

matrix<double>
Spacetime::buildStateVector(void) {
	matrix<double> state(joints.size()*DOF*2,1);
	matrix<double> theta = calculateAngularPosition();
	matrix<double> thetaDot = calculateAngularVelocity();
	for (int i = 0; i < joints.size()*DOF; i++)
		state(i,1) = theta(i,1);
	for (int i = 0; i < joints.size()*DOF; i++)
		state(i+joints.size()*DOF,1) = theta(i,1);
	return state;
}

void 
Spacetime::saveState(void) {
	linearVelocityVector.clear();
	angularVelocityVector.clear();
	globalPoseVector.clear();
	for (int i = 0; i < dynamic_actors.size(); i++) {
		PxRigidDynamic* current = dynamic_actors[i];
		linearVelocityVector.push_back(current->getLinearVelocity());
		angularVelocityVector.push_back(current->getAngularVelocity());
		globalPoseVector.push_back(current->getGlobalPose());
	}
}

void 
Spacetime::restoreState(void) {
	for (int i = 0; i < dynamic_actors.size(); i++) {
		PxRigidDynamic *current = dynamic_actors[i];
		current->setLinearVelocity(linearVelocityVector[i]);
		current->setAngularVelocity(angularVelocityVector[i]);
		current->setGlobalPose(globalPoseVector[i]);
	}
}

void
Spacetime::setState(matrix<double> stateVector) {
	for (int i = 0; i < joints.size(); i++) {
		PxQuat q = PxQuat::createIdentity();
		q *= PxQuat(stateVector(i*DOF+X,0), PxVec3(1,0,0));
		q *= PxQuat(stateVector(i*DOF+Y,0), PxVec3(0,1,0));
		q *= PxQuat(stateVector(i*DOF+Z,0), PxVec3(0,0,1));
		// TODO
	}
}

void
Spacetime::switchPause(void) {
	pause = !pause;
}
