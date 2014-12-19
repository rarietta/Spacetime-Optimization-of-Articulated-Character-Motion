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
	std::vector<PxQuat> theta;
	for (int i = 1; i <= joints.size(); i++) {
		PxQuat q = PxQuat::createIdentity();
		if (DOF > X) { q *= PxQuat(stateVector((i-1)*DOF+X,0), PxVec3(1,0,0)); }
		if (DOF > Y) { q *= PxQuat(stateVector((i-1)*DOF+Y,0), PxVec3(0,1,0)); }
		if (DOF > Z) { q *= PxQuat(stateVector((i-1)*DOF+Z,0), PxVec3(0,0,1)); }
		theta.push_back(q);
	}
	dynamic_actors[0]->setGlobalPose(PxTransform(root, PxQuat::createIdentity()));
	PxVec3 lastJointPos = dynamic_actors[0]->getGlobalPose().p + PxVec3(0,0.5,0);
	PxQuat lastJointRot = dynamic_actors[0]->getGlobalPose().q;
	for (int i = 1; i <= joints.size(); i++) {
		PxRigidDynamic *current = dynamic_actors[i];
		PxVec3 t = theta[i-1].rotate(-joint_local_positions[i-1]);
		PxVec3 gPos = lastJointPos + /*lastJointRot.rotate(*/t/*)*/;
		current->setGlobalPose(PxTransform(gPos, theta[i-1]));
		lastJointPos = lastJointPos + 2*t;
	}
	for (int i = 1; i <= joints.size(); i++) {
		PxRigidDynamic *current = dynamic_actors[i];
		PxVec3 angularVelocity;
		if (DOF > X) { angularVelocity[X] = stateVector(joints.size()*DOF + (i-1)*DOF+X,0); }
		else		 { angularVelocity[X] = 0.0; }
		if (DOF > Y) { angularVelocity[Y] = stateVector(joints.size()*DOF + (i-1)*DOF+Y,0); }
		else		 { angularVelocity[Y] = 0.0; }
		if (DOF > Z) { angularVelocity[Z] = stateVector(joints.size()*DOF + (i-1)*DOF+Z,0); }
		else		 { angularVelocity[Z] = 0.0; }
		current->setAngularVelocity(angularVelocity);
		current->setLinearVelocity(PxVec3(0,0,0));
	}
}

matrix<double>
Spacetime::getState(void) 
{
	matrix<double> state(joints.size()*DOF*2,1);
	matrix<double> theta = calculateAngularPosition();
	matrix<double> thetaDot = calculateAngularVelocity();
	for (int i = 0; i < joints.size()*DOF; i++)
		state(i,0) = theta(i,0);
	for (int i = 0; i < joints.size()*DOF; i++)
		state(i+joints.size()*DOF,0) = thetaDot(i,0);
	return state;
}

void
Spacetime::switchPause(void) {
	pause = !pause;
}
