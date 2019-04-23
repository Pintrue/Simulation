#include <stdio.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "KinematicsModel.hpp"

using namespace std;
using namespace KDL;


KinematicsModel::KinematicsModel() {}


/*
*	Description of the measurements of the
*	arm to feed to the KDL library.
*/
void KinematicsModel::init(double origin[3]) {
	// Construct segments: links of the arm

	// Origin
	_kdlChain.addSegment(Segment(Joint(Joint::None), Frame(
		Vector(origin[0], origin[1], origin[2]))));
	// Base joint
	_kdlChain.addSegment(Segment(Joint(Joint::RotY), Frame(
		Vector(0.0, 4.20, 0.0))));
	// Shoulder joint
	_kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(
		Vector(0.0, 3.30, 5.0))));
	// Elbow joint
	_kdlChain.addSegment(Segment(Joint(Joint::None),Frame(
		Vector(0.0, 2.0, -10.57))));
	// End-effector
	_kdlChain.addSegment(Segment(Joint(Joint::RotX),Frame(
		Vector(0.0, 3.50, 18.4))));

	// Initialize 
	unsigned int numJoints = _kdlChain.getNrOfJoints();
	_jointAngles = JntArray(numJoints);
}


/*
*	Given the joint angles the arm needs to move to,
*	calculate the final position of the end-effector
*	(also known as the tip of delivery part) in Car-
*	tesian coordinate form.
*/
bool KinematicsModel::jntsToCart(const JntArray& jointAngles, Frame& eeFrame) {
	ChainFkSolverPos_recursive fKSolver =
		ChainFkSolverPos_recursive(_kdlChain);

	if (fKSolver.JntToCart(jointAngles, eeFrame) >= 0) {
		return true;
	} else {
		return false;
	}
}