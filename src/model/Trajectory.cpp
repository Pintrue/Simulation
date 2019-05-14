#include "Trajectory.hpp"
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace KDL;


Trajectory::Trajectory() : _numJoints(0) {
	_joints.clear();
}


void Trajectory::init(int numJoints) {
	_numJoints = numJoints;
	for (int i = 0; i < numJoints; ++i) {
		_joints.push_back(new AngularKinematics());
	}
}


void Trajectory::prepare(JntArray& start, JntArray& end, double duration) {
	_tNow = 0;	// local time in the trajectory process
	_tEnd = duration;	// total time of the trajectory process
	AngularKinematics::TemporalData s, e;

	for (unsigned int i = 0; i < _joints.size(); ++i) {
		s.angle = start(i);
		e.angle = end(i);
		s.velocity = 0;
		e.velocity = 0;

		_joints[i]->init(s, e, duration);
	}
}


bool Trajectory::nextTimeStep(double tNow, JntArray& next) {
	if (tNow > _tEnd) {
		_tNow = _tEnd;
	} else {
		_tNow = tNow;
	}
	double angle;

	for (unsigned int i = 0; i < _joints.size(); ++i) {
		_joints[i]->angleAtTime(_tNow, &angle);
		next(i) = angle;
		// cout << i << " th joint's angle is " << angle << endl;
	}

	return _tNow < _tEnd; // If trajectory is still in progress
}


void Trajectory::finish() {
	// clear content in container
	for (Joints::iterator it = _joints.begin(); it != _joints.end(); ++it) {
		delete(*it);
	}
	_joints.clear();

	// release memory
	vector<AngularKinematics*>().swap(_joints);
}


double Trajectory::timeNow() {
	return _tNow;
}