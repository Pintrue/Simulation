#include "GLGraphics.hpp"

#include <iostream>
using namespace std;

GLGraphics::GLGraphics() {
}


GLGraphics::GLGraphics(const KinematicsModel& km) {
	_model.init(km);
	_goal = Goal(1.5);
	_obj = Obj(2.5, 1.0);
}


bool ifAtOrigin(double pose[POSE_DIM]) {
	return (pose[0] == 0 && pose[1] == 0 && pose[2] == 0);
}


void GLGraphics::render() {
	_floor.draw();
	_model.draw();
	
	if (!ifAtOrigin(_goal._pose))
		_goal.draw();
	
	if (!ifAtOrigin(_obj._pose))
		_obj.draw();
}