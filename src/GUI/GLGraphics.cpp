#include "GLGraphics.hpp"

#include <iostream>
using namespace std;

GLGraphics::GLGraphics() {
}


GLGraphics::GLGraphics(const KinematicsModel& km) {
	_model.init(km);
	_goal = Goal(1.5);
}


void GLGraphics::render() {
	_floor.draw();
	_model.draw();
	_goal.draw();
}