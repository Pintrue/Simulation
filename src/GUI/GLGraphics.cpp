#include "GLGraphics.hpp"

void GLGraphics::render() {
	_floor.draw();
	_model.draw();
	
	// EndEffector ee = EndEffector(2.1);
	// double pose[6];
	// pose[0]=10; pose[1] = 0; pose[2] = 0;
	// ee.setPose(pose);
	// ee.draw();
}