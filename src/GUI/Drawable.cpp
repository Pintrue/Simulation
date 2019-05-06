#include "Drawable.hpp"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#define FLR_MIN_X -20.0
#define FLR_MAX_X 20.0
#define FLR_MIN_Y -20.0
#define FLR_MAX_Y 20.0
#define FLR_MIN_Z -20.0
#define FLR_MAX_Z 20.0


Floor::Floor() : startX(FLR_MIN_X), endX(FLR_MAX_X),
						startY(FLR_MIN_Y), endY(FLR_MAX_Y),
						startZ(FLR_MIN_Z), endZ(FLR_MAX_Z) {}

void Floor::draw() {
	// glColor3f(.3, .3, .3);
	// glBegin(GL_QUADS);

	// // the four vertices of the grid floor
	// glVertex3d(startX, endY,);
	glColor3f(.3,.3,.3);
	glBegin(GL_QUADS);
	glVertex3f( 0,-0.001, 0);
	glVertex3f( 0,-0.001,10);
	glVertex3f(10,-0.001,10);
	glVertex3f(10,-0.001, 0);
	glEnd();

	glBegin(GL_LINES);
	for(int i=0;i<=10;i++) {
		if (i==0) { glColor3f(.6,.3,.3); } else { glColor3f(.25,.25,.25); };
		glVertex3f(i,0,0);
		glVertex3f(i,0,10);
		if (i==0) { glColor3f(.3,.3,.6); } else { glColor3f(.25,.25,.25); };
		glVertex3f(0,0,i);
		glVertex3f(10,0,i);
	};
	glEnd();

}