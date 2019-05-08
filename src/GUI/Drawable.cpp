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


Drawable::Drawable() {
	_pose[0] = 0.0; _pose[1] = 0.0; _pose[2] = 0.0;
	_pose[3] = 0.0; _pose[4] = 0.0; _pose[5] = 0.0;
}


void Drawable::setPose(double pose[POSE_DIM]) {
	for (int i = 0; i < 3; ++i) {
		_pose[i] = pose[i];
	}

	for (int i = 3; i < 6; ++i) {
		_pose[i] = RAD_TO_DEG(pose[i]);
	}
}


/**
 * 
 * Note floor is defined in X-Z plane, according to right-hand system
 * 
 **/
Floor::Floor() : Drawable(), startX(FLR_MIN_X), endX(FLR_MAX_X),
					startY(FLR_MIN_Y), endY(FLR_MAX_Y),
					startZ(FLR_MIN_Z), endZ(FLR_MAX_Z) {}


void Floor::draw() {
	glColor3f(.3, .3, .3);

	// draw floor
	glBegin(GL_QUADS);

	// four vertices of the floor
	glVertex3f( startX,-0.2, startZ);
	glVertex3f( startX,-0.2,endZ);
	glVertex3f(endX,-0.2,endZ);
	glVertex3f(endX,-0.2, startZ);

	glEnd();	// GL_QUADS


    glPushMatrix();
	glTranslatef(-100/2, -0.1, -100/2);

	// draw grid
	glBegin(GL_LINES);
	glColor3d(.5, .5, .5);

	// a 0 to 100 units grid in XZ plane
	for (int i = 0; i < 10; ++i) {
		glVertex3d((double)(i*10.0), 0.0, 0.0);
		glVertex3d((double)(i*10.0), 0.0, 100.0);
		glVertex3d(0.0, 0.0, (double)(i*10.0));
		glVertex3d(100.0, 0.0, (double)(i*10.0));
	}

	glEnd();	// GL_LINES

    glPopMatrix();
}


Joint::Joint() {
	_height = 0;
	_radius = 0;
}


void Joint::draw() {
	glPushMatrix ();

    glTranslated(_pose[0], _pose[1], _pose[2]);
    glRotated(_pose[5] , 0,0,1);
    glRotated(_pose[4], 0,1,0);
    glRotated(_pose[3] , 1,0,0);

    glScalef (0.25, 0.25, 0.25);

	// glLineWidth (2.0);
    // float origin[3] = {0,0,0};
    // float xp[3] = {_length,0,0}, yp[3] = {0,_length,0}, zp[3] = {0,0,_length};

    // glBegin (GL_LINES);
    // glColor3f (1,0,0); // X axis is red.
    // glVertex3fv (origin);
    // glVertex3fv (xp);
    // glColor3f (0,1,0); // Y axis is green.
    // glVertex3fv (origin);
    // glVertex3fv (yp);
    // glColor3f (0,0,1); // z axis is blue.
    // glVertex3fv (origin);
    // glVertex3fv (zp);
    // glEnd();
    glPopMatrix ();
}


BaseJoint::BaseJoint(double height, double radius) : Joint() {
	_height = height;
	_radius = radius;
}


void BaseJoint::draw() {
	// TODO: Draw vertical cylinder
}