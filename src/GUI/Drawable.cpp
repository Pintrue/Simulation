#include "Drawable.hpp"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "../utils/Utils.hpp"
#include <iostream>
#include <iterator>
#include <math.h>


#define FLR_MIN_X -20.0
#define FLR_MAX_X 20.0
#define FLR_MIN_Y -20.0
#define FLR_MAX_Y 20.0
#define FLR_MIN_Z -20.0
#define FLR_MAX_Z 20.0

#pragma clang diagnostic ignored "-Wdeprecated-declarations"

using namespace std;


Drawable::Drawable() {
	_pose[0] = 0.0; _pose[1] = 0.0; _pose[2] = 0.0;
	_pose[3] = 0.0; _pose[4] = 0.0; _pose[5] = 0.0;
}


Drawable::~Drawable() {

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
 * Note floor is defined in X-Z plane, according to left-hand system
 * 
 **/
Floor::Floor() : Drawable(), startX(FLR_MIN_X), endX(FLR_MAX_X),
					// startY(FLR_MIN_Y), endY(FLR_MAX_Y),
					startZ(FLR_MIN_Z), endZ(FLR_MAX_Z) {

}


Floor::~Floor() {

}


void Floor::draw() {
	glColor3f(.3, .3, .3);

	// draw floor
	glBegin(GL_QUADS);

	// four vertices of the floor
		glVertex3d(startX,-0.2, startZ);
		glVertex3d(startX,-0.2,endZ);
		glVertex3d(endX,-0.2,endZ);
		glVertex3d(endX,-0.2, startZ);

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


Jnt::Jnt() {
	_height = 0;
	_radius = 0;
}


Jnt::~Jnt() {

}


void Jnt::draw() {
	glPushMatrix ();

    glTranslated(_pose[0], _pose[1], _pose[2]);
    glRotated(_pose[5] , 0,0,1);
    glRotated(_pose[4], 0,1,0);
    glRotated(_pose[3] , 1,0,0);

    glScalef (0.25, 0.25, 0.25);

    glPopMatrix ();
}


BaseJoint::BaseJoint(double height, double radius) : Jnt() {
	_height = height;
	_radius = radius;
}


void BaseJoint::draw() {
	glPushMatrix();
		glTranslatef(_pose[0], _pose[1] - _height, _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(0.0, 1.0, 0.0);

		GLUquadricObj* quad;
		quad = gluNewQuadric();
		glRotated(90.0, -1.0, 0.0, 0.0);
		gluCylinder(quad, _radius, _radius, _height, 10, 25);

	glPopMatrix();
}


ArmJoint::ArmJoint(double height, double radius) : Jnt() {
	_height = height;
	_radius = radius;
}


void ArmJoint::draw() {
	glPushMatrix();

		glTranslatef(_pose[0], _pose[1], _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(0.0, 1.0, 0.0);

		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
}


ForearmJoint::ForearmJoint(double height, double radius) {
	_height = height;
	_radius = radius;
}


void ForearmJoint::draw() {
	glPushMatrix();
		// cout << "Forearm joint " << _pose[0] << " " << _pose[1] << " " << _pose[2] << endl;
		glTranslatef(_pose[0], _pose[1], _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(0.0, 1.0, 0.0);

		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
}


EndEffector::EndEffector() {

}


EndEffector::EndEffector(double radius) {
	_radius = radius;
}


void EndEffector::draw() {
	glPushMatrix();

		glTranslatef(_pose[0], _pose[1], _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(1.0, 0.0, 0.0);

		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
}


Model::Model() {
	_joints.clear();
}


Model::~Model() {

}


void Model::init() {
	_joints.push_back(new Jnt());
	_joints.push_back(new BaseJoint(4.2, 2.4));
	_joints.push_back(new ArmJoint(0.5, 2.1));
	_joints.push_back(new ForearmJoint(0.5, 2.1));
	_joints.push_back(new EndEffector(2.1));

	double initAngle[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	double allPoss[POSS_NUMBER][POSE_FRAME_DIM];

	KinematicsModel km;

	if (km.getAllPoss(initAngle, allPoss)) {
		for (int i = 0; i < POSS_NUMBER; ++i) {
			_joints[i + 1]->setPose(allPoss[i]);
		}
	} else {
		cout << "Given angles are out of bound at Model::init() " 
				<< "in file Drawable.cpp" << endl;
	}
}


void Model::finish() {
	for (JointList::iterator it = _joints.begin(); it != _joints.end(); ++it) {
		delete(*it);
	}
	_joints.clear();

	// release memory
	vector<Drawable*>().swap(_joints);
}


void Model::update(double jnts[NUM_OF_JOINTS]) {
	double allPoss[POSS_NUMBER][POSE_FRAME_DIM];

	KinematicsModel km;
	if (km.getAllPoss(jnts, allPoss)) {
		for (int i = 0; i < POSS_NUMBER; ++i) {
			_joints[i + 1]->setPose(allPoss[i]);
		}
	} else {
		cout << "Given angles are out of bound at Model::update() " 
				<< "in file Drawable.cpp" << endl;
	}
}


void Model::draw() {
	JointList::iterator it = _joints.begin();
	(*it)->draw();
	double prevX, prevY, prevZ;
	prevX = (*it)->_pose[0]; prevY = (*it)->_pose[1]; prevZ = (*it)->_pose[2];
	++it;

	for (; it != _joints.end(); ++it) {
		// draw line
		glPushMatrix();

		GLfloat widthRange[2] = {0.0f, 0.0f};
		glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, widthRange);
		glLineWidth(widthRange[1]);
		glBegin(GL_LINES);

			glColor3d(.5, .5, .5);		
			glVertex3d(prevX, prevY, prevZ);
			glVertex3d((*it)->_pose[0], (*it)->_pose[1], (*it)->_pose[2]);

		glEnd();	// GL_LINES

		glPopMatrix();

		(*it)->draw();
		prevX = (*it)->_pose[0]; prevY = (*it)->_pose[1]; prevZ = (*it)->_pose[2];
	}
	glLineWidth(1);
}


Goal::Goal() {

}


Goal::Goal(double radius) {
	_radius = radius;
}


Goal::~Goal() {

}


void Goal::draw() {
	glPushMatrix();

		glTranslatef(_pose[0], _pose[1], _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(0.0, 0.0, 1.0);

		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
}


Obj::Obj() {

}


Obj::Obj(double radius, double height) {
	_radius = radius;
	_height = height;
}


Obj::~Obj() {

}


void Obj::draw() {
	glPushMatrix();

		glTranslatef(_pose[0], _pose[1], _pose[2]);
		// glRotated(_pose[5], 0, 0, 1);
		// glRotated(_pose[4], 0, 1, 0);
		// glRotated(_pose[3], 1, 0, 0);
		glColor3d(1.0, 1.0, 0.0);
		glRotated(90.0, -1.0, 0.0, 0.0);

		gluCylinder(gluNewQuadric(), _radius, _radius, _height, 10, 25);

	glPopMatrix();
}