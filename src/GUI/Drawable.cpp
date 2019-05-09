#include "Drawable.hpp"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "../utils/Utils.hpp"
#include <iostream>


#define FLR_MIN_X -20.0
#define FLR_MAX_X 20.0
#define FLR_MIN_Y -20.0
#define FLR_MAX_Y 20.0
#define FLR_MIN_Z -20.0
#define FLR_MAX_Z 20.0


using namespace std;
using namespace KDL;


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
					startZ(FLR_MIN_Z), endZ(FLR_MAX_Z) {

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
		// cout << "Base joint " << _pose[0] << " " << _pose[1] << " " << _pose[2] << endl;
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
		// cout << "Arm joint " << _pose[0] << " " << _pose[1] << " " << _pose[2] << endl;
		glTranslatef(_pose[0], _pose[1], _pose[2]);
		glRotated(_pose[5], 0, 0, 1);
		glRotated(_pose[4], 0, 1, 0);
		glRotated(_pose[3], 1, 0, 0);
		glColor3d(0.0, 1.0, 0.0);

		// GLUquadricObj* quadratic;
		// quadratic = gluNewQuadric();
		// glRotated(90.0, 0.0, 1.0, 0.0);
		// gluCylinder(quadratic, _radius, _radius, _height, 10, 25);
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

		// GLUquadricObj* quadratic;
		// quadratic = gluNewQuadric();
		// glRotated(90.0, 0.0, 1.0, 0.0);
		// gluCylinder(quadratic, _radius, _radius, _height, 10, 25);
		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
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

		// GLUquadricObj* quadratic;
		// quadratic = gluNewQuadric();
		// glRotated(90.0, 0.0, 1.0, 0.0);
		// gluCylinder(quadratic, _radius, _radius, _height, 10, 25);
		gluSphere(gluNewQuadric(), _radius, 10, 25);

	glPopMatrix();
}


Model::Model() {
	_joints.clear();
}


void Model::init(const KinematicsModel& km) {
	Frame frame;

	_joints.push_back(new Jnt());
	_joints.push_back(new BaseJoint(4.2, 2.4));
	_joints.push_back(new ArmJoint(0.5, 2.1));
	_joints.push_back(new ForearmJoint(0.5, 2.1));
	_joints.push_back(new EndEffector(2.1));

	unsigned int numSegmnts = km._kdlChain.getNrOfSegments();
	for (unsigned int i = 0; i < numSegmnts; ++i) {
		frame = frame * km._kdlChain.getSegment(i).pose(0.0);

		double pose[POSE_DIM];
		convFrameToPose(frame, pose);

		_joints[i]->setPose(pose);
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


void Model::update(const KinematicsModel& km, const KDL::JntArray& jnts) {
	Frame frame;
	double q[5];

	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		q[i + 1] = jnts(i);
	}
	q[0] = 0.0; q[4] = 0.0;

	unsigned int numSegmnts = km._kdlChain.getNrOfSegments();
	for (unsigned int i = 0; i < numSegmnts; ++i) {
		frame = frame * km._kdlChain.getSegment(i).pose(q[i]);

		double pose[POSE_DIM];
		convFrameToPose(frame, pose);

		_joints[i]->setPose(pose);
	}
}


void Model::draw() {
	JointList::iterator it = _joints.begin();
	(*it)->draw();
	double prevX, prevY, prevZ;
	prevX = (*it)->_pose[0]; prevY = (*it)->_pose[1]; prevZ = (*it)->_pose[2];
	++it;

	for (it; it != _joints.end(); ++it) {
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