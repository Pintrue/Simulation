#include "GLWidgets.hpp"
#include <QtGui/QMouseEvent>
#include <QtCore/QCoreApplication>
#include <QtWidgets/QMessageBox>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include "../model/Constraints.hpp"
#include "UIUtils.hpp"
#include "../utils/Utils.hpp"
#include <iostream>
#include <math.h>

#pragma clang diagnostic ignored "-Wdeprecated-declarations"

using namespace std;


GLWidgets::GLWidgets(QWidget* parent) : QOpenGLWidget(parent) {
	// initial camera angles
	_cam[0] = 40; _cam[1] = 10; _cam[2] = 40;
	_mRotX = 0; _mRotY = 0; _mRotZ = 0;

	_timer = new QTimer(this);
	_timer->setInterval(500);


	// initialize the kinematics modules
	double ori[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	_sim = Sim(ori);

	// initialize the graphical modules
	_glg = GLGraphics(_sim._km);

	_angleInput = "0.0, 0.0, 0.0";
	_actionsIter = 0;
	_trajOn = false;
	_hasObj = false;
}


GLWidgets::~GLWidgets() {
	_timer->stop();
	delete _timer;

	cleanup();	// <- do the opengl finalization

	/**
	 * TODO: finalize the graphics components here
	 * 
	 * makeCurrent();
	 * _sim._glg.finish();
	 **/
	_glg._model.finish();
	_sim._tjt.finish();
}


QSize GLWidgets::minimumSizeHint() const {
	return QSize(50, 50);
}


QSize GLWidgets::sizeHint() const {
	return QSize(500, 500);
}


Sim GLWidgets::getSim() {
	return _sim;
}


/**
 * 
 * This function accepts external simulation configuration.
 * 
 **/
void GLWidgets::setSim(Sim sim) {
	_sim = sim;

	// set the target cartesian position
	double destPos[6];
	double objPos[6];
	for (int i = 0; i < 3; ++i) {
		destPos[i] = _sim._target[i];
		objPos[i] = _sim._initObj[i];
		_sim._obj[i] = _sim._initObj[i];
	}
	_glg._goal.setPose(destPos);
	_glg._obj.setPose(objPos);
}


void GLWidgets::setJointAngle(double ja[NUM_OF_JOINTS]) {
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		_ja[i] = ja[i];
	}
}


static void normalizeAngle(int& angle) {
	while (angle < 0) {
		angle += 360 * 16;
	}
	while (angle > 360 * 16) {
		angle -= 360 * 16;
	}
}


void GLWidgets::setXRotation(int angle) {
	normalizeAngle(angle);

	if (angle != _mRotX) {
		_mRotX = angle;
		emit xRotationChanged(angle);
		update();
	}
}


void GLWidgets::setYRotation(int angle) {
	normalizeAngle(angle);

	if (angle != _mRotY) {
		_mRotY = angle;
		emit yRotationChanged(angle);
		update();
	}
}


void GLWidgets::setZRotation(int angle) {
	normalizeAngle(angle);

	if (angle != _mRotZ) {
		_mRotZ = angle;
		emit zRotationChanged(angle);
		update();
	}
}


void GLWidgets::updateAngleInput(const QString& input) {
	_angleInput = input;
	
}


void GLWidgets::enableTraj(bool on) {
	_trajOn = on;
}


void GLWidgets::execAction() {
	double checkJA[NUM_OF_JOINTS];
	if (isValidJAFormat(_angleInput, checkJA)) {
		for (int i = 0; i < NUM_OF_JOINTS; ++i)
			_ja[i] = checkJA[i];

		if (_trajOn) {
			trajAction();
			return;
		} else {
			plainAction();
		}
	} else {
		QMessageBox::information(0, tr("Stop"), tr("Invalid input format - try again."));
	}
}


void GLWidgets::plainAction() {
	double pose[POSE_DIM];
	if (_sim._km.getPoseByJnts(_ja, pose)) {
		QString eePos = QString("[%1, %2, %3]")
             .arg(QString::number(pose[0], 'f', 2),
			 		QString::number(pose[1], 'f', 2),
					QString::number(pose[2], 'f', 2));
		emit updateEEPos(eePos);
	} else {
		QMessageBox::information(0, tr("Stop"), tr("No solution - try again."));
		_timer->stop();
	}

	update();
}


bool ifHadObj(const double eePos[6], const double objPos[6]) {
	double diff = 0;

	for (int i = 0; i < CART_DIM; ++i) {
		double delta = eePos[i] - objPos[i];
		diff += delta * delta;
	}
	bool res = sqrt(diff) <= OBJ_ATTACHED_RANGE;
	return res;
	// return sqrt(diff) <= OBJ_ATTACHED_RANGE;
}


void GLWidgets::plainActionObj() {
	double pose[POSE_DIM];
	if (_sim._km.getPoseByJnts(_ja, pose)) {
		initFwdKM();
		double magnetPose[POSE_FRAME_DIM];
		getMagnetPoseByJnts(_ja, magnetPose);
		if (roundf(_sim._actions[_actionsIter - 1][3]) == 1) {
			if (_hasObj) {
				_glg._obj.setPose(pose);
			} else if (ifHadObj(magnetPose, _sim._obj)) {
				_glg._obj.setPose(pose);
				_hasObj = true;
			}
		} else {
			double objPos[6];
			objPos[0] = _glg._obj._pose[0];
			objPos[1] = OBJ_HEIGHT;
			objPos[2] = _glg._obj._pose[2];
			_glg._obj.setPose(objPos);
		}
		QString eePos = QString("[%1, %2, %3]")
             .arg(QString::number(pose[0], 'f', 2),
			 		QString::number(pose[1], 'f', 2),
					QString::number(pose[2], 'f', 2));
		emit updateEEPos(eePos);
	} else {
		QMessageBox::information(0, tr("Stop"), tr("No solution - try again."));
		_timer->stop();
	}

	update();
}


void GLWidgets::trajAction() {
	/**
	 * prepare the trajectory from the current angle (1st param.)
	 * to the intended angle (2nd param.)
	 **/
	connect(_timer, SIGNAL(timeout()), this, SLOT(trajNextTimeStep()));
	_sim._tjt.prepare(_sim._km._jointAngles, _ja, 10.0);
	_timer->start();
}


void GLWidgets::trajNextTimeStep() {
	double nextJA[NUM_OF_JOINTS];
	if (!_sim._tjt.nextTimeStep(_sim._tjt.timeNow() + 1, nextJA)) {
		_timer->stop();
	}

	// _ja = nextJA;
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		_ja[i] = nextJA[i];
	}
	plainAction();
}


void GLWidgets::moveByActionPath() {
	// move to the initial JA set by resetState() in C_api
	// _ja = _sim._initJA;
	for (int i = 0; i < NUM_OF_JOINTS; ++i) {
		_ja[i] = _sim._initJA[i];
	}
	plainAction();

	connect(_timer, SIGNAL(timeout()), this, SLOT(actPathNextTimeStep()));
	_timer->start();
}


void GLWidgets::actPathNextTimeStep() {
	if (_actionsIter >= _sim._numOfActions) {
		_timer->stop();
		return;
	}

	double result[FULL_STATE_NUM_COLS];
	if (regulateJntAngles(_ja, _sim._actions[_actionsIter], result)) {
		for (int i = 0; i < NUM_OF_JOINTS; ++i) {
			_ja[i] += _sim._actions[_actionsIter][i];
		}

		if (ACTION_DIM == 4) {
			++_actionsIter;
			plainActionObj();
		} else {
			++_actionsIter;
			plainAction();
		}

	} else {
		QMessageBox::information(0, tr("Stop"), tr("Path moved out of bounds."));
		_timer->stop();
	}
}


void GLWidgets::cleanup() {
	// TODO: implementation
	makeCurrent();
	doneCurrent();
}


void GLWidgets::initializeGL() {
	QOpenGLContext context;
	connect(&context, &QOpenGLContext::aboutToBeDestroyed, this, &GLWidgets::cleanup);
	initializeOpenGLFunctions();

	glClearColor(0.1, 0.1, 0.1, 0.0);
	glClearDepth(1.0);
}


void GLWidgets::paintGL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// set camera orientation
	gluLookAt(_cam[0], _cam[1], _cam[2], 0, 0, 0, 0, 1, 0);

	// camera rotation around x,y,z axes
	glRotated(_mRotX / 16.0, 1.0, 0.0, 0.0);
	glRotated(_mRotY / 16.0, 0.0, 1.0, 0.0);
	glRotated(_mRotZ / 16.0, 0.0, 0.0, 1.0);

	// zoom in/out
	// glScalef(_zoom, _zoom, 1.0f);

	// render all components in the env
	_glg._model.update(_sim._km._jointAngles);
	_glg.render();

	glFinish();
}


void GLWidgets::resizeGL(int width, int height) {
	int side = qMin(width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, width/height, 1, 100);
	glViewport((width - side) / 2, (height - side) / 2, side, side);
}


void GLWidgets::mousePressEvent(QMouseEvent* event) {
	_mLastPos = event->pos();
}


void GLWidgets::mouseMoveEvent(QMouseEvent* event) {
	int dx = event->x() - _mLastPos.x();
	int dy = event->y() - _mLastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(_mRotX + 8 * dy);
		setYRotation(_mRotY + 8 * dx);
	} else if (event->buttons() & Qt::RightButton) {
		setXRotation(_mRotX + 8 * dy);
		setZRotation(_mRotZ + 8 * dx);
	}

	_mLastPos = event->pos();
}