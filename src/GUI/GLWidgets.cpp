#include "GLWidgets.hpp"
#include <QtGui/QMouseEvent>
#include <QtCore/QCoreApplication>


GLWidgets::GLWidgets(QWidget* parent) : QGLWidget(parent) {
	// initial camera angles
	_cam[0] = 40; _cam[1] = 40; _cam[2] = 10;
	_mRotX = 0; _mRotY = 0; _mRotZ = 0;

	_timer = new QTimer(this);
	_timer->setInterval(500);
	connect(_timer, SIGNAL(timeout()), this, SLOT(executeMovement()));

	/**
	 * TODO: intialize the graphics components here
	 **/

	// initialize the kinematics modules
	double ori[NUM_OF_JOINTS] = {0.0, 0.0, 0.0};
	_sim._km.init(ori);
	_sim._tjt.init(NUM_OF_JOINTS);
}


GLWidgets::~GLWidgets() {
	_timer->stop();
	delete _timer;

	cleanup();	// <- do the opengl finalization

	/**
	 * TODO: finalize the graphics components here
	 **/
	_sim._tjt.finish();
}


QSize GLWidgets::minimumSizeHint() const {
	return QSize(50, 50);
}


QSize GLWidgets::sizeHint() const {
	return QSize(500, 500);
}


static void normalizeAngle(int &angle) {
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


// void GLWidgets::setScale(int scale) {
// 	if (scale >= 0 && scale != _scale) {
// 		_scale = scale;
// 		emit scaleChanged(scale);
// 	}
// }


void GLWidgets::updateAngleInput(const QString& input) {
	_angleInput = input;
}


void GLWidgets::executeMovement() {
	// TODO: implementation
}


void GLWidgets::cleanup() {
	// TODO: implementation
}


void GLWidgets::initializeGL() {
	// TODO: implementation
}


void GLWidgets::paintGL() {
	// TODO: implementation
}


void GLWidgets::resizeGL(int width, int height) {
	// TODO: implementation
}


void GLWidgets::mousePressEvent(QMouseEvent *event) {
	_mLastPos = event->pos();
}


void GLWidgets::mouseMoveEvent(QMouseEvent *event) {
	int dx = event->x() - _mLastPos.x();
    int dy = event->y() - _mLastPos.y();

    if (event->buttons() & Qt::LeftButton)
    {
        setXRotation(_mRotX + 8 * dy);
        setYRotation(_mRotY + 8 * dx);
    }
    else if (event->buttons() & Qt::RightButton)
    {
        setXRotation(_mRotX + 8 * dy);
        setZRotation(_mRotZ + 8 * dx);
    }

    _mLastPos = event->pos();
}