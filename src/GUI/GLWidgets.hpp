#ifndef GUI_GLWIDGETS_HPP
#define GUI_GLWIDGETS_HPP

#include <QtWidgets/QOpenGLWidget>
#include <QtCore/QTimer>
#include <QtGui/QOpenGLFunctions>
#include "../Sim.hpp"
#include "../model/KinematicsModel.hpp"
#include "GLGraphics.hpp"


class GLWidgets : public QOpenGLWidget, protected QOpenGLFunctions {
	Q_OBJECT
	public:
	    GLWidgets(QWidget* parent = 0);
	    ~GLWidgets();

	    QSize minimumSizeHint() const override;
	    QSize sizeHint() const override;

		Sim getSim();
		void setSim(Sim sim);
		void setJointAngle(KDL::JntArray ja);
		// KDL::JntArray _ja;

	public slots:
	    void setXRotation(int angle);
	    void setYRotation(int angle);
	    void setZRotation(int angle);
		// void setScale(int scale);

		void updateAngleInput(const QString& input);
		void enableTraj(bool on);
		void execAction();
		void plainAction();
		void plainActionObj();
		void trajAction();
		void trajNextTimeStep();
		void moveByActionPath();
		void actPathNextTimeStep();

	    void cleanup();

	signals:
	    void xRotationChanged(int angle);
	    void yRotationChanged(int angle);
	    void zRotationChanged(int angle);
		// void scaleChanged(int scale);

	    void updateEEPos(const QString &);

	protected:
	    void initializeGL() override;
	    void paintGL() override;
	    void resizeGL(int width, int height) override;
	    void mousePressEvent(QMouseEvent *event) override;
	    void mouseMoveEvent(QMouseEvent *event) override;
		

	private:
		QString _angleInput;
		// double _jointAngles[NUM_OF_JOINTS];
		KDL::JntArray _ja;
		bool _trajOn;
		bool _hasObj;
		QTimer* _timer;

		float _cam[3];
    	int _mRotX, _mRotY, _mRotZ, _scale;
    	// QPoint _lastEEPos;

		QPoint _mLastPos;

		GLGraphics _glg;
		Sim _sim;

		int _actionsIter;
};

#endif