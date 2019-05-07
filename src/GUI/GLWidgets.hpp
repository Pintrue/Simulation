#ifndef GUI_GLWIDGETS_HPP
#define GUI_GLWIDGETS_HPP

#include <QtWidgets/QOpenGLWidget>
#include <QtCore/QTimer>
#include "../Sim.hpp"
// delete these...
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLBuffer>
#include <QtGui/QMatrix4x4>
#include <QtGui/QOpenGLShaderProgram>
#include "logo.h"

class GLWidgets : public QOpenGLWidget, protected QOpenGLFunctions {
	Q_OBJECT
	public:
	    GLWidgets(QWidget* parent = 0);
	    ~GLWidgets();

		// del these...
		static bool isTransparent() { return m_transparent; }
		static void setTransparent(bool t) { m_transparent = t; }

	    QSize minimumSizeHint() const override;
	    QSize sizeHint() const override;

	public slots:
	    void setXRotation(int angle);
	    void setYRotation(int angle);
	    void setZRotation(int angle);
		// void setScale(int scale);

		void updateAngleInput(const QString& input);
		void executeMovement();

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
		QTimer* _timer;

		float _cam[3];
    	int _mRotX, _mRotY, _mRotZ, _scale;
    	QPoint _lastEEPos;

		QPoint _mLastPos;

		/**
		 * TODO: put OpenGL components here
		 * 
		 * Graphics _graphics;
		 **/
		Sim _sim;

		// del these...
	    void setupVertexAttribs();
		bool m_core;
		Logo m_logo;
    	QOpenGLVertexArrayObject m_vao;
    	QOpenGLBuffer m_logoVbo;
    	QOpenGLShaderProgram *m_program;
    	int m_projMatrixLoc;
    	int m_mvMatrixLoc;
    	int m_normalMatrixLoc;
    	int m_lightPosLoc;
    	QMatrix4x4 m_proj;
    	QMatrix4x4 m_camera;
    	QMatrix4x4 m_world;
    	static bool m_transparent;
};

#endif