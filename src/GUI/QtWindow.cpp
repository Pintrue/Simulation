#include "QtWindow.hpp"
#include <QtGui/QKeyEvent>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>


QtWindow::QtWindow(QtMainWindow* mainWin)
					: _mainWindow(mainWin) {
	_glWidgets = new GLWidgets;

	// the three slider for camera rotation
	_xSlider = createSlider();
	_ySlider = createSlider();
	_zSlider = createSlider();

	_xSlider->setValue(15 * 16);
    _ySlider->setValue(345 * 16);
    _zSlider->setValue(0 * 16);

	connect(_xSlider, &QSlider::valueChanged, _glWidgets,
			&GLWidgets::setXRotation);
	connect(_glWidgets, &GLWidgets::xRotationChanged, _xSlider,
			&QSlider::setValue);
	connect(_ySlider, &QSlider::valueChanged, _glWidgets,
			&GLWidgets::setYRotation);
	connect(_glWidgets, &GLWidgets::yRotationChanged, _ySlider,
			&QSlider::setValue);
	connect(_zSlider, &QSlider::valueChanged, _glWidgets,
			&GLWidgets::setZRotation);
	connect(_glWidgets, &GLWidgets::zRotationChanged, _zSlider,
			&QSlider::setValue);

	// user input of angles to be executed
	_angleInputTxt = new QLineEdit("0.0, 0.0, 0.0");
	/**
	 * TODO: implement slot function
	 * 
	 * connect(_angleInputTxt, SIGNAL(textChanged(const QString &)),
						_glWidgets, SLOT(updateAngleInput(const QString &)));
	 **/
	connect(_angleInputTxt, SIGNAL(textChanged(const QString &)),
						_glWidgets, SLOT(updateAngleInput(const QString &)));


	// execute movement button
	_execActionBtn = new QPushButton(tr("Execute"));
	/**
	 * TODO: implement slot function
	 * 
	 * connect(_execActionBtn, SIGNAL(clicked()), _glWidgets, SLOT(executeMovement()));
	 **/

	// current end-effector position after update
	_eePosInfoTxt = new QLineEdit("N/P");
    _eePosInfoTxt->setReadOnly(true);
	_eePosInfoTxt->setStyleSheet(QString("color: blue; background: beige"));
	/**
	 * TODO: implement slot function
	 * 
	 * connect(_glWidgets, SIGNAL(updateEEPos(const QString &)), txSolutionInfo, SLOT(setText(const QString &))); 
	 **/
	connect(_glWidgets, SIGNAL(updateEEPos(const QString &)), _eePosInfoTxt, SLOT(setText(const QString &))); 
	

	QVBoxLayout* mainLayout = new QVBoxLayout;
    QHBoxLayout* container = new QHBoxLayout;
    QGridLayout* widgetLayout = new QGridLayout;

	container->addWidget(_glWidgets);
	container->addWidget(_xSlider);
	container->addWidget(_ySlider);
	container->addWidget(_zSlider);

	widgetLayout->addWidget(_angleInputTxt);
	widgetLayout->addWidget(_execActionBtn);
	widgetLayout->addWidget(_eePosInfoTxt);

	// Attach the container and widgetsLayout to the main layout
	container->addLayout(widgetLayout);
	mainLayout->addLayout(container);

	setLayout(mainLayout);

	setWindowTitle(tr("Simulation - Robotic Arm"));
}


void QtWindow::keyPressEvent(QKeyEvent* event) {
	if (event->key() == Qt::Key_Escape) {
		close();
	} else {
		QWidget::keyPressEvent(event);
	}
}


void QtWindow::execAction() {
	// TODO: implementation
	return;
}

QSlider* QtWindow::createSlider() {
	QSlider* slider = new QSlider(Qt::Vertical);

	slider->setRange(0, 360 * 16);
	slider->setSingleStep(16);
	slider->setPageStep(15 * 16);
	slider->setTickInterval(15 * 16);
	slider->setTickPosition(QSlider::TicksRight);

	return slider;
}