#ifndef GUI_QTWINDOW_HPP
#define GUI_QTWINDOW_HPP

#include "QtMainWindow.hpp"
#include "GLWidgets.hpp"
#include <QtWidgets/QWidget>
#include <QtWidgets/QSlider>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>


class QtWindow : public QWidget {
	Q_OBJECT

	public:
		QtWindow(QtMainWindow* mainWin);

	protected:
		void keyPressEvent(QKeyEvent* event) override;
	
	// private slots:
	// 	void execAction();

	private:
		QSlider* createSlider();

		GLWidgets* _glWidgets;

		QSlider* _xSlider;
		QSlider* _ySlider;
		QSlider* _zSlider;

		QLineEdit* _angleInputTxt;
		QRadioButton* _enableTrajBtn;
		QPushButton* _execActionBtn;
		QLineEdit* _eePosInfoTxt;

		QtMainWindow* _mainWindow;
};

#endif