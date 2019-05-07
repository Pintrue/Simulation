#ifndef GUI_QTWINDOW_HPP
#define GUI_QTWINDOW_HPP

#include "QtMainWindow.hpp"
#include "GLWidgets.hpp"
#include <QtWidgets/QWidget>
#include <QtWidgets/QSlider>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>


class QtWindow : public QWidget {
	Q_OBJECT

	public:
		QtWindow(QtMainWindow* mainWin);

	protected:
		void keyPressEvent(QKeyEvent* event) override;
	
	private slots:
		void execAction();

	private:
		QSlider* createSlider();

		GLWidgets* _glWidgets;

		QSlider* _xSlider;
		QSlider* _ySlider;
		QSlider* _zSlider;

		QLineEdit* _angleInputTxt;
		QPushButton* _execActionBtn;
		QLineEdit* _eePosInfoTxt;

		QtMainWindow* _mainWindow;
};

#endif