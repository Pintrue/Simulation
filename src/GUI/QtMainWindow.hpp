#ifndef GUI_QTMAINWINDOW_HPP
#define GUI_QTMAINWINDOW_HPP

#include <QtWidgets/QMainWindow>

class QtMainWindow : public QMainWindow {
	Q_OBJECT

	public:
		QtMainWindow();

	private slots:
		void onAddNew();
};

#endif