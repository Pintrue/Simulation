#include "QtMainWindow.hpp"
#include "QtWindow.hpp"

#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>


QtMainWindow::QtMainWindow() {
	QMenuBar* menuBar = new QMenuBar;

	// Drop down menu called "About"
	QMenu* about = menuBar->addMenu(tr("&Window"));
	QAction* newAct = new QAction(about);
	newAct->setText(tr("New Window"));
	about->addAction(newAct);
	connect(newAct, &QAction::triggered, this, &QtMainWindow::onAddNew);

	setMenuBar(menuBar);

	onAddNew();
}

void QtMainWindow::onAddNew() {
	if (!centralWidget()) {
		setCentralWidget(new QtWindow(this));
	} else {
		QMessageBox::information(0, tr("Stop"), tr("A window is in progress"));
	}
}