#include "MainWindow.hpp"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow) {
	d_ui->setupUi(this);
}

MainWindow::~MainWindow() {
	delete d_ui;
}
