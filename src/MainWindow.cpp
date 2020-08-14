#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "BuildMountain.hpp"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow) {
	d_ui->setupUi(this);

	MountainOptions opts;
	opts.GridSize = 30;

	auto b = BuildMountain(opts);


	d_ui->viewer->setModel(b.Mountain);
}

MainWindow::~MainWindow() {
	delete d_ui;
}
