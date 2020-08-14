#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "BuildMountain.hpp"

#include <chrono>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow) {
	d_ui->setupUi(this);

	MountainOptions opts;
	opts.GridSize = 200;

	using clock = std::chrono::high_resolution_clock;
	auto start = clock::now();
	auto b = BuildMountain(opts);
	std::chrono::duration<float,std::milli> durationMS = clock::now() - start;

	std::cerr << "Model was built in " << durationMS.count() << "ms" << std::endl;

	d_ui->viewer->setModel(b.Mountain);
}

MainWindow::~MainWindow() {
	delete d_ui;
}
