#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "BuildMountain.hpp"

#include <chrono>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow) {
	d_ui->setupUi(this);

	connect(d_ui->bpRadiusBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->bpClusterBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->bpAngleBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->gridSizeBox,static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->buildButton,&QPushButton::clicked,
	        this,&MainWindow::updateModel);

	updateModel();
}

MainWindow::~MainWindow() {
	delete d_ui;
}

void MainWindow::enableBuild() {
	d_ui->buildButton->setEnabled(true);
}

void MainWindow::updateModel() {
	MountainOptions opts;
	opts.GridSize = d_ui->gridSizeBox->value();
	opts.BallPivotingRadius = d_ui->bpRadiusBox->value() ;
	opts.BallPivotingCluster = d_ui->bpClusterBox->value();
	opts.BallPivotingAngle = d_ui->bpAngleBox->value();
	using clock = std::chrono::high_resolution_clock;
	auto start = clock::now();
	auto b = BuildMountain(opts);

	std::chrono::duration<float,std::milli> durationMS = clock::now() - start;

	std::cerr << "Model was built in " << durationMS.count() << "ms" << std::endl;

	d_ui->viewer->setMesh(b.Mountain);

	d_ui->buildButton->setEnabled(false);
}
