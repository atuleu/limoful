#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "BuildMountain.hpp"

#include <QCloseEvent>
#include <QSettings>

#include <chrono>
#include <fstream>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow) {
	d_ui->setupUi(this);

	d_octavesSliders = {d_ui->mOctave1Slider,
	                    d_ui->mOctave2Slider,
	                    d_ui->mOctave3Slider,
	                    d_ui->mOctave4Slider};

	d_noiseLabels = { d_ui->noise1Label,
	                  d_ui->noise2Label,
	                  d_ui->noise3Label,
	                  d_ui->noise4Label };

	loadSettings();

	connect(d_ui->bpRadiusBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->bpClusterBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->bpAngleBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mMinSlopeBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mMaxSlopeBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->gridSizeBox,static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
	        this,[this]() {
		             d_ui->buildButton->setEnabled(true);
	             });

	connect(d_ui->mSeedBox,static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->buildButton,&QPushButton::clicked,
	        this,&MainWindow::updateModelHigh);

	for ( const auto & s : d_octavesSliders ) {
		connect(s,&QAbstractSlider::valueChanged,
		        this,&MainWindow::enableBuild);
	}

	connect(d_ui->threeDLayerBox,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
	        this,&MainWindow::update3DLayer);



	updateModelLow();
}

MainWindow::~MainWindow() {
	delete d_ui;
}

void MainWindow::enableBuild() {
	d_ui->buildButton->setEnabled(true);
	updateModelLow();
}

void MainWindow::buildModel(size_t gridSize) {
	MountainOptions opts;
	opts.GridSize = gridSize;

	opts.Curves = {Curve::Exp(),Curve::Exp(),Curve::Exp()};
	opts.Angles = {0.0,120.0,240.0};

	for ( const auto & s : d_octavesSliders ) {
		opts.OctaveWeights.push_back(float(s->value())/float(s->maximum()));
	}
	opts.Seed = d_ui->mSeedBox->value();
	opts.SlopeMinAngle = d_ui->mMinSlopeBox->value();
	opts.SlopeMaxAngle = d_ui->mMaxSlopeBox->value();

	using clock = std::chrono::high_resolution_clock;
	auto start = clock::now();
	d_mountain = BuildMountain(opts);

	std::chrono::duration<float,std::milli> durationMS = clock::now() - start;

	std::cerr << "Model was built in " << durationMS.count() << "ms" << std::endl;

	update3DLayer();

	for ( size_t i = 0;
	      i < std::min(d_noiseLabels.size(),
	                   d_mountain.Noises.size());
	      ++i ) {
		auto l = d_noiseLabels[i];
		auto p = QPixmap::fromImage(d_mountain.Noises[i]).scaled(l->width(),l->height(),Qt::KeepAspectRatio);
		l->setPixmap(p);

	}
}


void MainWindow::closeEvent(QCloseEvent *event) {
	saveSettings();
	event->accept();
}

void MainWindow::saveSettings() {
	QSettings settings;
	settings.setValue("gridSize",d_ui->gridSizeBox->value());
	settings.setValue("bp/radius",d_ui->bpRadiusBox->value());
	settings.setValue("bp/cluster",d_ui->bpClusterBox->value());
	settings.setValue("bp/angle",d_ui->bpAngleBox->value());
	settings.setValue("m/minSlope",d_ui->mMinSlopeBox->value());
	settings.setValue("m/maxSlope",d_ui->mMaxSlopeBox->value());
	settings.setValue("m/seed",d_ui->mSeedBox->value());
	for ( size_t i = 0; i < d_octavesSliders.size(); ++i ) {
		settings.setValue(QString("m/octave%1").arg(i+1),d_octavesSliders[i]->value());
	}
}

void MainWindow::loadSettings() {
	QSettings settings;
	d_ui->gridSizeBox->setValue(settings.value("gridSize",30).toInt());
	d_ui->mMinSlopeBox->setValue(settings.value("m/minSlope",5.0).toDouble());
	d_ui->mMaxSlopeBox->setValue(settings.value("m/maxSlope",40.0).toDouble());
	d_ui->bpRadiusBox->setValue(settings.value("bp/radius",0.07).toDouble());
	d_ui->bpClusterBox->setValue(settings.value("bp/cluster",0.2).toDouble());
	d_ui->bpAngleBox->setValue(settings.value("bp/angle",90.0).toDouble());
	d_ui->mSeedBox->setValue(settings.value("m/seed",42).toInt());
	d_ui->mOctave1Slider->setValue(settings.value("m/octave1",90).toInt());
	d_ui->mOctave2Slider->setValue(settings.value("m/octave2",40).toInt());
	d_ui->mOctave3Slider->setValue(settings.value("m/octave3",20).toInt());
	d_ui->mOctave4Slider->setValue(settings.value("m/octave4",10).toInt());
}

void MainWindow::update3DLayer() {
	switch(d_ui->threeDLayerBox->currentIndex()) {
	case 0:
		d_ui->viewer->setMesh(d_mountain.Mountain);
		break;
	case 1:
		d_ui->viewer->setMesh(d_mountain.Maximum);
		break;
	case 2:
		d_ui->viewer->setMesh(d_mountain.Minimum);
		break;
	default:
		d_ui->viewer->setMesh(d_mountain.Mountain);
	}
}

void MainWindow::on_actionExport_triggered() {
	std::cerr << "coucou" << std::endl;
	std::vector<Eigen::Vector3f> points;
	points.reserve(2*d_mountain.Points.size() );

	for ( const auto & p : d_mountain.Points) {
		if ( p.z() < 0.0 ) { continue;}
		points.push_back(p);
		//		points.push_back(Eigen::Vector3f(p.x(),p.y(),0));
	}

	std::ofstream file("/tmp/moutain.xyz");
	file << points.size() << std::endl;
	for ( const auto & p : points ) {
		file << p.x() << " " << p.y() << " " << p.z() << std::endl;
	}
}


void MainWindow::updateModelHigh() {
	buildModel(d_ui->gridSizeBox->value());
}
void MainWindow::updateModelLow() {
	buildModel(100);
}
