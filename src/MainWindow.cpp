#include "MainWindow.hpp"
#include "ui_MainWindow.h"

#include "BuildMountain.hpp"

#include <QCloseEvent>
#include <QSettings>
#include <QStandardItemModel>
#include <QFileDialog>

#include <wrap/io_trimesh/export.h>

#include <chrono>
#include <fstream>

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, d_ui(new Ui::MainWindow)
	, d_curves(new QStandardItemModel(this)) {
	d_ui->setupUi(this);


	connect(d_curves,&QStandardItemModel::itemChanged,
	        this,&MainWindow::onItemChanged);

	d_curveDefinitions.insert({"exp",Curve::Exp()});

	for( const auto & [name,curve] : Curve::AllCurves() ) {
		d_curveDefinitions.insert({name.c_str(),curve});
	}


	d_octavesSliders = {d_ui->mOctave1Slider,
	                    d_ui->mOctave2Slider,
	                    d_ui->mOctave3Slider,
	                    d_ui->mOctave4Slider};

	d_noiseLabels = { d_ui->noise1Label,
	                  d_ui->noise2Label,
	                  d_ui->noise3Label,
	                  d_ui->noise4Label };
	d_curves->setHorizontalHeaderLabels({tr("type"),tr("angle")});
	d_ui->tableView->setModel(d_curves);
	d_ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
	loadSettings();

	connect(d_ui->mSmoothRadiusBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);


	connect(d_ui->mMinSlopeBot,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mMinSlopeTop,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);


	connect(d_ui->mMaxSlopeTop,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mMaxSlopeBot,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);


	connect(d_ui->mJumpBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mLowMinBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,&MainWindow::enableBuild);

	connect(d_ui->mCutBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
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

	for ( size_t i = 0; i < d_curves->rowCount(); ++i) {
		auto name = d_curves->item(i,0)->text();
		auto angle = d_curves->item(i,1)->text().toFloat();
		if ( d_curveDefinitions.count(name) > 0 ) {
			opts.Curves.push_back(d_curveDefinitions[name]);
			opts.Angles.push_back(angle);
		}
	}

	for ( const auto & s : d_octavesSliders ) {
		opts.OctaveWeights.push_back(float(s->value())/float(s->maximum()));
	}
	opts.Seed = d_ui->mSeedBox->value();
	opts.SlopeMinTop = d_ui->mMinSlopeTop->value();
	opts.SlopeMinBot = d_ui->mMinSlopeBot->value();
	opts.SlopeMaxTop = d_ui->mMaxSlopeTop->value();
	opts.SlopeMaxBot = d_ui->mMaxSlopeBot->value();
	opts.SmoothRadius = d_ui->mSmoothRadiusBox->value();
	opts.EdgeJump = d_ui->mJumpBox->value();
	opts.LowMin = d_ui->mLowMinBox->value();
	opts.BaseCut = d_ui->mCutBox->value();

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
	settings.setValue("m/minSlopeTop",d_ui->mMinSlopeTop->value());
	settings.setValue("m/maxSlopeTop",d_ui->mMaxSlopeTop->value());
	settings.setValue("m/minSlopeBot",d_ui->mMinSlopeBot->value());
	settings.setValue("m/maxSlopeBot",d_ui->mMaxSlopeBot->value());
	settings.setValue("m/smoothRadius",d_ui->mSmoothRadiusBox->value());
	settings.setValue("m/edgeJump",d_ui->mJumpBox->value());
	settings.setValue("m/lowMin",d_ui->mLowMinBox->value());
	settings.setValue("m/cut",d_ui->mCutBox->value());

	settings.setValue("m/seed",d_ui->mSeedBox->value());
	for ( size_t i = 0; i < d_octavesSliders.size(); ++i ) {
		settings.setValue(QString("m/octave%1").arg(i+1),d_octavesSliders[i]->value());
	}
	settings.setValue("c/count",d_curves->rowCount());
	for ( size_t i = 0; i < d_curves->rowCount(); ++i ) {
		auto name = d_curves->item(i,0)->text();
		auto angle = d_curves->item(i,1)->text().toDouble();
		settings.setValue(QString("c/%1/name").arg(i),name);
		settings.setValue(QString("c/%1/angle").arg(i),angle);
	}

}

void MainWindow::loadSettings() {
	QSettings settings;
	d_ui->gridSizeBox->setValue(settings.value("gridSize",30).toInt());
	d_ui->mMinSlopeTop->setValue(settings.value("m/minSlopeTop",70.0).toDouble());
	d_ui->mMaxSlopeTop->setValue(settings.value("m/maxSlopeTop",20.0).toDouble());
	d_ui->mMinSlopeBot->setValue(settings.value("m/minSlopeBot",20.0).toDouble());
	d_ui->mMaxSlopeBot->setValue(settings.value("m/maxSlopeBot",4.0).toDouble());
	d_ui->mSmoothRadiusBox->setValue(settings.value("m/smoothRadius",0.1).toDouble());

	d_ui->mJumpBox->setValue(settings.value("m/edgeJump",-0.01).toDouble());
	d_ui->mLowMinBox->setValue(settings.value("m/lowMin",-0.1).toDouble());
	d_ui->mCutBox->setValue(settings.value("m/cut",0.002).toDouble());

	d_ui->mSeedBox->setValue(settings.value("m/seed",42).toInt());
	d_ui->mOctave1Slider->setValue(settings.value("m/octave1",90).toInt());
	d_ui->mOctave2Slider->setValue(settings.value("m/octave2",40).toInt());
	d_ui->mOctave3Slider->setValue(settings.value("m/octave3",20).toInt());
	d_ui->mOctave4Slider->setValue(settings.value("m/octave4",10).toInt());

	for ( int i = 0; i < settings.value("c/count",0).toInt(); ++i ) {
		addCurve(settings.value(QString("c/%1/name").arg(i),"exp").toString(),
		         settings.value(QString("c/%1/angle").arg(i),0.0).toDouble());
	}

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
	auto filename = QFileDialog::getSaveFileName(this,"Save as STL file","","STL Files(*.stl)");

	if ( filename.isEmpty() ) {
		return;
	}

	vcg::tri::io::ExporterSTL<LIFMesh>::Save(*d_mountain.Mountain,filename.toUtf8().constData());
}


void MainWindow::updateModelHigh() {
	buildModel(d_ui->gridSizeBox->value());
}
void MainWindow::updateModelLow() {
	buildModel(100);
}


QComboBox * MainWindow::curveBox() {
	auto res = new QComboBox(this);
	for ( const auto & [name,curve] : d_curveDefinitions ) {
		res->addItem(name);
	}
	return res;
}


void MainWindow::addCurve(const QString & name,
                          float angle) {
	auto typeItem = new QStandardItem(name);
	auto angleItem = new QStandardItem(QString::number(angle));
	auto spinBox = new QDoubleSpinBox(this);
	spinBox->setMinimum(0);
	spinBox->setMaximum(360);
	spinBox->setSingleStep(5);
	spinBox->setValue(angle);
	auto cBox = curveBox();
	cBox->setCurrentIndex(cBox->findText(name));
	connect(spinBox,static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
	        this,[angleItem](double value) {
		             angleItem->setText(QString::number(value));
	             });

	connect(cBox,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
	        this,[typeItem,cBox]() {
		             typeItem->setText(cBox->currentText());
	             });


	d_curves->insertRow(d_curves->rowCount(),{typeItem,angleItem});
	d_ui->tableView->setIndexWidget(d_curves->index(d_curves->rowCount()-1,0),cBox);
	d_ui->tableView->setIndexWidget(d_curves->index(d_curves->rowCount()-1,1),spinBox);
}


void MainWindow::on_addButton_clicked() {
	float angle = 0.0;
	if ( d_curves->rowCount() != 0 ) {
		angle = (360 + d_curves->item(d_curves->rowCount()-1,1)->text().toFloat()) / 2.0;
	}
	addCurve("exp",angle);
	enableBuild();
}

void MainWindow::on_removeButton_clicked() {
	d_curves->removeRows(d_curves->rowCount()-1,1);
}

void MainWindow::onItemChanged(QStandardItem * item) {
	enableBuild();
}
