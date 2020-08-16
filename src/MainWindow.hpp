#pragma once

#include <QMainWindow>

#include "BuildMountain.hpp"

namespace Ui {
class MainWindow;
}

class QAbstractSlider;
class QLabel;

class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
	void update3DLayer();
	void updateModelHigh();
	void updateModelLow();
	void enableBuild();

	void loadSettings();
	void saveSettings();

	void on_actionExport_triggered();
private:
	void buildModel(size_t gridSize);

	Ui::MainWindow * d_ui;

	Mountain d_mountain;

	std::vector<QAbstractSlider*> d_octavesSliders;
	std::vector<QLabel*>          d_noiseLabels;

};
