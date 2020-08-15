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
	void updateModel();
	void enableBuild();

	void loadSettings();
	void saveSettings();
private:

	Ui::MainWindow * d_ui;

	Mountain d_mountain;

	std::vector<QAbstractSlider*> d_octavesSliders;
	std::vector<QLabel*>          d_noiseLabels;

};
