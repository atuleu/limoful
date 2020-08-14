#include <QApplication>

#include "MainWindow.hpp"

int main(int argc, char ** argv) {
	QCoreApplication::setOrganizationName("Alexandre Tuleu");
	QCoreApplication::setOrganizationDomain("tuleu.lighting");
	QCoreApplication::setApplicationName("Les Immobilitées Fulgurantes");
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);


	QApplication limoful(argc,argv);

	MainWindow main;
	main.show();


	return limoful.exec();
}
