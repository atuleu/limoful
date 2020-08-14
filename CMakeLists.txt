project(limoful C CXX)

cmake_minimum_required(VERSION 3.11)

find_package(Qt5 COMPONENTS Core Widgets Xml OpenGL REQUIRED)

set(CMAKE_CXX_STANDARD 17)

find_library(QGLVIEWER_LIBRARY QGLViewer-qt5)

find_path(QGLVIEWER_INCLUDE_DIR QGLViewer/qglviewer.h)

if(NOT QGLVIEWER_LIBRARY OR NOT QGLVIEWER_INCLUDE_DIR)
	message(FATAL_ERROR "Could not find QGLViewer")
endif(NOT QGLVIEWER_LIBRARY OR NOT QGLVIEWER_INCLUDE_DIR)

include_directories(${QGLVIEWER_INCLUDE_DIR}/QGLViewer)

add_subdirectory(src)
