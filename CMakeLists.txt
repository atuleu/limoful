project(limoful C CXX)

cmake_minimum_required(VERSION 3.11)

find_package(Qt5 COMPONENTS Core Widgets Xml OpenGL REQUIRED)
set(CMAKE_CXX_STANDARD 17)

find_library(QGLVIEWER_LIBRARY QGLViewer-qt5)

find_path(QGLVIEWER_INCLUDE_DIR QGLViewer/qglviewer.h)

if(NOT QGLVIEWER_LIBRARY OR NOT QGLVIEWER_INCLUDE_DIR)
	message(FATAL_ERROR "Could not find QGLViewer")
endif(NOT QGLVIEWER_LIBRARY OR NOT QGLVIEWER_INCLUDE_DIR)

find_path(LIBNOISE_INCLUDE_DIR libnoise/module/perlin.h)
find_library(LIBNOISE_LIBRARY noise)


if(NOT LIBNOISE_LIBRARY OR NOT LIBNOISE_INCLUDE_DIR)
	message(FATAL_ERROR "Could not find libnoise")
endif(NOT LIBNOISE_LIBRARY OR NOT LIBNOISE_INCLUDE_DIR)


include(FetchContent)
FetchContent_Declare(vcg
                     GIT_REPOSITORY https://github.com/cnr-isti-vclab/vcglib
                     GIT_TAG v1.0.1
                     )

FetchContent_GetProperties(vcg)
if(NOT vcg_POPULATED)
	FetchContent_Populate(vcg)
endif(NOT vcg_POPULATED)

find_package(CGAL COMPONENTS Core REQUIRED)

include_directories(${QGLVIEWER_INCLUDE_DIR}/QGLViewer
                    ${vcg_SOURCE_DIR}
                    ${vcg_SOURCE_DIR}/eigenlib
                    ${LIBNOISE_INCLUDE_DIR}
                    ${CGAL_INCLUDE_DIRS}
                    )

add_subdirectory(src)
