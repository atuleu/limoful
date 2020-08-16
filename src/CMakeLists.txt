set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_definitions("-Wno-ignored-attributes"
	            "-Wno-deprecated-declarations")

set(SRC_FILES main.cpp
              MainWindow.cpp
              Viewer.cpp
              Curve.cpp
              BuildMountain.cpp
              PolarGrid.cpp
              Meshifier.cpp
              )

set(HDR_FILES MainWindow.hpp
              Viewer.hpp
              Curve.hpp
              BuildMountain.hpp
              PolarGrid.hpp
              Meshifier.hpp
              )

set(UI_FILES MainWindow.ui)

set(RCC_FILES )

include_directories(${PROJECT_BINARY_DIR}/src
                    ${PROJECT_SOURCE_DIR}/src)

add_executable(limoful ${SRC_FILES}
                       ${HDR_FILES}
                       ${UI_FILES}
                       ${RCC_FILES}
                       )

target_link_libraries(limoful ${QGLVIEWER_LIBRARY}
                              Qt5::Xml
                              Qt5::Widgets
                              Qt5::OpenGL
                              -lGL
                              ${LIBNOISE_LIBRARY}
                              CGAL::CGAL
                              )
