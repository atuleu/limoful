set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


set(SRC_FILES main.cpp)

set(HDR_FILES )

set(UI_FILES )

set(RCC_FILES )


add_executable(limoful ${SRC_FILES}
                       ${HDR_FILES}
                       ${UI_FILES}
                       ${RCC_FILES}
                       )

target_link_libraries(limoful ${QGLVIEWER_LIBRARY}
                              Qt5::Widgets)
