# qt stuff
find_package(Qt6 REQUIRED COMPONENTS Widgets)
# https://doc.qt.io/qt-6/qt-standard-project-setup.html
# qt_standard_project_setup() # this is only in qt 6.3, ubuntu only has 6.2, instead do the below
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

qt_add_executable(helloworld
    mainwindow.cpp
    main.cpp
)

target_link_libraries(helloworld PRIVATE Qt6::Widgets)
#target_link_libraries(helloworld PRIVATE rocket)

set_target_properties(helloworld PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
