# creating qt executable for include
# qt stuff
find_package(Qt6 REQUIRED COMPONENTS Widgets)
# https://doc.qt.io/qt-6/qt-standard-project-setup.html
# qt_standard_project_setup() # this is only in qt 6.3, ubuntu only has 6.2, instead do the below
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

qt_add_executable(inc_test
    main.cpp
)

target_link_libraries(inc_test PRIVATE Qt6::Widgets Eigen3::Eigen uuid_v4::uuid_v4 nlohmann_json::nlohmann_json)

set_target_properties(inc_test PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)