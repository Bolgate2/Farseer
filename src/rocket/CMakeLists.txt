find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(uuid_v4 REQUIRED)

add_executable(component_test main.cpp)

add_library(rocket "")

add_dependencies(component_test rocket)
target_link_libraries(component_test rocket)

add_subdirectory(components)
target_link_libraries(rocket Eigen3::Eigen)
target_link_libraries(rocket uuid_v4::uuid_v4)