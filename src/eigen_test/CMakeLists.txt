find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(eigen_test main.cpp)

target_link_libraries(eigen_test Eigen3::Eigen)