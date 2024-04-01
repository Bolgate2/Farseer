cmake_minimum_required(VERSION 3.16)

project(Farseer VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

# qt dir
add_subdirectory(src/app)
# eigen
add_subdirectory(src/eigen_test)
# rocket
add_subdirectory(src/rocket)