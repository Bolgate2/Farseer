cmake_minimum_required(VERSION 3.13)
project(Farseer VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

# creating libraries
add_library(rocket "")
add_library(sim "")
add_library(geometry "")
# adding src folder
add_subdirectory(src)
add_subdirectory(lib)

# https://stackoverflow.com/questions/39383193/compiling-legacy-gcc-code-with-avx-vector-warnings
# required for uuid_v4, means compliation may not work on come older cpus
add_compile_options(-mavx2)

# defining exe
add_executable(Farseer main.cpp)
# linking lib libraries to src
target_link_libraries(rocket uuid_v4)
target_link_libraries(geometry fmt)
# linking src libraties to exe
target_link_libraries(Farseer rocket)

target_include_directories(Farseer PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/src/rocket"
    "${PROJECT_SOURCE_DIR}/src/sim"
    "${PROJECT_SOURCE_DIR}/src/geometry"
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
