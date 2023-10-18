cmake_minimum_required(VERSION 3.13)
project(Farseer VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

# creating libraries
add_library(sim "")
add_dependencies(sim eigen fmt)
add_library(rocket "")
add_dependencies(rocket eigen fmt sim)

# adding src folder
add_subdirectory(src)
add_subdirectory(lib)

# defining exe
add_executable(Farseer main.cpp)

# setting up and linking dependencies
add_dependencies(sim eigen fmt uuid_v4)
target_link_libraries(sim eigen fmt uuid_v4)

add_dependencies(rocket sim eigen fmt uuid_v4)
target_link_libraries(rocket sim eigen fmt uuid_v4)
target_include_directories(rocket PUBLIC "${PROJECT_SOURCE_DIR}/src/sim")

# linking src libraties to exe
target_link_libraries(Farseer eigen)
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
