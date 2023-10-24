cmake_minimum_required(VERSION 3.13)
project(Farseer VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
# setting up and linking dependencies

# THIS MUST COME BEFORE ADDING SUBDIRECTORIES
# need at least these 3 options enabled for uuid to compile
# https://github.com/crashoz/uuid_v4/issues/14
# only actually need mavx2, not the other 2 commands
add_compile_options(-mavx2)

# creating libraries
add_library(sim "")
add_library(rocket "")
add_library(utils "")

# adding src folder
add_subdirectory(src)
add_subdirectory(lib)

# defining exe
add_executable(Farseer main.cpp)

add_dependencies(utils eigen)
target_link_libraries(utils eigen)

add_dependencies(sim utils eigen fmt uuid_v4)
target_link_libraries(sim utils eigen fmt uuid_v4)
target_include_directories(sim PUBLIC
    "${PROJECT_SOURCE_DIR}/src/utils"
    )

add_dependencies(rocket sim utils eigen fmt uuid_v4)
target_link_libraries(rocket sim utils eigen fmt uuid_v4)
target_include_directories(rocket PUBLIC
    "${PROJECT_SOURCE_DIR}/src/sim"
    "${PROJECT_SOURCE_DIR}/src/utils"
    )

# linking src libraties to exe
target_link_libraries(Farseer eigen)
target_link_libraries(Farseer rocket)

target_include_directories(Farseer PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/src/rocket"
    "${PROJECT_SOURCE_DIR}/src/sim"
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
