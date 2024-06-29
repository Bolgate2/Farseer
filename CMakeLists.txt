cmake_minimum_required(VERSION 3.16)

project(Farseer VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)


# adding json package
add_subdirectory(include/json)

# adding uuid package, this requires some SIMD flags to be set
add_subdirectory(include/uuid_v4)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif ()

# including eigen
add_subdirectory(include/eigen)
# including fmt
add_subdirectory(include/fmt)
# including lru cache header directly in the rocket package


# qt dir
add_subdirectory(src/app)
# simulation
add_subdirectory(src/sim)
# rocket
add_subdirectory(src/rocket)
# include test
add_subdirectory(src/test_includes)