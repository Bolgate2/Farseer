find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(uuid_v4 REQUIRED)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif ()

add_executable(component_test main.cpp)

add_library(rocket "")

add_dependencies(component_test rocket)
target_link_libraries(component_test rocket)

add_subdirectory(components)
target_link_libraries(rocket Eigen3::Eigen)
target_link_libraries(rocket uuid_v4::uuid_v4)