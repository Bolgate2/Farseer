add_library(sim "")

target_link_libraries(sim Eigen3::Eigen)
target_link_libraries(sim fmt)

target_sources(sim
    PRIVATE
        simulation.cpp
        simulation.hpp
        rocketInterface.cpp
        rocketInterface.hpp
        RealAtmos.hpp
        RealAtmos.cpp
        stateArray.hpp
        stateArray.cpp
)