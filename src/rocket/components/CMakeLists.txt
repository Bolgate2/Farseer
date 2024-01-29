target_sources(rocket
    PUBLIC
        component.hpp
        aerodynamicCalculator.cpp
        kinematicCalculator.cpp
)

add_subdirectory(bodyTube)