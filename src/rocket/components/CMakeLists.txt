target_sources(rocket
    PUBLIC
        component.hpp
        aerodynamicCalculator.hpp
        kinematicCalculator.cpp
)

add_subdirectory(bodyTube)