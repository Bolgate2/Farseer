target_sources(rocket
    PRIVATE
        shape.hpp
        shape.cpp
        axisymmetricShape.hpp
        axisymmetricShape.cpp
        aeroShapeFunctions.hpp
        hollowShape.hpp
        hollowShape.cpp
        hollowAxisymmetricShape.hpp
        cylinder.hpp
        cylinder.cpp
)

add_subdirectory(nosecone)