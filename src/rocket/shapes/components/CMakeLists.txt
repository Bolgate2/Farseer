target_sources(rocket
    PRIVATE
        aeroComponentShape.hpp
        aeroComponentShape.cpp
        bodyComponentShape.hpp
        externalComponentShape.hpp
        internalComponentShape.hpp
        componentShape.hpp
        componentShape.cpp
)

add_subdirectory(bodyTube)
add_subdirectory(nosecone)
add_subdirectory(fins)