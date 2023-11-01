add_subdirectory(bodyTube)
# add_subdirectory(propertyCalculator)

target_sources(rocket
    PRIVATE
        rocket.cpp
        rocket.hpp
        stage.cpp
        stage.hpp
        component.cpp
        component.hpp
        internalComponent.cpp
        internalComponent.hpp
        shapedInternalComponent.hpp
        shapedInternalComponent.cpp
        aeroComponent.cpp
        aeroComponent.hpp
        externalComponent.cpp
        externalComponent.hpp
        bodyComponent.cpp
        bodyComponent.hpp
)
