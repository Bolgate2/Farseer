# adding subdirectories
add_subdirectory(geometry)
add_subdirectory(sim)
add_subdirectory(rocket)

# creating rocket dependency
add_dependencies(rocket sim)
target_include_directories(rocket PUBLIC "sim" )
target_link_libraries(rocket sim)

# creating simulator dependency
add_dependencies(sim geometry)
target_include_directories(sim PUBLIC "geometry")
target_link_libraries(sim geometry)
