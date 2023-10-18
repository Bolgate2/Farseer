add_subdirectory(rocket)
add_subdirectory(sim)
add_subdirectory(geometry)

# linking libraries
target_link_libraries(sim geometry)
target_link_libraries(rocket sim)