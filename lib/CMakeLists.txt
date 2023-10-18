add_subdirectory(fmt)
add_subdirectory(uuid_v4)

# https://stackoverflow.com/questions/39383193/compiling-legacy-gcc-code-with-avx-vector-warnings
# required for uuid_v4, means compliation may not work on come older cpus
add_compile_options(-mavx2)