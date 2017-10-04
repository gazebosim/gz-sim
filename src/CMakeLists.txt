
# Turn on C++14 support for the library
set(CMAKE_CXX_STANDARD 14)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Create the library target
ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

# Create installation instructions for the library target. This must be called
# in the same scope that the target is created.
ign_install_library()

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# graph namespace
add_subdirectory(graph)