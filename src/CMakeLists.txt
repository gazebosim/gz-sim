
# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Create the library target
# Turn on C++14 support for the library
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# graph namespace
add_subdirectory(graph)
