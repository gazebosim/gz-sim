
# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})
