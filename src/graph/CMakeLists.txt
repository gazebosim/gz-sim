
# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
gz_get_libsources_and_unittests(sources gtest_sources)

# Build the unit tests
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources})
