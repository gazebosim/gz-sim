gz_add_component(eigen3 INTERFACE
  GET_TARGET_NAME component)

target_link_libraries(${component} INTERFACE Eigen3::Eigen)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
gz_get_libsources_and_unittests(sources gtest_sources)

# Build the unit tests
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources}
  LIB_DEPS ${component})

