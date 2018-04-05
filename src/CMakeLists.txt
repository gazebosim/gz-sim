set (sources
  Entity.cc
)

set (gtest_sources
  Entity_TEST.cc
)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})
