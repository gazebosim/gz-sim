
# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
gz_get_libsources_and_unittests(sources gtest_sources)

# Create the library target
gz_create_core_library(SOURCES ${sources} CXX_STANDARD ${c++standard})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)

# Build the unit tests
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# graph namespace
add_subdirectory(graph)

# Bindings subdirectories
if (${pybind11_FOUND})
  add_subdirectory(python_pybind11)
endif()
add_subdirectory(ruby)
