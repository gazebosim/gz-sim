include (${project_cmake_dir}/Utils.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set (sources
  Example.cc
)

set (gtest_sources
  Example_TEST.cc
)

ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${sources})
ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION})
