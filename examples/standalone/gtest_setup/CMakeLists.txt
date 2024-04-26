cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(GTestSetup)

# Find Gazebo
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(GTest REQUIRED)
if(NOT TARGET GTest::gtest_main AND TARGET gtest_main)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()

include(CTest)
# CTest defines "BUILD_TESTING" so that users can easily disable tests in a common way.
if(BUILD_TESTING)
  # Generate tests
  foreach(TEST_TARGET
    command_TEST
    gravity_TEST)

    add_executable(
      ${TEST_TARGET}
      ${TEST_TARGET}.cc
    )
    target_link_libraries(${TEST_TARGET}
      GTest::gtest_main
      gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    )
    include(GoogleTest)
    gtest_discover_tests(${TEST_TARGET})
  endforeach()
endif()
