cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

project(GTestSetup)

# Find Gazebo
set(IGN_GAZEBO_VER 5)
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

# Fetch and configure GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(Dart)

# Generate tests
foreach(TEST_TARGET
  command_TEST
  gravity_TEST)

  add_executable(
    ${TEST_TARGET}
    ${TEST_TARGET}.cc
  )
  target_link_libraries(${TEST_TARGET}
    gtest_main
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  )
  include(GoogleTest)
  gtest_discover_tests(${TEST_TARGET})
endforeach()
