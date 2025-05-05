cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(GTestSetup)

# Find Gazebo
find_package(gz-sim REQUIRED)

# Fetch and configure GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  # Version 1.14. Use commit hash to prevent tag relocation
  URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

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
    gz-sim::gz-sim
  )

  include(GoogleTest)
  if (APPLE)
    gtest_discover_tests(${TEST_TARGET} DISCOVERY_MODE PRE_TEST)
  else()
    gtest_discover_tests(${TEST_TARGET})
  endif()
endforeach()
