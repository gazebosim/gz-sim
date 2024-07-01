cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

project(GTestSetup)

# Find Gazebo
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

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
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  )
  include(GoogleTest)
  gtest_discover_tests(${TEST_TARGET})
endforeach()
