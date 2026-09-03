set(TEST_TYPE "PERFORMANCE")

set(tests
  each.cc
  level_manager.cc
)

set(tests_needing_display
  sensors_system.cc
)

set(exec
  sdf_runner
)

link_directories(${PROJECT_BINARY_DIR}/test)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(
    ${tests} ${tests_needing_display}
    COMPILE_FLAGS "/wd4251 /wd4146")
  set_source_files_properties(${exec}.cc COMPILE_FLAGS "/wd4251 /wd4146")
endif()

gz_build_tests(TYPE PERFORMANCE
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

gz_build_tests(TYPE PERFORMANCE
  SOURCES
    ${tests_needing_display}
  TEST_LIST display_test_targets
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if(BUILD_TESTING AND (NOT VALID_DISPLAY OR NOT VALID_DRI_DISPLAY))
  message(STATUS
    "Disabling these PERFORMANCE tests because a valid display was not found:")
  foreach(test_target ${display_test_targets})
    message(STATUS " ${test_target}")
    set_tests_properties(${test_target} PROPERTIES DISABLED TRUE)
    if(TEST check_${test_target})
      set_tests_properties(check_${test_target} PROPERTIES DISABLED TRUE)
    endif()
  endforeach()
endif()

add_executable(
  PERFORMANCE_${exec}
  ${exec}.cc
)

target_link_libraries(
  PERFORMANCE_${exec}
    gz-common::gz-common
    gz-sim
)

if(TARGET PERFORMANCE_sensors_system)
  target_link_libraries(PERFORMANCE_sensors_system
    gz-rendering::gz-rendering
  )
endif()
