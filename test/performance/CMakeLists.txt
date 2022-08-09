set(TEST_TYPE "PERFORMANCE")

set(tests
  each.cc
  level_manager.cc
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
  set_source_files_properties(${tests} COMPILE_FLAGS "/wd4251 /wd4146")
  set_source_files_properties(${exec}.cc COMPILE_FLAGS "/wd4251 /wd4146")
endif()

gz_build_tests(TYPE PERFORMANCE
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
)

add_executable(
  PERFORMANCE_${exec}
  ${exec}.cc
)

target_link_libraries(
  PERFORMANCE_${exec}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${PROJECT_VERSION_MAJOR}
    gz-sim${PROJECT_VERSION_MAJOR}-gui
)

