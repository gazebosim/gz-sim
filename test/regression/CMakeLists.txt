set(TEST_TYPE "REGRESSION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${tests} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

gz_build_tests(TYPE REGRESSION
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
