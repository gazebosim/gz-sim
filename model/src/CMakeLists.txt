ign_get_libsources_and_unittests(sources gtest_sources)
list(APPEND sources cmd/ModelCommandAPI.cc)

ign_add_component(model SOURCES ${sources} GET_TARGET_NAME model_lib_target)


if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${sources} ${gtest_sources} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

# Unit tests
ign_build_tests(
  TYPE "UNIT"
  SOURCES ${gtest_sources}
  LIB_DEPS ${EXTRA_TEST_LIB_DEPS}
  TEST_LIST model_tests
)

if(TARGET UNIT_ModelCommandAPI_TEST)

  target_compile_definitions(UNIT_ModelCommandAPI_TEST PRIVATE
      "IGN_PATH=\"${IGNITION-TOOLS_BINARY_DIRS}\"")

  target_compile_definitions(UNIT_ModelCommandAPI_TEST PRIVATE
  "PROJECT_SOURCE_PATH=\"${PROJECT_SOURCE_DIR}\"")

endif()

if(NOT WIN32)
  add_subdirectory(cmd)
endif()
