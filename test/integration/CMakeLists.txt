set(TEST_TYPE "INTEGRATION")

gz_get_sources(tests)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)
endif()

gz_build_tests(TYPE INTEGRATION SOURCES ${tests})

if(TARGET INTEGRATION_ExamplesBuild_TEST)
  add_dependencies(INTEGRATION_ExamplesBuild_TEST FAKE_INSTALL)

  set(_env_vars)
  list(APPEND _env_vars "CMAKE_PREFIX_PATH=${FAKE_INSTALL_PREFIX}:$ENV{CMAKE_PREFIX_PATH}")
  set_tests_properties(INTEGRATION_ExamplesBuild_TEST PROPERTIES
    ENVIRONMENT "${_env_vars}")
endif()
