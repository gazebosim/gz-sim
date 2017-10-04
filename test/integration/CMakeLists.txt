set(TEST_TYPE "INTEGRATION")

set(tests
)

link_directories(${PROJECT_BINARY_DIR}/test)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  set(PATH_TO_PROJECT_LIBRARY )
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)
endif()

ign_build_tests(${tests})
