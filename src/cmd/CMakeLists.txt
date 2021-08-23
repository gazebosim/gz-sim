# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

add_library(ign STATIC ign.cc)

target_include_directories(ign PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ign PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
)

set(server_executable ign-gazebo-server)
add_executable(${server_executable} server_main.cc)
target_link_libraries(${server_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

set(gui_executable ign-gazebo-gui)
add_executable(${gui_executable} gui_main.cc)
target_link_libraries(${gui_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
)

set(model_executable ign-model)
add_executable(${model_executable} model_main.cc)
target_link_libraries(${model_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

install(
  TARGETS
  ${gui_executable}
  ${server_executable}
  ${model_executable}
  DESTINATION
  ${IGN_LIB_INSTALL_DIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources}
  TEST_LIST test_list
  LIB_DEPS ${EXTRA_TEST_LIB_DEPS}
)

# Running `ign gazebo` on macOS has problems when run with /usr/bin/ruby
# due to System Integrity Protection (SIP). Try to find ruby from
# homebrew as a workaround.
if (APPLE)
  find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
endif()

foreach(test ${test_list})
  target_link_libraries(${test} ign)

  # Inform each test of its output directory so it knows where to call the
  # auxiliary files from. Using a generator expression here is useful for
  # multi-configuration generators, like Visual Studio.
  target_compile_definitions(${test} PRIVATE
    "IGN_TEST_LIBRARY_PATH=\"$<TARGET_FILE_DIR:${PROJECT_LIBRARY_TARGET_NAME}>\"")


  message("Test name: " ${test})

endforeach()

set(_env_vars)
list(APPEND _env_vars "IGN_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf")
list(APPEND _env_vars "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:TestModelSystem>")

if (TARGET UNIT_ign_TEST)
  set_tests_properties(
    UNIT_ign_TEST
    PROPERTIES
    ENVIRONMENT
      "${_env_vars}"
  )
  target_compile_definitions(UNIT_ign_TEST PRIVATE
    "IGN_PATH=\"${IGN_TOOLS_PROGRAM}\"")
  target_compile_definitions(UNIT_ign_TEST PRIVATE
    "BREW_RUBY=\"${BREW_RUBY}\"")
endif()

if (TARGET UNIT_ModelCommandAPI_TEST)
  set_tests_properties(
    UNIT_ModelCommandAPI_TEST
    PROPERTIES
    ENVIRONMENT
      "${_env_vars}"
  )
  target_compile_definitions(UNIT_ModelCommandAPI_TEST PRIVATE
    "IGN_PATH=\"${IGN_TOOLS_PROGRAM}\"")
  target_compile_definitions(UNIT_ModelCommandAPI_TEST PRIVATE
    "BREW_RUBY=\"${BREW_RUBY}\"")
endif()

########################################################################################################################################################################################################
#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdmodel5.rb
set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test "${CMAKE_CURRENT_BINARY_DIR}/test_cmdmodel${PROJECT_VERSION_MAJOR}.rb.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(model_exe_location "$<TARGET_FILE:${model_executable}>")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_script_configured_test}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}"
)

#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdmodel5.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdmodel${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: model5.yaml
configure_file(
  "model.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml.configured" @ONLY)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/model${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml.configured")

# Used for the installed version.
set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdmodel${PROJECT_VERSION_MAJOR}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: model5.yaml
configure_file(
  "model.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/ignition)

########################################################################################################################################################################################################
#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdgazeboserver.rb
set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test "${CMAKE_CURRENT_BINARY_DIR}/test_cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(gazebo_server_exe_location "$<TARGET_FILE:${server_executable}>")
set(gazebo_gui_exe_location "$<TARGET_FILE:${gui_executable}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured_test}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}"
)

#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdgazeboserver5.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(gazebo_server_exe_location "../../../${CMAKE_INSTALL_LIBDIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${server_executable}>")

configure_file(
  "cmd${IGN_DESIGNATION}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: launch5.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured" @ONLY)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured")

# Used for the installed version.
set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmd${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: launch5.yaml
configure_file(
  "${IGN_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/ignition)
