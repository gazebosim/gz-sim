# Create model executable
set(model_executable gz-sim-model)
add_executable(${model_executable} model_main.cc ModelCommandAPI.cc)
target_link_libraries(${model_executable}
  gz-utils${GZ_UTILS_VER}::cli
  ${PROJECT_LIBRARY_TARGET_NAME}
)

install(
  TARGETS ${model_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/)

# Build the unit tests.
set(test_sources)

# Add systems that need a valid display here.
# \todo(anyone) Find a way to run these tests with a virtual display such Xvfb
# or Xdummy instead of skipping them
if(VALID_DISPLAY AND VALID_DRI_DISPLAY)
  list(APPEND test_sources ModelCommandAPI_TEST.cc)
else()
  message(STATUS
    "Skipping ModelCommandAPI tests because a valid display was not found")
endif()

# gz_TEST is not supported with multi config
# CMake generators, see also cmd/CMakeLists.txt
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT GENERATOR_IS_MULTI_CONFIG)
  list(APPEND test_sources gz_TEST.cc)
endif()

# Build unit tests if Gazebo tools is installed
if(BUILD_TESTING AND GZ_TOOLS_PROGRAM)
  gz_build_tests(TYPE UNIT
    SOURCES
      ${test_sources}
    LIB_DEPS
      gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
      gz-sim${PROJECT_VERSION_MAJOR}
    ENVIRONMENT
        GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  )
endif()

foreach(CMD_TEST
  UNIT_gz_TEST UNIT_ModelCommandAPI_TEST)

  if(NOT TARGET ${CMD_TEST})
    continue()
  endif()

  add_dependencies(${CMD_TEST}
    ${gz_lib_target}
    TestModelSystem
    TestSensorSystem
    TestWorldSystem
  )

  target_compile_definitions(${CMD_TEST}
    PRIVATE
      "GZ_PATH=\"${GZ_TOOLS_PROGRAM}\""
  )

  if(${CMD_TEST} STREQUAL UNIT_ModelCommandAPI_TEST)
    set(_env_vars)
    list(APPEND _env_vars "GZ_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>")
    list(APPEND _env_vars "GZ_IP=127.0.0.1")
    set_tests_properties(${CMD_TEST} PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endif()

  if(${CMD_TEST} STREQUAL UNIT_gz_TEST)
    # Running `gz sim` on macOS has problems when run with /usr/bin/ruby
    # due to System Integrity Protection (SIP). Try to find ruby from
    # homebrew as a workaround.
    if(APPLE)
      find_program(BREW_RUBY ruby HINTS /usr/local/opt/ruby/bin)
    endif()

    target_compile_definitions(${CMD_TEST}
      PRIVATE
        "BREW_RUBY=\"${BREW_RUBY} \""
    )

    set(_env_vars)
    list(APPEND _env_vars "GZ_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf")
    list(APPEND _env_vars "GZ_IP=127.0.0.1")
    list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:TestModelSystem>")

    set_tests_properties(${CMD_TEST} PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endif()

  # On Windows there is no RPATH, so an alternative way for tests for finding .dll libraries
  # in build directory in necessary. For regular tests, the trick is to place all libraries
  # and executables in a common CMAKE_RUNTIME_OUTPUT_DIRECTORY, so that the .dll are found
  # as they are in the same directory where the executable is loaded. For tests that are
  # launched via Ruby, this does not work, so we need to manually add CMAKE_RUNTIME_OUTPUT_DIRECTORY
  # to the PATH. This is done via the ENVIRONMENT_MODIFICATION that was added in CMake 3.22.
  if (WIN32)
    set_tests_properties(${CMD_TEST} PROPERTIES
      ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
  endif()
endforeach()

#===============================================================================
# Generate the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdsim0.rb
set(cmd_script_name "cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>_${cmd_script_name}")
set(cmd_script_configured "${CMAKE_CURRENT_BINARY_DIR}/${cmd_script_name}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
if(WIN32)
  set(plugin_location ${CMAKE_INSTALL_BINDIR})
else()
  set(plugin_location ${CMAKE_INSTALL_LIBDIR})
endif()

set(library_location "../../../${plugin_location}/$<TARGET_FILE_NAME:${gz_lib_target}>")

configure_file(
  "cmd${GZ_DESIGNATION}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/gz RENAME ${cmd_script_name})

set(gz_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: sim0.yaml
configure_file(
  "${GZ_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install( FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/gz/)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdsim0.rb
# The logic is valid only for single-config CMake generators, so no script is
# generated if a multiple-config CMake generator is used
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT GENERATOR_IS_MULTI_CONFIG)
  set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/gz/cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
  set(cmd_script_configured_test "${cmd_script_generated_test}.configured")

  # Set the library_location variable to the relative path to the library file
  # within the install directory structure.
  set(library_location "$<TARGET_FILE:${gz_lib_target}>")

  configure_file(
    "cmd${GZ_DESIGNATION}.rb.in"
    "${cmd_script_configured_test}"
    @ONLY)

  file(GENERATE
    OUTPUT "${cmd_script_generated_test}"
    INPUT  "${cmd_script_configured_test}")

  # Used only for internal testing.
  set(gz_library_path
    "${CMAKE_BINARY_DIR}/test/lib/ruby/gz/cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}")

  # Generate a configuration file for internal testing.
  # Note that the major version of the library is included in the name.
  # Ex: sim0.yaml
  configure_file(
    "${GZ_DESIGNATION}.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)
endif()

#===============================================================================
# Generate the ruby script for internal testing of model command.
# Note that the major version of the library is included in the name.
# Ex: cmdmodel0.rb
set(gz_model_ruby_path
  "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/gz/cmdmodel${PROJECT_VERSION_MAJOR}")
set(cmd_model_script_generated_test "${gz_model_ruby_path}.rb")
set(cmd_model_script_configured_test
  "${CMAKE_CURRENT_BINARY_DIR}/test_cmdmodel${PROJECT_VERSION_MAJOR}.rb.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(model_exe_location "$<TARGET_FILE:${model_executable}>")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured_test}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_model_script_generated_test}"
  INPUT "${cmd_model_script_configured_test}"
)

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: model0.yaml
configure_file(
  "model.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml.configured"
  @ONLY
)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/model${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml.configured"
)

#===============================================================================
# Used for the installed model command version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdmodel0.rb
set(cmd_model_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_model_script_configured "${cmd_model_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(model_exe_location "../../../${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${model_executable}>")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_model_script_generated}"
  INPUT "${cmd_model_script_configured}"
)
# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_model_script_generated} DESTINATION lib/ruby/gz)

# Used for the installed version.
set(gz_model_ruby_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmdmodel${PROJECT_VERSION_MAJOR}")

set(model_configured "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml")
configure_file(
  "model.yaml.in"
  ${model_configured}
  @ONLY)

install(FILES ${model_configured} DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/gz/)

#===============================================================================
# Bash completion

# Tack version onto and install the bash completion script
configure_file(
  "sim.bash_completion.sh"
    "${CMAKE_CURRENT_BINARY_DIR}/sim${PROJECT_VERSION_MAJOR}.bash_completion.sh" @ONLY)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sim${PROJECT_VERSION_MAJOR}.bash_completion.sh
  DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/gz/gz${GZ_TOOLS_VER}.completion.d)

configure_file(
  "model.bash_completion.sh"
    "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.bash_completion.sh" @ONLY)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.bash_completion.sh
  DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/gz/gz${GZ_TOOLS_VER}.completion.d)
