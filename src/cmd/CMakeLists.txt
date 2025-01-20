# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
gz_get_libsources_and_unittests(sources gtest_sources)

add_library(gz STATIC gz.cc)
target_link_libraries(gz
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
)

set(model_executable gz-sim-model)
add_executable(${model_executable} model_main.cc)
target_link_libraries(${model_executable}
  gz
  gz-utils${GZ_UTILS_VER}::cli
  ${PROJECT_LIBRARY_TARGET_NAME}
)

install(
  TARGETS ${model_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/)

# Build the unit tests.
gz_build_tests(TYPE UNIT
  SOURCES
    ../ModelCommandAPI_TEST.cc
  LIB_DEPS
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    ${EXTRA_TEST_LIB_DEPS}
    gz-sim${PROJECT_VERSION_MAJOR}
  ENVIRONMENT
      GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

target_compile_definitions(UNIT_ModelCommandAPI_TEST
  PRIVATE
    "GZ_PATH=\"${GZ_TOOLS_PROGRAM}\"")

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
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdsim0.rb
# The logic is valid only for single-config CMake generators, so no script is
# generated if a multiple-config CMake geneator is used
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
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# The logic is valid only for single-config CMake generators, so no script is
# generated if a multiple-config CMake geneator is used
if(NOT GENERATOR_IS_MULTI_CONFIG)
  set(cmd_model_script_generated_test
    "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/gz/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
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

  # Used for internal testing.
  set(gz_model_ruby_path "${cmd_model_script_generated_test}")

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
endif()

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
