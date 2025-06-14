# Create model executable
set(model_executable gz-sim-model)
add_executable(${model_executable} model_main.cc ModelCommandAPI.cc)
target_link_libraries(${model_executable}
  gz-utils::cli
  ${PROJECT_LIBRARY_TARGET_NAME}
)

install(
  TARGETS ${model_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

# Create sim utilities library
add_library(gz STATIC gz.cc)
target_link_libraries(gz
  ${PROJECT_LIBRARY_TARGET_NAME}
  Backward::Object
  gz-common::gz-common
  gz-utils::cli
  gz-sim
)

if(ENABLE_GUI)
  target_compile_definitions(gz PRIVATE WITH_GUI)
  target_link_libraries(gz gz-sim-gui)
endif()

# Create server executable by default
set(server_executable gz-sim-server)
add_executable(${server_executable} sim_main.cc gz.cc)
target_link_libraries(${server_executable}
  PRIVATE
  Backward::Object
  gz-common::gz-common
  gz-utils::cli
  gz-sim
)

install(
  TARGETS ${server_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

# Create GUI executable if enabled
if(ENABLE_GUI)
  set(gui_executable gz-sim-gui-client)
  add_executable(${gui_executable} gui_main.cc)
  target_link_libraries(${gui_executable}
    PRIVATE gz Backward::Object)

  target_compile_definitions(${gui_executable} PRIVATE WITH_GUI)

  install(
    TARGETS ${gui_executable}
    DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
  )
endif()

# Create sim main executable
set(sim_executable gz-sim-main)
add_executable(${sim_executable} sim_main.cc)
target_link_libraries(${sim_executable}
  PRIVATE Backward::Object gz)

install(
  TARGETS ${sim_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

if(ENABLE_GUI)
  target_compile_definitions(${sim_executable} PRIVATE WITH_GUI)
  target_compile_definitions(${sim_executable}
    PRIVATE
      "GZ_SIM_GUI_EXE=\"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${gui_executable}>\""
  )
endif()

# Build the unit tests.
set(test_sources gz_TEST.cc)

# Add systems that need a valid display here.
# \todo(anyone) Find a way to run these tests with a virtual display such Xvfb
# or Xdummy instead of skipping them
if(VALID_DISPLAY AND VALID_DRI_DISPLAY)
  list(APPEND test_sources ModelCommandAPI_TEST.cc)
else()
  message(STATUS
    "Skipping ModelCommandAPI tests because a valid display was not found")
endif()

# Build unit tests if Gazebo tools is installed
if(BUILD_TESTING AND GZ_TOOLS_PROGRAM)
  gz_build_tests(TYPE UNIT
    SOURCES
      ${test_sources}
    LIB_DEPS
      gz-utils::gz-utils
      ${PROJECT_LIBRARY_TARGET_NAME}
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
    TestModelSystem
    TestSensorSystem
    TestWorldSystem
  )

  target_compile_definitions(${CMD_TEST}
    PRIVATE
      "GZ_PATH=\"${GZ_TOOLS_PROGRAM}\""
  )

  if(${CMD_TEST} STREQUAL UNIT_gz_TEST AND ENABLE_GUI)
    target_compile_definitions(${CMD_TEST} PRIVATE WITH_GUI)
  endif()

  if(${CMD_TEST} STREQUAL UNIT_ModelCommandAPI_TEST OR ${CMD_TEST} STREQUAL UNIT_gz_TEST)
    set(_env_vars)
    list(APPEND _env_vars "GZ_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>")
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
      ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>")
  endif()
endforeach()

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
# Generate the ruby script for internal testing of sim command.
# Note that the major version of the library is included in the name.
# Ex: cmdsim0.rb
set(gz_sim_ruby_path
  "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/gz/cmdsim${PROJECT_VERSION_MAJOR}")
set(cmd_sim_script_generated_test "${gz_sim_ruby_path}.rb")
set(cmd_sim_script_configured_test
  "${CMAKE_CURRENT_BINARY_DIR}/test_cmdsim${PROJECT_VERSION_MAJOR}.rb.configured")

# Set the sim_exe_location variable to the full path of the executable within
# the build directory.
set(sim_exe_location "$<TARGET_FILE:${sim_executable}>")

configure_file(
  "cmdsim.rb.in"
  "${cmd_sim_script_configured_test}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_sim_script_generated_test}"
  INPUT "${cmd_sim_script_configured_test}"
)

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: sim0.yaml
configure_file(
  "sim.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/sim${PROJECT_VERSION_MAJOR}.yaml.configured"
  @ONLY
)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/sim${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/sim${PROJECT_VERSION_MAJOR}.yaml.configured"
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
# Used for the installed sim command version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdsim0.rb
set(cmd_sim_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdsim${PROJECT_VERSION_MAJOR}.rb")
set(cmd_sim_script_configured "${cmd_sim_script_generated}.configured")

# Set the sim_exe_location variable to the relative path to the executable
# within the install directory structure.
set(sim_exe_location "../../../${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${sim_executable}>")

configure_file(
  "cmdsim.rb.in"
  "${cmd_sim_script_configured}"
  @ONLY
)

file(GENERATE
  OUTPUT "${cmd_sim_script_generated}"
  INPUT "${cmd_sim_script_configured}"
)
# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_sim_script_generated} DESTINATION lib/ruby/gz)

# Used for the installed version.
set(gz_sim_ruby_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmdsim${PROJECT_VERSION_MAJOR}")

set(sim_configured "${CMAKE_CURRENT_BINARY_DIR}/sim${PROJECT_VERSION_MAJOR}.yaml")
configure_file(
  "sim.yaml.in"
  ${sim_configured}
  @ONLY)

install(FILES ${sim_configured} DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/gz/)

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
