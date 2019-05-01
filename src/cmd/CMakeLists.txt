# Generate the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdgazebo0.rb
if (APPLE)
  set(IGN_LIBRARY_NAME lib${PROJECT_LIBRARY_TARGET_NAME}-ign.dylib)
else()
  set(IGN_LIBRARY_NAME lib${PROJECT_LIBRARY_TARGET_NAME}-ign.so)
endif()

configure_file(
  "cmdgazebo.rb.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmdgazebo${PROJECT_VERSION_MAJOR}.rb" @ONLY)

# Install the ruby command line library in an unversioned location.
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmdgazebo${PROJECT_VERSION_MAJOR}.rb
  DESTINATION
    lib/ruby/ignition)

set(ign_library_path
  "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdgazebo${PROJECT_VERSION_MAJOR}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: gazebo0.yaml
configure_file(
  "gazebo.yaml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/gazebo${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/gazebo${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdgazebo0.rb
set(cmd_script_generated_test
  "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdgazebo${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured_test
  "${cmd_script_generated_test}.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(IGN_LIBRARY_NAME "$<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>-ign")

configure_file(
  "cmdgazebo.rb.in"
  "${cmd_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}")

# Used only for internal testing.
set(ign_library_path
  "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmdgazebo${PROJECT_VERSION_MAJOR}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: gazebo0.yaml
configure_file(
  "gazebo.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/gazebo${PROJECT_VERSION_MAJOR}.yaml" @ONLY)
