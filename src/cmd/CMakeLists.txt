#===============================================================================
# Generate the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdgazebo0.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(library_location "../../../${CMAKE_INSTALL_LIBDIR}/$<TARGET_FILE_NAME:${gz_lib_target}>")

configure_file(
  "cmd${GZ_DESIGNATION}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/gz)

set(gz_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmd${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: gazebo0.yaml
configure_file(
  "${GZ_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install( FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml
  DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gz/)

#===============================================================================
# Used for the installed model command version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
set(cmd_model_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_model_script_configured "${cmd_model_script_generated}.configured")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured}"
  @ONLY)
file(GENERATE
  OUTPUT "${cmd_model_script_generated}"
  INPUT  "${cmd_model_script_configured}")

install(FILES ${cmd_model_script_generated} DESTINATION lib/ruby/gz)

# Used for the installed version.
set(gz_model_ruby_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmdmodel${PROJECT_VERSION_MAJOR}")

set(model_configured "${CMAKE_CURRENT_BINARY_DIR}/model${PROJECT_VERSION_MAJOR}.yaml")
configure_file(
  "model.yaml.in"
  ${model_configured})

install(FILES ${model_configured} DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gz/)


#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdgazebo0.rb
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
# Ex: gazebo0.yaml
configure_file(
  "${GZ_DESIGNATION}.yaml.in"
  "${CMAKE_BINARY_DIR}/test/conf/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml" @ONLY)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
set(cmd_model_ruby_test_dir "${CMAKE_BINARY_DIR}/test/lib/ruby/gz")
set(cmd_model_script_generated_test "${cmd_model_ruby_test_dir}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_model_script_configured_test "${cmd_model_script_generated_test}.configured")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_model_script_generated_test}"
  INPUT  "${cmd_model_script_configured_test}")

# Used for internal testing.
set(gz_model_ruby_path "${cmd_model_script_generated_test}")

configure_file(
  "model.yaml.in"
  "${CMAKE_BINARY_DIR}/test/conf/model${PROJECT_VERSION_MAJOR}.yaml" @ONLY)
