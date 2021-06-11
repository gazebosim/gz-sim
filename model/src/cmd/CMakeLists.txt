#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
set(cmd_model_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_model_script_configured "${cmd_model_script_generated}.configured")

# Set the model_library_location variable to the relative path to the library file
# within the install directory structure.
set(model_library_location "../../../${CMAKE_INSTALL_LIBDIR}/$<TARGET_FILE_NAME:${model_lib_target}>")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_model_script_generated}"
  INPUT  "${cmd_model_script_configured}")

install(FILES ${cmd_model_script_generated} DESTINATION lib/ruby/ignition)

# Used for the installed version.
set(ign_model_ruby_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdmodel${PROJECT_VERSION_MAJOR}")

set(ignmodel_configured "${CMAKE_CURRENT_BINARY_DIR}/ignmodel${PROJECT_VERSION_MAJOR}.yaml")
configure_file(
  "ignmodel.yaml.in"
  ${ignmodel_configured})

install(FILES ${ignmodel_configured} DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)

#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
set(cmd_model_ruby_test_dir "${CMAKE_BINARY_DIR}/model/test/lib/ruby/ignition")
set(cmd_model_script_generated_test "${cmd_model_ruby_test_dir}/cmdmodel${PROJECT_VERSION_MAJOR}.rb")
set(cmd_model_script_configured_test "${cmd_model_script_generated_test}.configured")

# Set the model_library_location variable to the full path of the library file
# within the build directory
set(model_library_location "$<TARGET_FILE:${model_lib_target}>")

configure_file(
  "cmdmodel.rb.in"
  "${cmd_model_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_model_script_generated_test}"
  INPUT  "${cmd_model_script_configured_test}")

# Used for internal testing.
set(ign_model_ruby_path "${cmd_model_script_generated_test}")

configure_file(
  "ignmodel.yaml.in"
  "${cmd_model_ruby_test_dir}/ignmodel${PROJECT_VERSION_MAJOR}.yaml")
