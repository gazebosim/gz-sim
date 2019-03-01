set (diff_drive_system_sources
  DiffDrive.cc
)

ign_add_component(diff-drive-system
  SOURCES ${diff_drive_system_sources}
  GET_TARGET_NAME diff_drive_system_target
  CXX_STANDARD 17)

target_link_libraries(${diff_drive_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${diff_drive_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
