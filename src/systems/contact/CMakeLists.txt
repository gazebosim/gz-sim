set (contact_system_sources
  Contact.cc
)

ign_add_component(contact-system
  SOURCES ${contact_system_sources}
  GET_TARGET_NAME contact_system_target
  CXX_STANDARD 17)

target_link_libraries(${contact_system_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${contact_system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

