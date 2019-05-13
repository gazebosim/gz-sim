#################################################
# gz_add_gui_plugin(<plugin_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#
# Add a gui plugin to Ignition Gazebo.
#
# <plugin_name> Required. Name of the gui plugin.
#
# [SOURCES]: Specify the source files for the system.
#
# [QT_HEADERS]: Qt headers that need to be moc'ed
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
function(gz_add_gui_plugin plugin_name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs SOURCES QT_HEADERS PUBLIC_LINK_LIBS PRIVATE_LINK_LIBS)

  cmake_parse_arguments(gz_add_gui_plugin "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  QT5_WRAP_CPP(${plugin_name}_headers_MOC ${gz_add_gui_plugin_QT_HEADERS})
  QT5_ADD_RESOURCES(${plugin_name}_RCC ${plugin_name}.qrc)

  add_library(${plugin_name} SHARED
    ${gz_add_gui_plugin_SOURCES}
    ${${plugin_name}_headers_MOC}
    ${${plugin_name}_RCC}
  )
  target_link_libraries(${plugin_name}
    PUBLIC
      ${PROJECT_LIBRARY_TARGET_NAME}
      TINYXML2::TINYXML2
      ${gz_add_gui_plugin_PUBLIC_LINK_LIBS}
    PRIVATE
      ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
      ignition-plugin${IGN_PLUGIN_VER}::register
      ${gz_add_gui_plugin_PRIVATE_LINK_LIBS}
  )
  install (TARGETS ${plugin_name} DESTINATION ${IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR})

endfunction()

add_subdirectory(entity_tree)
add_subdirectory(scene3d)
