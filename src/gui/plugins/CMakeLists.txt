#################################################
# gz_add_gui_library (<library_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#
# Add a gui library to Ignition Gazebo.
#
# <library_name> Required. Name of the gui library
#
# [SOURCES]: Specify the source files for the system.
#
# [QT_HEADERS]: Qt headers that need to be moc'ed
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
function(gz_add_gui_library library_name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs SOURCES QT_HEADERS PUBLIC_LINK_LIBS PRIVATE_LINK_LIBS)

  cmake_parse_arguments(gz_add_gui_library "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  QT5_WRAP_CPP(${library_name}_headers_MOC ${gz_add_gui_library_QT_HEADERS})
  QT5_ADD_RESOURCES(${library_name}_RCC ${library_name}.qrc)

  add_library(${library_name} SHARED
    ${gz_add_gui_library_SOURCES}
    ${${library_name}_headers_MOC}
    ${${library_name}_RCC}
  )
  target_link_libraries(${library_name}
    PUBLIC
      ${PROJECT_LIBRARY_TARGET_NAME}
      TINYXML2::TINYXML2
      ${gz_add_gui_library_PUBLIC_LINK_LIBS}
    PRIVATE
      ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
      ${gz_add_gui_library_PRIVATE_LINK_LIBS}
  )
endfunction()


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

  gz_add_gui_library(${plugin_name}
    SOURCES ${gz_add_gui_plugin_SOURCES}
    QT_HEADERS ${gz_add_gui_plugin_QT_HEADERS}
    PUBLIC_LINK_LIBS ${gz_add_gui_plugin_PUBLIC_LINK_LIBS}
    PRIVATE_LINK_LIBS ${gz_add_gui_plugin_PRIVATE_LINK_LIBS} ignition-plugin${IGN_PLUGIN_VER}::register
  )

  install (TARGETS ${plugin_name} DESTINATION ${IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR})
endfunction()

add_subdirectory(modules)

# Plugins
add_subdirectory(entity_tree)
add_subdirectory(grid_config)
add_subdirectory(scene3d)
add_subdirectory(transform_control)
add_subdirectory(video_recorder)
add_subdirectory(view_angle)
