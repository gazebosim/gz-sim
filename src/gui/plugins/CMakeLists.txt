#################################################
# gz_add_gui_library (<library_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#
# Add a gui library to Gazebo.
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

  if(MSVC)
    # Warning #4251 is the "dll-interface" warning that tells you when types
    # used by a class are not being exported. These generated source files have
    # private members that don't get exported, so they trigger this warning.
    # However, the warning is not important since those members do not need to
    # be interfaced with.
    set_source_files_properties(${${library_name}_headers_MOC} COMPILE_FLAGS "/wd4251")
    set_source_files_properties(${gz_add_gui_library_SOURCES} COMPILE_FLAGS "/wd4251")
  endif()

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
      gz-sim${PROJECT_VERSION_MAJOR}-gui
      ${gz_add_gui_library_PRIVATE_LINK_LIBS}
  )
endfunction()


#################################################
# gz_add_gui_plugin(<plugin_name>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>])
#
# Add a gui plugin to Gazebo.
#
# <plugin_name> Required. Name of the gui plugin.
#
# [SOURCES]: Specify the source files for the system.
#
# [QT_HEADERS]: Qt headers that need to be moc'ed
#
# [TEST_SOURCES]: Source files for unit tests.
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
function(gz_add_gui_plugin plugin_name)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs
    SOURCES
    QT_HEADERS
    TEST_SOURCES
    PUBLIC_LINK_LIBS
    PRIVATE_LINK_LIBS
  )

  cmake_parse_arguments(gz_add_gui_plugin "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(MSVC)
    # Warning #4251 is the "dll-interface" warning that tells you when types
    # used by a class are not being exported. These generated source files have
    # private members that don't get exported, so they trigger this warning.
    # However, the warning is not important since those members do not need to
    # be interfaced with.
    set_source_files_properties(${gz_add_gui_plugin_SOURCES} COMPILE_FLAGS "/wd4251")
  endif()

  gz_add_gui_library(${plugin_name}
    SOURCES ${gz_add_gui_plugin_SOURCES}
    QT_HEADERS ${gz_add_gui_plugin_QT_HEADERS}
    PUBLIC_LINK_LIBS ${gz_add_gui_plugin_PUBLIC_LINK_LIBS}
    PRIVATE_LINK_LIBS ${gz_add_gui_plugin_PRIVATE_LINK_LIBS} gz-plugin${GZ_PLUGIN_VER}::register
  )

  if(gz_add_gui_plugin_TEST_SOURCES)
    # Plugin symbols failing to resolve on Windows:
    # error LNK2001: unresolved external symbol
    if(NOT WIN32)
      gz_build_tests(TYPE UNIT
        SOURCES
          ${gz_add_gui_plugin_TEST_SOURCES}
        LIB_DEPS
          gz-sim${PROJECT_VERSION_MAJOR}-gui
          ${plugin_name}
        INCLUDE_DIRS
          # Used to make internal source file headers visible to the unit tests
          ${CMAKE_CURRENT_SOURCE_DIR}
          # Used to make test-directory headers visible to the unit tests
          ${PROJECT_SOURCE_DIR}
          # Used to make test_config.h visible to the unit tests
          ${PROJECT_BINARY_DIR}
        ENVIRONMENT
          GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
    endif()
  endif()

  install (TARGETS ${plugin_name} DESTINATION ${GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR})
endfunction()

add_subdirectory(modules)

# Plugins
add_subdirectory(align_tool)
add_subdirectory(apply_force_torque)
add_subdirectory(banana_for_scale)
add_subdirectory(component_inspector)
add_subdirectory(component_inspector_editor)
add_subdirectory(copy_paste)
add_subdirectory(entity_context_menu)
add_subdirectory(entity_tree)
add_subdirectory(environment_loader)
add_subdirectory(environment_visualization)
add_subdirectory(global_illumination_civct)
add_subdirectory(global_illumination_vct)
add_subdirectory(joint_position_controller)
add_subdirectory(lights)
add_subdirectory(mouse_drag)
add_subdirectory(playback_scrubber)
add_subdirectory(plot_3d)
add_subdirectory(plotting)
add_subdirectory(resource_spawner)
add_subdirectory(select_entities)
add_subdirectory(scene_manager)
add_subdirectory(shapes)
add_subdirectory(spawn)
add_subdirectory(transform_control)
add_subdirectory(video_recorder)
add_subdirectory(view_angle)
add_subdirectory(visualization_capabilities)
add_subdirectory(visualize_contacts)
add_subdirectory(visualize_lidar)
