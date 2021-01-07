#################################################
# gz_add_system(<system>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>]
#              [PRIVATE_COMPILE_DEFS <definitions...>])
#
# Add a system component to Ignition Gazebo.
#
# <system> Required. Name of the system.
#
# SOURCES: Required. Specify the source files for the system.
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
# [PRIVATE_COMPILE_DEFS]: Specify a list of private compile definitions to add to target.
#
function(gz_add_system system_name)

  set(options)
  set(oneValueArgs)
  set(multiValueArgs SOURCES PUBLIC_LINK_LIBS PRIVATE_LINK_LIBS PRIVATE_COMPILE_DEFS)

  cmake_parse_arguments(gz_add_system "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(gz_add_system_SOURCES)
    set(sources ${gz_add_system_SOURCES})
  else()
    message(FATAL_ERROR "You must specify SOURCES for gz_add_system(~)!")
  endif()

  set(system_name ${system_name}-system)


  if (MSVC)
    # Warning #4251 is the "dll-interface" warning that tells you when types used
    # by a class are not being exported. These generated source files have private
    # members that don't get exported, so they trigger this warning. However, the
    # warning is not important since those members do not need to be interfaced
    # with.
    set_source_files_properties(${sources} COMPILE_FLAGS "/wd4251 /wd4146")
  endif()


  ign_add_component(${system_name}
    SOURCES ${sources}
    GET_TARGET_NAME system_target
    CXX_STANDARD 17)

  target_link_libraries(${system_target}
    PUBLIC
      ${gz_add_system_PUBLIC_LINK_LIBS}
      ${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${gz_add_system_PRIVATE_LINK_LIBS}
      ignition-plugin${IGN_PLUGIN_VER}::register
  )

  if(gz_add_system_PRIVATE_COMPILE_DEFS)
    target_compile_definitions(${system_target}
      PRIVATE
        ${gz_add_system_PRIVATE_COMPILE_DEFS}
    )
  endif()

  # Note that plugins are currently being installed in 2 places. /lib and the plugin dir
  install(TARGETS ${system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

  if (NOT WIN32)
  # The library created by `ign_add_component` includes the ign-gazebo version
  # (i.e. libignition-gazebo1-name-system.so), but for portability of SDF
  # files, we also install an unversioned symlink into the same versioned folder.
  set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${system_target}${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${system_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
    INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
  endif()
endfunction()

add_subdirectory(air_pressure)
add_subdirectory(altimeter)
add_subdirectory(apply_joint_force)
add_subdirectory(battery_plugin)
add_subdirectory(breadcrumbs)
add_subdirectory(buoyancy)
add_subdirectory(contact)
add_subdirectory(camera_video_recorder)
add_subdirectory(detachable_joint)
add_subdirectory(diff_drive)
add_subdirectory(follow_actor)
add_subdirectory(imu)
add_subdirectory(joint_controller)
add_subdirectory(joint_position_controller)
add_subdirectory(joint_state_publisher)
add_subdirectory(lift_drag)
add_subdirectory(log)
add_subdirectory(log_video_recorder)
add_subdirectory(logical_camera)
add_subdirectory(magnetometer)
add_subdirectory(multicopter_motor_model)
add_subdirectory(multicopter_control)
add_subdirectory(performer_detector)
add_subdirectory(physics)
add_subdirectory(pose_publisher)
add_subdirectory(scene_broadcaster)
add_subdirectory(sensors)
add_subdirectory(thermal)
add_subdirectory(touch_plugin)
add_subdirectory(triggered_publisher)
add_subdirectory(user_commands)
add_subdirectory(velocity_control)
add_subdirectory(wheel_slip)
add_subdirectory(wind_effects)
