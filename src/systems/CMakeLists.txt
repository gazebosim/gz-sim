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

  # The library created by `ign_add_component` includes the ign-gazebo version
  # (i.e. libignition-gazebo1-name-system.so), but for portability of SDF
  # files, we also install an unversioned symlink into the same versioned folder.
  set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${system_target}${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${system_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
  if (WIN32)
    # create_symlink requires cmake 3.13 on windows
    cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
  endif()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
endfunction()

add_subdirectory(air_pressure)
add_subdirectory(altimeter)
add_subdirectory(apply_joint_force)
add_subdirectory(battery_plugin)
add_subdirectory(breadcrumbs)
add_subdirectory(contact)
add_subdirectory(detachable_joint)
add_subdirectory(diff_drive)
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
add_subdirectory(wind_effects)
