#################################################
# gz_add_system(<system>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>]
#              [PRIVATE_COMPILE_DEFS <definitions...>])
#
# Add a system component to Gazebo.
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
  set(multiValueArgs SOURCES PUBLIC_LINK_LIBS PRIVATE_INCLUDE_DIRS PRIVATE_LINK_LIBS PRIVATE_COMPILE_DEFS)

  cmake_parse_arguments(gz_add_system "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(gz_add_system_SOURCES)
    set(sources ${gz_add_system_SOURCES})
  else()
    message(FATAL_ERROR "You must specify SOURCES for gz_add_system(~)!")
  endif()

  set(system_name ${system_name}-system)


  if(MSVC)
    # Warning #4251 is the "dll-interface" warning that tells you when types
    # used by a class are not being exported. These generated source files have
    # private members that don't get exported, so they trigger this warning.
    # However, the warning is not important since those members do not need to
    # be interfaced with.
    set_source_files_properties(${sources} COMPILE_FLAGS "/wd4251")
  endif()


  gz_add_component(${system_name}
    SOURCES ${sources}
    GET_TARGET_NAME system_target)

  target_link_libraries(${system_target}
    PUBLIC
      ${gz_add_system_PUBLIC_LINK_LIBS}
      ${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${gz_add_system_PRIVATE_LINK_LIBS}
      gz-plugin${GZ_PLUGIN_VER}::register
  )

  if(gz_add_system_PRIVATE_INCLUDE_DIRS)
    target_include_directories(${system_target}
      PRIVATE
        ${gz_add_system_PRIVATE_INCLUDE_DIRS}
    )
  endif()

  if(gz_add_system_PRIVATE_COMPILE_DEFS)
    target_compile_definitions(${system_target}
      PRIVATE
        ${gz_add_system_PRIVATE_COMPILE_DEFS}
    )
  endif()

  # Note that plugins are currently being installed in 2 places. /lib and the plugin dir
  install(TARGETS ${system_target} DESTINATION ${GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR})

  # The library created by `gz_add_component` includes the gz-sim version
  # (i.e. libgz-sim1-name-system.so), but for portability of SDF
  # files, we also install an unversioned symlink into the same versioned folder.
  #
  # NOTE(CH3): On migration of the CMake project name, ignition-gazebo -> gz-sim
  # so the generated library target WILL CHANGE
  set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${system_target}${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${system_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
  if(WIN32)
    # symlinks on Windows require admin priviledges, fallback to copy
    INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
      ${GZ_SIM_PLUGIN_INSTALL_DIR}\/${versioned}
      ${GZ_SIM_PLUGIN_INSTALL_DIR}\/${unversioned})")
  else()
    file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned} WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
    INSTALL(FILES ${PROJECT_BINARY_DIR}/lib/${unversioned} DESTINATION ${GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR})
  endif()
endfunction()

add_subdirectory(ackermann_steering)
add_subdirectory(acoustic_comms)
add_subdirectory(advanced_lift_drag)
add_subdirectory(air_pressure)
add_subdirectory(air_speed)
add_subdirectory(altimeter)
add_subdirectory(apply_joint_force)
add_subdirectory(apply_link_wrench)
add_subdirectory(battery_plugin)
add_subdirectory(breadcrumbs)
add_subdirectory(buoyancy)
add_subdirectory(buoyancy_engine)
add_subdirectory(collada_world_exporter)
add_subdirectory(comms_endpoint)
add_subdirectory(contact)
add_subdirectory(camera_video_recorder)
add_subdirectory(detachable_joint)
add_subdirectory(diff_drive)
add_subdirectory(dvl)
# elevator system causes compile erros on windows
if (NOT WIN32)
add_subdirectory(elevator)
endif()
add_subdirectory(environment_preload)
add_subdirectory(environmental_sensor_system)
add_subdirectory(follow_actor)
add_subdirectory(force_torque)
add_subdirectory(hydrodynamics)
add_subdirectory(imu)
add_subdirectory(joint_controller)
add_subdirectory(joint_position_controller)
add_subdirectory(joint_state_publisher)
add_subdirectory(joint_traj_control)
add_subdirectory(kinetic_energy_monitor)
add_subdirectory(label)
add_subdirectory(lens_flare)
add_subdirectory(lift_drag)
add_subdirectory(lighter_than_air_dynamics)
add_subdirectory(log)
add_subdirectory(log_video_recorder)
add_subdirectory(logical_audio_sensor_plugin)
add_subdirectory(logical_camera)
add_subdirectory(magnetometer)
add_subdirectory(model_photo_shoot)
add_subdirectory(mecanum_drive)
add_subdirectory(multicopter_motor_model)
add_subdirectory(multicopter_control)
add_subdirectory(navsat)
add_subdirectory(odometry_publisher)
add_subdirectory(optical_tactile_plugin)
add_subdirectory(particle_emitter)
add_subdirectory(performer_detector)
add_subdirectory(perfect_comms)
add_subdirectory(physics)
add_subdirectory(pose_publisher)
add_subdirectory(python_system_loader)
add_subdirectory(rf_comms)
add_subdirectory(scene_broadcaster)
add_subdirectory(sensors)
add_subdirectory(shader_param)
add_subdirectory(spacecraft_thruster_model)
add_subdirectory(thermal)
add_subdirectory(thruster)
add_subdirectory(touch_plugin)
add_subdirectory(track_controller)
add_subdirectory(tracked_vehicle)
add_subdirectory(trajectory_follower)
add_subdirectory(triggered_publisher)
add_subdirectory(user_commands)
add_subdirectory(velocity_control)
add_subdirectory(wheel_slip)
add_subdirectory(wind_effects)
