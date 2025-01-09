set(TEST_TYPE "INTEGRATION")

set(tests
  ackermann_steering_system.cc
  actor.cc
  acoustic_comms.cc
  air_pressure_system.cc
  air_speed_system.cc
  altimeter_system.cc
  added_mass.cc
  apply_joint_force_system.cc
  apply_link_wrench_system.cc
  battery_plugin.cc
  breadcrumbs.cc
  buoyancy.cc
  buoyancy_engine.cc
  collada_world_exporter.cc
  components.cc
  contact_system.cc
  detachable_joint.cc
  diff_drive_system.cc
  each_new_removed.cc
  entity_erase.cc
  entity_system.cc
  environment_preload_system.cc
  environmental_sensor_system.cc
  events.cc
  examples_build.cc
  follow_actor_system.cc
  force_torque_system.cc
  fuel_cached_server.cc
  halt_motion.cc
  hydrodynamics.cc
  hydrodynamics_flags.cc
  imu_system.cc
  include_sim_hh.cc
  joint.cc
  joint_controller_system.cc
  joint_position_controller_system.cc
  joint_state_publisher_system.cc
  joint_trajectory_controller_system.cc
  kinetic_energy_monitor_system.cc
  lift_drag_system.cc
  level_manager.cc
  level_manager_runtime_performers.cc
  light.cc
  lighter_than_air_dynamics.cc
  link.cc
  logical_camera_system.cc
  logical_audio_sensor_plugin.cc
  magnetometer_system.cc
  material.cc
  mecanum_drive_system.cc
  mesh_inertia_calculation.cc
  model.cc
  model_photo_shoot_default_joints.cc
  model_photo_shoot_random_joints.cc
  multicopter.cc
  multiple_servers.cc
  navsat_system.cc
  nested_model_physics.cc
  network_handshake.cc
  odometry_publisher.cc
  particle_emitter.cc
  perfect_comms.cc
  performer_detector.cc
  physics_system.cc
  play_pause.cc
  pose_publisher_system.cc
  projector.cc
  rf_comms.cc
  recreate_entities.cc
  reset.cc
  reset_detachable_joint.cc
  save_world.cc
  spacecraft.cc
  scene_broadcaster_system.cc
  sdf_frame_semantics.cc
  sdf_include.cc
  sensor.cc
  spherical_coordinates.cc
  thruster.cc
  touch_plugin.cc
  tracked_vehicle_system.cc
  triggered_publisher.cc
  user_commands.cc
  velocity_control_system.cc
  log_system.cc
  wheel_slip.cc
  wind_effects.cc
  world.cc
  world_control_state.cc
)

# elevator system causes compile erros on windows
if (NOT WIN32)
  list(APPEND tests elevator_system.cc)
endif()

# Tests that require a valid display
set(tests_needing_display
  actor_trajectory.cc
  camera_lens_flare.cc
  camera_sensor_background.cc
  camera_sensor_background_from_scene.cc
  camera_video_record_system.cc
  depth_camera.cc
  distortion_camera.cc
  dvl_system.cc
  dvl_system_bottom_tracking.cc
  dvl_system_water_mass_tracking.cc
  camera_sensor_global_illumination.cc
  gpu_lidar.cc
  markers.cc
  mesh_uri.cc
  optical_tactile_plugin.cc
  reset_sensors.cc
  rgbd_camera.cc
  sensors_system.cc
  sensors_system_battery.cc
  sensors_system_update_rate.cc
  shader_param_system.cc
  thermal_sensor_system.cc
  thermal_system.cc
  triggered_camera.cc
  wide_angle_camera.cc
)

# Disable tests that need CLI if gz-tools is not found
if (MSVC OR NOT GZ_TOOLS_PROGRAM)
  list(REMOVE_ITEM tests log_system.cc)
endif()

# Add systems that need a valid display here.
# \todo(anyone) Find a way to run these tests with a virtual display such Xvfb
# or Xdummy instead of skipping them
if(VALID_DISPLAY AND VALID_DRI_DISPLAY)
  list(APPEND tests ${tests_needing_display})
else()
  message(STATUS
    "Skipping these INTEGRATION tests because a valid display was not found:")
  foreach(test ${tests_needing_display})
    message(STATUS " ${test}")
  endforeach(test)
endif()

# Add tests that need pybind11
if (${pybind11_FOUND})
  list(APPEND tests python_system_loader.cc)
endif()

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${tests} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

gz_build_tests(TYPE INTEGRATION
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)


# For INTEGRATION_physics_system, we need to check what version of DART is
# available so that we can disable tests that are unsupported by the particular
# version of physics engine
cmake_policy(PUSH)
if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
gz_find_package(DART CONFIG)
cmake_policy(POP)
if (DART_FOUND AND TARGET INTEGRATION_physics_system)
  # Only adding include directories, no need to link against DART to check version
  target_include_directories(INTEGRATION_physics_system SYSTEM PRIVATE ${DART_INCLUDE_DIRS})
  target_compile_definitions(INTEGRATION_physics_system PRIVATE HAVE_DART)

  target_link_libraries(INTEGRATION_physics_system
    gz-math${GZ_MATH_VER}::eigen3
  )
endif()

if (TARGET INTEGRATION_tracked_vehicle_system)
  target_link_libraries(INTEGRATION_tracked_vehicle_system
    gz-physics${GZ_PHYSICS_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::loader
  )
endif()

if (TARGET INTEGRATION_model_photo_shoot_default_joints)
  target_link_libraries(INTEGRATION_model_photo_shoot_default_joints
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  )
endif()

if (TARGET INTEGRATION_model_photo_shoot_random_joints)
  target_link_libraries(INTEGRATION_model_photo_shoot_random_joints
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  )
endif()

if (TARGET INTEGRATION_log_system)
  target_link_libraries(INTEGRATION_log_system
    gz-transport${GZ_TRANSPORT_VER}::log
  )
endif()

if (TARGET INTEGRATION_collada_world_exporter)
  target_link_libraries(INTEGRATION_collada_world_exporter
    gz-common${GZ_COMMON_VER}::graphics
  )
endif()

if(TARGET INTEGRATION_reset_sensors)
  target_link_libraries(INTEGRATION_reset_sensors
    gz-common${GZ_COMMON_VER}::graphics
  )
endif()

# The default timeout (240s) doesn't seem to be enough for this test.
if(TARGET INTEGRATION_tracked_vehicle_system )
  set_tests_properties(INTEGRATION_tracked_vehicle_system PROPERTIES TIMEOUT 300)
endif()

if(TARGET INTEGRATION_examples_build)
  set_tests_properties(INTEGRATION_examples_build PROPERTIES TIMEOUT 320)
endif()

if(VALID_DISPLAY AND VALID_DRI_DISPLAY AND TARGET INTEGRATION_sensors_system)
  target_link_libraries(INTEGRATION_sensors_system
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  )
  target_link_libraries(INTEGRATION_actor_trajectory
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  )
endif()

if (TARGET INTEGRATION_python_system_loader)
  set_tests_properties(INTEGRATION_python_system_loader PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/python/:$ENV{PYTHONPATH}")
endif()
