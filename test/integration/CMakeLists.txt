set(TEST_TYPE "INTEGRATION")

set(tests
  ackermann_steering_system.cc
  air_pressure_system.cc
  altimeter_system.cc
  apply_joint_force_system.cc
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
  events.cc
  examples_build.cc
  follow_actor_system.cc
  force_torque_system.cc
  fuel_cached_server.cc
  halt_motion.cc
  imu_system.cc
  joint_controller_system.cc
  joint_position_controller_system.cc
  joint_state_publisher_system.cc
  joint_trajectory_controller_system.cc
  kinetic_energy_monitor_system.cc
  lift_drag_system.cc
  level_manager.cc
  level_manager_runtime_performers.cc
  link.cc
  logical_camera_system.cc
  logical_audio_sensor_plugin.cc
  magnetometer_system.cc
  model.cc
  multicopter.cc
  multiple_servers.cc
  navsat_system.cc
  nested_model_physics.cc
  network_handshake.cc
  odometry_publisher.cc
  particle_emitter.cc
  particle_emitter2.cc
  performer_detector.cc
  physics_system.cc
  play_pause.cc
  pose_publisher_system.cc
  recreate_entities.cc
  reset.cc
  save_world.cc
  scene_broadcaster_system.cc
  sdf_frame_semantics.cc
  sdf_include.cc
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

# Tests that require a valid display
set(tests_needing_display
  camera_sensor_background.cc
  camera_video_record_system.cc
  depth_camera.cc
  gpu_lidar.cc
  optical_tactile_plugin.cc
  rgbd_camera.cc
  sensors_system.cc
  thermal_system.cc
  thermal_sensor_system.cc
)

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

ign_build_tests(TYPE INTEGRATION
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
)


# For INTEGRATION_physics_system, we need to check what version of DART is
# available so that we can disable tests that are unsupported by the particular
# version of physics engine
ign_find_package(DART QUIET)
if (DART_FOUND)
  # Only adding include directories, no need to link against DART to check version
  target_include_directories(INTEGRATION_physics_system SYSTEM PRIVATE ${DART_INCLUDE_DIRS})
  target_compile_definitions(INTEGRATION_physics_system PRIVATE HAVE_DART)
endif()

target_link_libraries(INTEGRATION_tracked_vehicle_system
  ignition-physics${IGN_PHYSICS_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::loader
)
# The default timeout (240s) doesn't seem to be enough for this test.
set_tests_properties(INTEGRATION_tracked_vehicle_system PROPERTIES TIMEOUT 300)

if(TARGET INTEGRATION_examples_build)
  set_tests_properties(INTEGRATION_examples_build PROPERTIES TIMEOUT 320)
endif()
