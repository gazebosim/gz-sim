set(TEST_TYPE "INTEGRATION")

set(tests
  air_pressure_system.cc
  altimeter_system.cc
  apply_joint_force_system.cc
  battery_plugin.cc
  breadcrumbs.cc
  components.cc
  contact_system.cc
  diff_drive_system.cc
  each_new_removed.cc
  entity_erase.cc
  events.cc
  examples_build.cc
  imu_system.cc
  joint_controller_system.cc
  lift_drag_system.cc
  level_manager.cc
  level_manager_runtime_performers.cc
  link.cc
  logical_camera_system.cc
  magnetometer_system.cc
  model.cc
  multicopter.cc
  network_handshake.cc
  physics_system.cc
  play_pause.cc
  pose_publisher_system.cc
  scene_broadcaster_system.cc
  sdf_frame_semantics.cc
  sdf_include.cc
  thermal_system.cc
  touch_plugin.cc
  user_commands.cc
  log_system.cc
  wind_effects.cc
)

# Tests that require a valid display
set(tests_needing_display
  depth_camera.cc
  gpu_lidar.cc
  rgbd_camera.cc
  sensors_system.cc
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

link_directories(${PROJECT_BINARY_DIR}/test)
include_directories(${PROJECT_SOURCE_DIR}/test)

ign_build_tests(TYPE INTEGRATION
  SOURCES
    ${tests}
  LIB_DEPS
    ${EXTRA_TEST_LIB_DEPS}
)
