
add_subdirectory(include/ignition/math)

ign_add_component(eigen3 INTERFACE
  GET_TARGET_NAME component)

target_link_libraries(${component} INTERFACE Eigen3::Eigen)
