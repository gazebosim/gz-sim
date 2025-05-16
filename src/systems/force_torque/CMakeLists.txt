gz_add_system(forcetorque
  SOURCES
    ForceTorque.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::force_torque
)
