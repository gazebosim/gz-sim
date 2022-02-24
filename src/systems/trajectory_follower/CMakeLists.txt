gz_add_system(trajectory-follower
  SOURCES
  TrajectoryFollower.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
