gz_add_system(trajectory-follower
  SOURCES
  TrajectoryFollower.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
