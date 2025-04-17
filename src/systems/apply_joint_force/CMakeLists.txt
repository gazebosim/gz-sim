gz_add_system(apply-joint-force
  SOURCES
  ApplyJointForce.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport::gz-transport
)
