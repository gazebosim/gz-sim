gz_add_system(detachable-joint
  SOURCES
  DetachableJoint.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
