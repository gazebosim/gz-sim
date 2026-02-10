gz_add_system(dynamic-detachable-joint
  SOURCES
  DynamicDetachableJoint.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
