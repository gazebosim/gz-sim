gz_add_system(detachable-joint
  SOURCES
  DetachableJoint.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
