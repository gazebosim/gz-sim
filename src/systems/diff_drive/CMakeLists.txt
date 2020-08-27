gz_add_system(diff-drive
  SOURCES
    DiffDrive.cc
    SpeedLimiter.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
