gz_add_system(mecanum-drive
  SOURCES
    MecanumDrive.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
