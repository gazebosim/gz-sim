gz_add_system(mecanum-drive
  SOURCES
    MecanumDrive.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
