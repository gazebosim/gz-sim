gz_add_system(drive-to-pose-controller
  SOURCES
    DriveToPoseController.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
