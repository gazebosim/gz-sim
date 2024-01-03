gz_add_system(diff-drive
  SOURCES
    DiffDrive.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
