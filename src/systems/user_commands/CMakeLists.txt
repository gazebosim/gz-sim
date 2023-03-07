gz_add_system(user-commands
  SOURCES
    UserCommands.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
