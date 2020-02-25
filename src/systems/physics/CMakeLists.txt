gz_add_system(physics
  SOURCES
    Physics.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-common${IGN_COMMON_VER}::profiler
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
  PRIVATE_COMPILE_DEFS
    # Directory where all ign-physics engine plugins are installed
    "IGN_PHYSICS_ENGINES=\"$<TARGET_SONAME_FILE_DIR:ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}>\""
    "IGN_PHYSICS_VER=\"${IGN_PHYSICS_VER}\""
)

