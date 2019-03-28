gz_add_system(physics
  SOURCES
    Physics.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-common${IGN_COMMON_VER}::profiler
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
  PRIVATE_COMPILE_DEFS
    "dartsim_plugin_LIB=\"$<TARGET_SONAME_FILE:ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}-dartsim-plugin>\""
)

