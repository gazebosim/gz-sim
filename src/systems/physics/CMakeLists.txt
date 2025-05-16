gz_add_system(physics
  SOURCES
    Physics.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-common::geospatial
    gz-common::graphics
    gz-common::profiler
    gz-physics::gz-physics
)

set (gtest_sources
  EntityFeatureMap_TEST.cc
)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${gtest_sources} COMPILE_FLAGS "/wd4251 /wd4146")
endif()

gz_build_tests(TYPE UNIT
  SOURCES
  ${gtest_sources}
  LIB_DEPS
  gz-physics::core
  gz-plugin::loader
  ENVIRONMENT
  GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
