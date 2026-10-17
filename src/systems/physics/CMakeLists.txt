# The write-back and command-clearing helpers are used by every physics
# provider. They are an internal static library that the physics and
# simplified-physics systems link, so that each source is compiled once. It is
# not installed.
add_library(gz-sim-physics-helpers STATIC
  CommandClearer.cc
  FrameDataWriter.cc
)
set_target_properties(gz-sim-physics-helpers PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_link_libraries(gz-sim-physics-helpers
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-math::eigen3
    gz-physics::core
)

gz_add_system(physics
  SOURCES
    ContactWriter.cc
    JointStateWriter.cc
    Physics.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-common::geospatial
    gz-common::graphics
    gz-common::profiler
    gz-physics::gz-physics
  PRIVATE_LINK_LIBS
    gz-sim-physics-helpers
)

set (gtest_sources
  CommandClearer_TEST.cc
  ContactWriter_TEST.cc
  EntityFeatureMap_TEST.cc
  FrameDataWriter_TEST.cc
  JointStateWriter_TEST.cc
  OwnerFilter_TEST.cc
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
  GZ_IP=127.0.0.1
)

if (TARGET UNIT_CommandClearer_TEST)
  target_link_libraries(UNIT_CommandClearer_TEST
    gz-sim-physics-helpers)
endif()

if (TARGET UNIT_ContactWriter_TEST)
  target_sources(UNIT_ContactWriter_TEST PRIVATE ContactWriter.cc)
endif()

if (TARGET UNIT_FrameDataWriter_TEST)
  target_link_libraries(UNIT_FrameDataWriter_TEST
    gz-sim-physics-helpers)
endif()

if (TARGET UNIT_JointStateWriter_TEST)
  target_sources(UNIT_JointStateWriter_TEST PRIVATE JointStateWriter.cc)
endif()
