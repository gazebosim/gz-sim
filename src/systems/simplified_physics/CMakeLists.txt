# Collision queries for the planar modes, built on the collision code of
# Box2D. Internal and not installed. Nothing links it yet: the system will
# once it uses the collision queries. The file is compiled without floating
# point contraction, as Box2D is, so that results are bit-identical across
# compilers that would otherwise fuse multiply-adds.
add_library(gz-sim-planar-collision STATIC PlanarCollision.cc)
set_target_properties(gz-sim-planar-collision PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gz-sim-planar-collision PRIVATE -ffp-contract=off)
endif()
target_link_libraries(gz-sim-planar-collision
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common::gz-common
    gz-common::graphics
  PRIVATE
    gz-sim-box2d
)

gz_build_tests(TYPE UNIT
  SOURCES
    PlanarCollision_TEST.cc
  LIB_DEPS
    gz-sim-planar-collision
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

gz_add_system(simplified-physics
  SOURCES
    PhysicsMode.cc
    PlanarMotion.cc
    SimplifiedPhysics.cc
    SimplifiedPhysicsModel.cc
    SimplifiedPhysicsPlugins.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-common::profiler
    gz-physics::core
  PRIVATE_LINK_LIBS
    gz-sim-physics-helpers
)

gz_build_tests(TYPE UNIT
  SOURCES
    PhysicsMode_TEST.cc
    PlanarMotion_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_PhysicsMode_TEST)
  target_sources(UNIT_PhysicsMode_TEST PRIVATE PhysicsMode.cc)
endif()

if (TARGET UNIT_PlanarMotion_TEST)
  target_sources(UNIT_PlanarMotion_TEST PRIVATE PlanarMotion.cc)
endif()
