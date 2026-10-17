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
