gz_add_system(simplified-physics
  SOURCES
    ModelParams.cc
    OdometryNoise.cc
    PhysicsMode.cc
    PlanarMotion.cc
    SimplifiedPhysics.cc
    SimplifiedPhysicsModel.cc
    SimplifiedPhysicsPlugins.cc
    WheelInference.cc
    WheelKinematics.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-common::profiler
    gz-physics::core
  PRIVATE_LINK_LIBS
    gz-sim-physics-helpers
)

gz_build_tests(TYPE UNIT
  SOURCES
    ModelParams_TEST.cc
    OdometryNoise_TEST.cc
    PhysicsMode_TEST.cc
    PlanarMotion_TEST.cc
    WheelKinematics_TEST.cc
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

if (TARGET UNIT_ModelParams_TEST)
  target_sources(UNIT_ModelParams_TEST PRIVATE ModelParams.cc)
endif()

if (TARGET UNIT_OdometryNoise_TEST)
  target_sources(UNIT_OdometryNoise_TEST PRIVATE OdometryNoise.cc)
endif()

if (TARGET UNIT_WheelKinematics_TEST)
  target_sources(UNIT_WheelKinematics_TEST PRIVATE WheelKinematics.cc)
endif()
