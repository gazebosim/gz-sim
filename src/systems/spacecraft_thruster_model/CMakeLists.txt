gz_add_system(spacecraft-thruster-model
  SOURCES
    SpacecraftThrusterModel.cc
  TEST_SOURCES
    SpacecraftThrusterModel_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
