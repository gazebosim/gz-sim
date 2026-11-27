gz_add_system(mecanum-drive
  SOURCES
    MecanumDrive.cc
  TEST_SOURCES
    MecanumDrive_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
