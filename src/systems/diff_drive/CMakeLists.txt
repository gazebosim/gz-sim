gz_add_system(diff-drive
  SOURCES
    DiffDrive.cc
  TEST_SOURCES
    DiffDrive_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)
