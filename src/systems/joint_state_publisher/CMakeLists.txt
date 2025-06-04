gz_add_system(joint-state-publisher
  SOURCES
  JointStatePublisher.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
