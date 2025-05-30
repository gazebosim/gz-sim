gz_add_system(triggered-publisher
  SOURCES
  TriggeredPublisher.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)
