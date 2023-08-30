file(GLOB gz_msgs_proto_files
  ${CMAKE_CURRENT_SOURCE_DIR}/gz/sim/private_msgs/*.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.sim.private_msgs"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}
  MSGS_PROTOS ${gz_msgs_proto_files}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
