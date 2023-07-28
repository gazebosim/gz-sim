file(GLOB gz_msgs_proto_files
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/*.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.msgs"
  MSGS_PATH ${PROJECT_SOURCE_DIR}/proto
  MSGS_PROTOS ${gz_msgs_proto_files}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}-msgs gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}-msgs gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}-msgs
)

install(
  DIRECTORY gz
  DESTINATION share/protos
  COMPONENT proto
  FILES_MATCHING PATTERN "*.proto")
