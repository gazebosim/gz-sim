file(GLOB gz_msgs_proto_files
  ${CMAKE_CURRENT_SOURCE_DIR}/gz/sim/private_msgs/*.proto
)

get_target_property(msgs_desc_file gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER} GZ_MSGS_DESC_FILE)

gz_msgs_generate_messages_impl(
  MSGS_GEN_SCRIPT
    ${gz-msgs10_PROTO_GENERATOR_SCRIPT}
  FACTORY_GEN_SCRIPT
    ${gz-msgs10_FACTORY_GENERATOR_SCRIPT}
  GZ_PROTOC_PLUGIN
    ${gz-msgs10_PROTO_GENERATOR_PLUGIN}
  INPUT_PROTOS
    ${gz_msgs_proto_files}
  PROTO_PACKAGE
    "gz.sim.private_msgs"
  PROTO_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_DIRECTORY
    ${PROJECT_BINARY_DIR}/gz_msgs_gen
  OUTPUT_SOURCES
    msgs_sources
  OUTPUT_HEADERS
    msgs_headers
  OUTPUT_DETAIL_HEADERS
    msgs_detail_headers
  OUTPUT_PYTHON
    msgs_python
  DEPENDENCY_DESCRIPTIONS ${msgs_desc_file}
)

set(msgs_sources ${msgs_sources} PARENT_SCOPE)
