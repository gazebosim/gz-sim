file(GLOB component_headers *.hh)

set (component_includes "")
foreach (header_full ${component_headers})
  get_filename_component(header ${header_full} NAME)
  set(component_includes "${component_includes}#include <gz/sim/components/${header}>\n")
endforeach()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/components.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/components.hh
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/components.hh
  DESTINATION ${GZ_INCLUDE_INSTALL_DIR_FULL}/gz/sim
)
