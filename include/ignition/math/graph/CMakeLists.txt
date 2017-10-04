include (${project_cmake_dir}/Utils.cmake)

set (headers
  Edge.hh
  Graph.hh
  GraphAlgorithms.hh
  Vertex.hh
)

foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Ignition math headers" "#include <ignition/math/graph/${hdr}>\n")
endforeach()

ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition/${IGN_PROJECT_NAME}/graph" ${headers})
