include (${project_cmake_dir}/Utils.cmake)

set (headers
  AffineException.hh
  Angle.hh
  Box.hh
  Filter.hh
  Helpers.hh
  IndexException.hh
  Kmeans.hh
  Line2.hh
  Matrix3.hh
  Matrix4.hh
  Plane.hh
  Pose3.hh
  Quaternion.hh
  Rand.hh
  RotationSpline.hh
  Spline.hh
  Triangle.hh
  Vector2.hh
  Vector3.hh
  Vector4.hh
)

set (ign_headers "" CACHE INTERNAL "Ignition math headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Ignition math headers" "#include <ignition/math/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/math.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/math.hh)

ign_install_includes("" ${CMAKE_CURRENT_BINARY_DIR}/math.hh)
ign_install_includes("math" ${headers})
