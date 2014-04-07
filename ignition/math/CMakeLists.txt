include (${project_cmake_dir}/Utils.cmake)

set (headers
  Angle.hh
  Box.hh
  Helpers.hh
  MathTypes.hh
  Matrix3.hh
  Matrix4.hh
  Plane.hh
  Pose.hh
  Quaternion.hh
  Rand.hh
  RotationSpline.hh
  Spline.hh
  Vector2d.hh
  Vector2f.hh
  Vector2i.hh
  Vector3d.hh
  Vector3f.hh
  Vector3i.hh
  Vector4d.hh
  Vector4f.hh
  Vector4i.hh
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
