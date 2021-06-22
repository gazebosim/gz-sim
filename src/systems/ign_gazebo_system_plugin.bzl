load( "//ign_bazel:build_defs.bzl",
     "IGNITION_ROOT",
)

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary", "cc_test")

SYSTEM_COMMON_DEPS = [
    IGNITION_ROOT + "ign_common",
    IGNITION_ROOT + "ign_common/av",
    IGNITION_ROOT + "ign_common/events",
    IGNITION_ROOT + "ign_common/profiler",
    IGNITION_ROOT + "ign_math",
    IGNITION_ROOT + "ign_math/eigen3",
    IGNITION_ROOT + "ign_msgs",
    IGNITION_ROOT + "ign_gazebo",
    IGNITION_ROOT + "ign_gazebo:ign_gazebo_headers",
    IGNITION_ROOT + "ign_plugin/register",
    IGNITION_ROOT + "ign_transport",
    IGNITION_ROOT + "sdformat",
]

def ign_gazebo_system_plugin(folder_name, deps = [], data = []):
  cc_library(
      name = folder_name,
      srcs = native.glob([ folder_name + "/*.cc" ], 
                     exclude = [folder_name + "/*_TEST.cc"]),
      hdrs = native.glob([ folder_name + "/*.hh" ]),
      includes = [ folder_name ],
      deps = deps + SYSTEM_COMMON_DEPS,
      alwayslink = 1,
  )

  cc_binary(
      name = "libignition-gazebo-%s-system.so" % '-'.join(folder_name.split('_')),
      deps = [":" + folder_name],
      data = data,
      linkshared = 1,
  )
