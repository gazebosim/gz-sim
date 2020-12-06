load(
    "//ign_bazel:qt.bzl",
    "qt_cc_binary",
    "qt_cc_library",
)

def ign_gazebo_gui_plugin(name, dirname, srcs, hdrs, resources, deps):
    native.genrule(
        name = "%s_gen" % dirname,
        srcs = resources,
        outs = ["qrc_%s.cpp" % name],
        cmd = "cd ign_gazebo/src/gui/plugins/%s && qtchooser -qt=5 -run-tool=rcc --name %s %s.qrc -o qrc_%s.cpp && cp qrc_%s.cpp ../../../../../$(OUTS)" % (dirname, name, name, name, name),
    )

    qt_cc_binary(
        name = "lib%s.so" % name,
        srcs = srcs + ["qrc_%s.cpp" % name],
        hdrs = hdrs,
        linkopts = ["-ltinyxml2"],
        linkshared = True,
        includes = ["."],
        data = ["IgnGazebo/qmldir"],
        deps = deps + [
            "//sdformat",
            "//ign_msgs",
            "//ign_common",
            "//ign_common/av",
            "//ign_common/events",
            "//ign_common/graphics",
            "//ign_common/profiler",
            "//ign_math",
            "//ign_rendering",
            "//ign_rendering/ogre2",
            "//ign_transport",
            "//ign_gui",
            "//ign_gazebo:ign_gazebo",
            "//ign_gazebo:ign_gazebo_gui",
            "@qt//:qt_core",
            "@qt//:qt_network",
            "@qt//:qt_widgets",
            "@qt//:qt_quick_control",
            "@qt//:qt_quick",
            "@qt//:qt_qml",
            "@qt//:qt_gui",
            "@qt//:qt_opengl",
        ],
    )
