"""
Rules for System libraries.
"""

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

visibility("public")

def _generate_static_plugin_src_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.plugin_cc,
        output = ctx.outputs.out,
        substitutions = {
            # Macro substitutions:
            "GZ_ADD_PLUGIN": "GZ_ADD_STATIC_PLUGIN",
            "GZ_ADD_PLUGIN_ALIAS": "GZ_ADD_STATIC_PLUGIN_ALIAS",
            # Header substitutions:
            "plugin/Register.hh": "plugin/RegisterStatic.hh",
            "plugin/RegisterMore.hh": "plugin/RegisterStatic.hh",
        },
    )

# This rule performs a substitution to link the plugin class to the static
# plugin registry instead of the plugin hook registry for dynamic loading.
_generate_static_plugin_src = rule(
    attrs = {
        "plugin_cc": attr.label(allow_single_file = True, mandatory = True),
        "out": attr.output(mandatory = True),
    },
    implementation = _generate_static_plugin_src_impl,
)

def gz_sim_system_libraries(static_lib_name, so_lib_name, srcs, includes = [], **kwargs):
    """
    Adds two library targets for the System plugin for static and dynamic loading respectively

    Args:
        static_lib_name: Name of the `cc_library` target with static linking.
          Note that the plugin registration macro is substituted with
          `GZ_ADD_STATIC_PLUGIN` in the source file for this target to register
          the plugin with the static registry.
          The `alwayslink` attribute of this target is set to True, so that
          downstream linking preserves symbols which are not referenced
          explicitly.
        so_lib_name: Name of the `cc_binary` shared library target which can be
          loaded at runtime. Set this to empty string if the shared library
          target should not be added.
        srcs: List of source files including private headers. For example, this
          can be a globbed list of *.cc and *.hh files.
          ```
          srcs = glob(
              [
                  "src/systems/wheel_slip/**/*.cc",
                  "src/systems/wheel_slip/**/*.hh",
              ],
          ),
          ```
          Any test files should be excluded and can be added to separate
           `cc_test` targets.
        includes: List of include dirs to be added to the `cc_library` and
          `cc_binary` targets
        **kwargs: Forwarded to both the `cc_library` and `cc_binary` targets.
    """
    if not static_lib_name:
        fail("The static_lib_name field must be non-empty.")

    supported_cc_extensions = ["cc", "cpp"]
    cc_files = [f for f in srcs if f.split(".")[-1] in supported_cc_extensions]
    non_cc_files = [f for f in srcs if f not in cc_files]

    if not cc_files:
        fail("Did not find any .cc files in the provided srcs for library with static_lib_name '", static_lib_name, "'.")

    plugin_dir = "/".join(cc_files[0].split("/")[:-1])

    # Run the _generate_static_plugin_src rule to generate modified source files
    # suitable for registering the System plugin(s) with the static plugin
    # registry. Ideally the rule only needs to be run on source files which
    # register System plugins with the GZ_ADD_PLUGIN macro. However, in the
    # bazel analysis phase, there is no way to determine whether a particular
    # .cc file registers a System plugin or not. To circumvent this limitation,
    # the _generate_static_plugin_src rule is simply run for all source files.
    # This has a very small overhead of writing out the original file as is into
    # a new source file for the input source files which do not register a
    # System plugin.
    static_cc_files = []
    for cc_file in cc_files:
        name_without_extension = ".".join(cc_file.split(".")[:-1])
        static_plugin_src_gen_without_extension = name_without_extension + "_static_plugin"
        static_plugin_src_gen = static_plugin_src_gen_without_extension + ".cc"
        _generate_static_plugin_src(
            name = static_plugin_src_gen_without_extension,
            plugin_cc = cc_file,
            out = static_plugin_src_gen,
        )
        static_cc_files.append(static_plugin_src_gen)

    cc_library(
        name = static_lib_name,
        alwayslink = True,
        includes = includes + [plugin_dir],
        srcs = non_cc_files + static_cc_files,
        **kwargs
    )

    if so_lib_name:
        cc_binary(
            name = so_lib_name,
            linkshared = True,
            includes = includes,
            srcs = srcs,
            **kwargs
        )
