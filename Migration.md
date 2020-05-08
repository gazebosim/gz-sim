# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Gazebo 2.x to 3.x

* Use ign-rendering3, ign-sensors3 and ign-gui3.

## Ignition Gazebo 1.x to 2.x

* Changed component data types:
    * `Altimeter` now uses `sdf::Sensor`
    * `JointVelocity` now uses `std::vector<double>`

* Deprecated components:
    * `JointVelocity2`: use `JointVelocity`'s vector instead.

* The `--distributed` command line argument has been deprecated. Use
  `--network-role` instead.

* The `-f`/`--file` command line argument has been deprecated. The SDF
  file can now be loaded without a flag.

* The `ign-gazebo` command line tool is deprecated. The new tool is
  `ign gazebo`, which has all the same options, except for
  `--distributed` and `--file`/`-f`, which have been removed.

* The `entity_name` field in the messages published by the imu system is
updated to report its scoped name.

* Log files generated from Ignition Gazebo 1.X are no longer compatible with
Gazebo 2+ for playback. [BitBucket pull request
#257](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/257)
added an SDF message to the start of log files.

## Ignition Gazebo 1.0.2 to 1.1.0

* All headers in `gazebo/network` are no longer installed.

* The ignition-gazebo1-gui library has been changed to a `gui` component of
ignition-gazebo. To use the gui component downstream, update the find package
call in cmake to request for the component, e.g.
`ign_find_package(ignition-gazebo1 REQUIRED COMPONENTS gui)`, and link to the
`libignition-gazebo1::gui` target instead of `libignition-gazebo1-gui`

