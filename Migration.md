# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Gazebo 1.x to 2.x

* The `entity_name` field in the messages published by the imu system is
updated to report its scoped name.

## Ignition Gazebo 1.0.2 to 1.1.0

* All headers in `gazebo/network` are no longer installed.

* The ignition-gazebo1-gui library has been changed to a `gui` component of
ignition-gazebo. To use the gui component downstream, update the find package
call in cmake to request for the component, e.g.
`ign_find_package(ignition-gazebo1 REQUIRED COMPONENTS gui)`, and link to the
`libignition-gazebo1::gui` target instead of `libignition-gazebo1-gui`

