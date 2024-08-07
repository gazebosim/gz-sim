# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Ignition Gazebo 6.11.X to 6.12.X

 * **Modified**:
   + In the Hydrodynamics plugin, inverted the added mass contribution to make it
     act in the correct direction.

## Ignition Gazebo 6.1 to 6.2

* If no `<namespace>` is given to the `Thruster` plugin, the namespace now
  defaults to the model name, instead of an empty string.

## Ignition Gazebo 5.x to 6.x

* The ParticleEmitter system is deprecated. Please use the ParticleEmitter2
system.

* Marker example has been moved to Ignition GUI.

* Some GUI plugins have been moved to Ignition GUI. Gazebo users don't need to
  change their configuration files, the plugins will be loaded the same way.
    * Grid Config
    * Tape Measure

* `dynamic_pose/info` topic is removed from `LogRecord` and `LogPlayback`
since pose information is being logged in the `changed_state` topic.

* The internal management of entities and components in the
  `EntityComponentManager` has been updated to improve runtime performance. As a
  result, several methods have been deprecated, and a few types have changed.
  * **Deprecated**:
    + All `EntityComponentManager` methods that use `ComponentKey` as an input
      parameter.
    + The `EntityComponentManager::First` method.
    + The `ComponentId` and `ComponentKey` types are now deprecated. A
      combination of `Entity` and `ComponentTypeId` should be used instead.
    + The `components::StorageDescriptorBase` and
      `components::StorageDescriptor<ComponentTypeT>` classes.
    + Methods in `components::Factory` that have deprecated input parameter
      types and/or deprecated return types.
        - The version of `components::Factory::Register` which has a
          `StorageDescriptorBase *` input parameter.
        - `components::Factory::NewStorage`
    + The `ComponentStorageBase` and `ComponentStorage<ComponentTypeT>` classes.
  * **Modified**:
    + `EntityComponentManager::CreateComponent` now returns a pointer to the
      created component instead of a `ComponentKey`.
    + `ComponentKey` has been modified to be a
      `std::pair<ComponentTypeId, Entity>` (it used to be a
      `std::pair<ComponentTypeId, ComponentId>`) since the `ComponentId` type
      is now deprecated. `ComponentKey` has also been deprecated, so usage of
      this type is discouraged (see the **Deprecated** section above for more
      information about how to replace usage of `ComponentKey`).

* The `GzScene3D` GUI plugin is being deprecated in favor of `MinimalScene`. In
  order to get the same functionality as `GzScene3D`, users need to add the
  following plugins:
      + `MinimalScene`: base rendering functionality
      + `GzSceneManager`: adds / removes / moves entities in the scene
      + `EntityContextMenuPlugin`: right-click menu
      + `InteractiveViewControl`: orbit controls
      + `CameraTracking`: Move to, follow, set camera pose
      + `MarkerManager`: Enables the use of markers
      + `SelectEntities`: Select entities clicking on the scene
      + `Spawn`: Functionality to spawn entities into the scene via GUI
      + `VisualizationCapabilities`: View collisions, inertial, CoM, joints, etc.

    Moreover, legacy mode needs to be turned off for the following plugins
    for them to work with `MinimalScene` (set `<legacy>false</legacy>`):
      + `TransformControl`: Translate and rotate
      + `ViewAndle`: Move camera to preset angles

* The `gui.config` and `server.config` files are now located in a versioned
  folder inside `$HOME/.ignition/gazebo`, i.e. `$HOME/.ignition/gazebo/6/gui.config`.

## Ignition Gazebo 5.2 to 5.3

* If no `<namespace>` is given to the `Thruster` plugin, the namespace now
  defaults to the model name, instead of an empty string.

## Ignition Gazebo 4.x to 5.x

* Use `cli` component of `ignition-utils1`.

* `ignition::gazebo::RenderUtil::SelectedEntities()` now returns a
  `const std::vector<Entity> &` instead of forcing a copy. The calling code
  should create a copy if it needs to modify the vector in some way.

* Default generated topic name for thermal cameras now includes the `/image`
  suffix. The `camera_info` topic has also been fixed to include the sensor
  name in the generated topic string. The naming scheme should be consistent
  with a normal camera sensor. Topic changes:
    * `/<prefix>/<sensor_name>` -> `/<prefix>/<sensor_name>/image`
    * `/<prefix>/camera_info` -> `/<prefix>/<sensor_name>/camera_info`

* Various `GuiEvent`s were deprecated in favor of their Ignition GUI
  equivalents.
  * **Deprecated** `ignition::gazebo::gui::SnapIntervals`
  * **Replacement** `ignition::gui::SnapIntervals`
  * **Deprecated** `ignition::gazebo::gui::Render`
  * **Replacement** `ignition::gui::Render`
  * **Deprecated** `ignition::gazebo::gui::SpawnPreviewModel`
  * **Replacement** `ignition::gui::SpawnFromDescription`
  * **Deprecated** `ignition::gazebo::gui::SnapPreviewPath`
  * **Replacement** `ignition::gui::SnapFromPath`

* The `<direction>` tag of spot lights was previously not parsed by the
  scene, so all spot lights shone in the direction corresponding to the
  default `0 0 -1`. Since 5.x, the `<direction>` tag is correctly
  processed.

## Ignition Gazebo 4.0.0 to 4.X.X

* Ignition Gazebo 4.0.0 enabled double sided material by default but this
caused shadow artifacts to appear on some meshes. Double sided material is
now disabled and made an opt-in feature. Users can configure this property
in SDF by setting the `<visual><material><double_sided>` SDF element.

## Ignition Gazebo 3.x to 4.x

* The `RenderUtil::SetEnabledSensors` callback in gazebo rendering has a new
  required function argument for the Entity of the sensor.
    * ***Removed***
      `public: void SetEnableSensors(bool, std::function<
          std::string(const sdf::Sensor &, const std::string &)>)`
    * ***Replacement***
      `public: void SetEnableSensors(bool, std::function<
          std::string(const gazebo::Entity &,
          const sdf::Sensor &, const std::string &)>)`

* Log playback using `<path>` SDF parameter is removed. Use --playback command
  line argument instead.

* `rendering::SceneManager`
    * **Deprecated**: `Entity EntityFromNode(const rendering::NodePtr &_node) const;`
    * **Replacement**: `Entity entity = std::get<int>(visual->UserData("gazebo-entity"));`

## Ignition Gazebo 3.12.0 to 3.X.X

* Some sensors will only have the `SensorTopic` component after the 1st iteration.

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

* Log playback using `<path>` SDF parameter is deprecated. Use `--playback`
  command line argument instead.

## Ignition Gazebo 1.0.2 to 1.1.0

* All headers in `gazebo/network` are no longer installed.

* The ignition-gazebo1-gui library has been changed to a `gui` component of
ignition-gazebo. To use the gui component downstream, update the find package
call in cmake to request for the component, e.g.
`ign_find_package(ignition-gazebo1 REQUIRED COMPONENTS gui)`, and link to the
`libignition-gazebo1::gui` target instead of `libignition-gazebo1-gui`
