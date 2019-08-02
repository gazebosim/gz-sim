## Ignition Gazebo 2.x

### Ignition Gazebo 2.X.X

1. Fix log playback with levels. This drops support for logs created before v2.0.0.
   * [Pull Request 407](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/407)

1. Add worker threads for System PostUpdate phase
   * [Pull Request 387](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/387)

1. Added a test runner for executing an SDF and recording simulation rates.
   See the `test/performance/READEM.md` file for more info.
   * [Pull Request 389](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/389)

### Ignition Gazebo 2.6.1 (2019-07-26)

1. Clear stepMsg before populating it
   * [Pull Request 398](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/398)

### Ignition Gazebo 2.6.0 (2019-07-24)

1.  Improve performance of Pose Publisher
   * [Pull Request 392](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/392)

1. Fix distributed sim
    * [Pull Request 385](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/385)

### Ignition Gazebo 2.5.0 (2019-07-19)

1. The LinearBatteryPlugin system publishes battery state
   * [Pull Request 388](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/388)

### Ignition Gazebo 2.4.0 (2019-07-17)

1. Bundle scene updates in sensor system
    * [Pull Request 386](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/386)

### Ignition Gazebo 2.3.0 (2019-07-13)

1. Improve physics system peformance by skipping static model updates.
   Components state information has been incorporated, which is used to
   indicate if a component change is periodic (such as through a physics
   update) or a one-time change (such as through a user command).
   * [Pull Request 384](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/384)

1. Add sdf parameter to battery to start draining only when robot has started moving
    * [Pull Request 370](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/370)

1. Improve SceneBroadcaster peformance by 1) Limit message generation if
   subscribers to pose topics are not present, 2) Set world stats message
   instead of copying the message, 3) Suppress scenegraph updates when there
   are no new entities, 4) Make better use of const functions, 5) Prevent
   creation of msgs::SerializedStep every PostUpdate, 6) Only serialized and
   transmit components that have changed.
    * [Pull Request 371](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/371)
    * [Pull Request 372](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/372)
    * [Pull Request 373](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/373)
    * [Pull Request 374](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/374)
    * [Pull Request 375](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/375)
    * [Pull Request 376](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/376)

### Ignition Gazebo 2.2.0

1. The DiffDrive system publishes odometry information.
    * [Pull Request 368](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/368)

1. Allow attaching plugins to sensors from a server config.
    * [Pull Request 366](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/366)

1. Remove world name from frame_ids
    * [Pull Request 364](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/364)

1. Fix deadlock when spawning robots
    * [Pull Request 365](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/365)

1. Set default topics for rendering sensors
    * [Pull Request 363](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/363)

1. Support custom random seed from the command line.
    * [Pull Request 362](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/362)

### Ignition Gazebo 2.1.0

1. RenderUtil fix bad merge: check for existing entities in GzScene3D on initialization.
    * [Pull Request 360](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/360)

1. Allow sensors to load plugins.
    * [Pull Request 356](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/356)
    * [Pull Request 366](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/366)

1. Parse and load submesh geometry in visuals.
    * [Pull Request 353](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/353)

1. Allow setting the update frequency of pose publisher.
    * [Pull Request 352](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/352)

1. Added RGBD camera sensor.
    * [Pull Request 351](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/351)

1. Fix Docker scripts.
    * [Pull Request 347](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/347)

1. Support log playback from a different path
    * [Pull Request 355](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/355)

### Ignition Gazebo 2.0.0

1. RenderUtil: check for existing entities in GzScene3D on initialization.
    * [Pull Request 350](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/350)

1. SceneBroadcaster: only send pose state periodically.
    * [Pull Request 345](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/345)

1. PeerTracker: increase distributed simulation peer tracking timeout.
    * [Pull Request 344](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/344)

1. MultiCopterMotorModel: add mutex to protect motor velocity command.
    * [Pull Request 341](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/341)

1. Tweaks to example worlds
    * [Pull Request 342](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/342)

1. DiffDrive system: add topic as system parameter.
    * [Pull Request 343](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/343)

1. Log entity creation and deletion
    * [Pull Request 337](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/337)

1. Multicopter motor model
    * [Pull Request 322](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/322)

1. Fix removing selected entity
    * [Pull Request 339](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/339)

1. Collision serialization
    * [Pull Request 326](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/326)

1. Add support for moving and rotating models
    * [Pull Request 316](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/316)

1. Pose commands
    * [Pull Request 334](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/334)

1. Level performers can be added at runtime using a service call. See the
   levels tutorial for more information.
    * [Pull Request 264](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/264)

1. Update worlds to GzScene3D
    * [Pull Request 333](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/333)

1. Reduce logging file size
    * [Pull Request 332](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/332)

1. Update PosePublisher system to publish sensor poses and to use scoped names for frame ids
    * [Pull Request 331](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/331)

1. Fix gui plugin linking issue
    * [Pull Request 327](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/327)
    * [Pull Request 330](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/330)

1. Toolbar colors
    * [Pull Request 329](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/329)

1. Rename Scene3D gui plugin to GzScene3D
    * [Pull Request 328](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/328)

1. Fix distributed sim documentation
    * [Pull Request 318](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/318)

1. Port Scene3D gui plugin from ign-gui. Renamed to GzScene3D.
    * [Pull Request 315](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/315)

1. Entity tree UI
    * [Pull Request 285](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/285)

1. Add rendering component
    * [Pull Request 306](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/306)

1. Update Camera and DepthCamera components to use sdf::Sensor object instead of an sdf::ElementPtr.
    * [Pull Request 299](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/299)

1. Added system for ignition::sensors::AirPressureSensor.
    * [Pull Request 300](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/300)

1. Support conversion and serialization of Imu components. IMU sensors are
   loaded from an SDF DOM object.
    * [Pull Request 302](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/302)

1. Throttle sensors update rate
    * [Pull Request 323](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/323)

1. Fix changing themes
    * [Pull Request 321](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/321)

1. Battery tweaks
    * [Pull Request 314](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/314)

1. Support conversion and serialization of PBR parameters in a material component
    * [Pull Request 304](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/304)

1. Joint state pub
    * [Pull Request 260](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/260)

1. Update Altimeter component to use sdf::Sensor object instead of an
   sdf::ElementPtr.
    * [Pull Request 286](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/286)

1. Update docker nightly dependencies
    * [Pull Request 310](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/310)

1. Ign tool
    * [Pull Request 296](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/296)
    * [Pull Request 336](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/336)

1. State broadcast
    * [Pull Request 307](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/307)

1. Use world statistics message on network
    * [Pull Request 305](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/305)

1. Update Magnetometer component to use sdf::Sensor object instead of an sdf::ElementPtr.
    * [Pull Request 272](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/272)

1. Fix Scene3D loading empty world
    * [Pull Request 308](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/308)

1. Support conversion and serialization of scene and light components
    * [Pull Request 297](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/297)

1. Operators instead of De/Serialize
    * [Pull Request 293](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/293)

1. Remove PIMPL from Component
    * [Pull Request 267](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/267)

1. Delay scene broadcaster transport setup
    * [Pull Request 292](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/292)

1. Report link poses from secondaries during distributed simulation, using a cache
    * [Pull Request 276](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/276)
    * [Pull Request 265](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/265)

1. Restore log playback
    * [Pull Request 288](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/288)

1. ECM changed state
    * [Pull Request 287](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/287)

1. Joint serialization
    * [Pull Request 281](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/281)

1. Use scene ambient and background color information in sensor
   configuration.
    * [Pull Request 268](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/268)

1. Performance benchmarking
    * [Pull Request 220](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/220)
    * [Pull Request 253](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/253)
    * [Pull Request 258](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/258)
    * [Pull Request 283](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/283)
    * [Pull Request 312](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/312)

1. Remove emissive component from visual materials
    * [Pull Request 271](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/271)

1. Serialization for more components
    * [Pull Request 255](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/255)

1. Added an SDF message to the start of log files.
    * [Pull Request 257](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/257)

1. Unify network and sync managers
    * [Pull Request 261](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/261)

1. Add PerformerLevels component
    * [Pull Request 262](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/262)

1. Distributed sim deprecate envs
    * [Pull Request 240](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/240)

1. Use ign-sensors magnetometer sensor plugin
    * [Pull Request 221](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/221)

1. Use ign-sensors altimeter sensor plugin
    * [Pull Request 215](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/215)

1. Use ign-sensors imu sensor plugin
    * [Pull Request 219](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/219)

1. Depend on ign-sensors rendering component
    * [Pull Request 212](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/212)

## Ignition Gazebo 1.x

### Ignition Gazebo 1.X.X

1. Add Wind Plugin (Ported from Gazebo classic)
    * [Pull Request 273](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/273/)

1. Port battery plugin from Gazebo classic
    * [Pull request 234](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/234)
    * [Pull request 317](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/317)
    * [Pull request 324](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/324)

1. Use ISO timestamp for default log path
    * [Pull request 289](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/289)

1. Logging tutorial
    * [Pull request 280](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/280)

1. Joystick SDF small typos
    * [Pull request 284](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/284)

1. Add `Link`: a convenience class for interfacing with link entities
    * [Pull Request 269](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/269)

1. Added LiftDragPlugin (ported from Gazebo classic)
    * [Pull Request 256](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/256)

1. Logging refactor unique path functions to ign-common
    * [Pull request 270](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/270)

1. Added test for log record and playback.
    * [Pull Request 263](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/263)

1. Add ApplyJointForce system
    * [Pull request 254](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/254)

1. More ign-msgs <-> SDF conversions: Inertial, Geometry, Material
    * [Pull Request 251](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/251)

1. Logging command line support
    * [Pull request 249](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/249)

1. Remove inactive performers instead of setting static
    * [Pull request 247](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/247)

1. Use state instead of pose in distributed simulation
    * [Pull request 242](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/242)

1. Distributed implies levels
    * [Pull request 243](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/243)

1. Add a basic JointController system
    * [Pull Request 246](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/246)

1. Enforce component type uniqueness
    * [Pull request 236](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/236)

1. Clean CI: disable test known to fail on OSX
    * [Pull request 244](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/244)

1. Logical camera topic name check
    * [Pull request 245](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/245)

1. Added command line options to configure distributed simulation. These
   will replace the environment variables.
    * [Pull Request 238](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/238)

1. Add systems to queue before actually adding them to runner
    * [Pull request 241](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/241)

1. Added a docker image that uses the ignition meta package
    * [Pull request 237](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/237)

1. Move some design docs to tutorials
    * [Pull request 230](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/230)

1. Disable GUI when using distributed simulation
    * [Pull request 235](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/235)

1. Bring component type names back
    * [Pull request 232](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/232)

1. A few tweaks to logging
    * [Pull request 228](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/228)

1. Handle friction coefficients
    * [Pull request 227](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/227)

1. Change private msgs namespace
    * [Pull request 233](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/233)

1. Set tutorial titles
    * [Pull request 231](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/231)

1. Example tunnel world
    * [Pull request 205](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/205)

1. Conversion from chrono to ign-msgs
    * [Pull request 223](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/223)

1. Prevent error message when using levels
    * [Pull request 229](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/229)

### Ignition Gazebo 1.1.0 (2019-03-15)

1. Distributed performers running in lockstep
    * [Pull Request 186](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/186)
    * [Pull Request 201](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/201)
    * [Pull Request 209](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/209)
    * [Pull Request 213](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/213)

1. Fix documentation tagfiles
    * [Pull Request 214](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/214)

1. Convert gui library into a component
    * [Pull Request 206](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/206)

1. include <cstdint> wherever special int types like uint64_t are used
    * [Pull Request 208](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/208)

1. Move network internal
    * [Pull Request 211](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/211)

1. Logging / playback
    * [Pull Request 181](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/181)

1. ECM state streaming
    * [Pull Request 184](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/184)

1. Unversioned system libraries
    * [Pull Request 222](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/222)

### Ignition Gazebo 1.0.2 (2019-03-12)

1. Use TARGET_SO_NAME to fix finding dartsim plugin
    * [Pull Request 217](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/217)

### Ignition Gazebo 1.0.1 (2019-03-01)

1. Update gazebo version number in sdf files
    * [Pull Request 207](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/207)

### Ignition Gazebo 1.0.0 (2019-03-01)

1. Initial release

## Ignition Gazebo 0.x

### Ignition Gazebo 0.1.0

1. Add support for joints
    * [Pull Request 77](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/77)

1. Use SimpleWrapper for more component types
    * [Pull Request 78](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/78)

1. Create EventManager and delegate System instantiation to SimulationRunner
    * [Pull Request 79](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests/79)

1. Integrate ign-gui
    * [Pull request 11](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/11)

1. Remove some build dependencies.
    * [Pull request 6](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/6)

1. Added basic Entity class.
    * [Pull request 3](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/3)

1. Added a basic System class.
    * [Pull request 4](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-request/4)
