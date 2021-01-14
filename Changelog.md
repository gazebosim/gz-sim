## Ignition Gazebo 3.x

### Ignition Gazebo 3.7.0 (2021-01-13)

1. Fix examples in migration plugins tutorial.
    * [Pull Request 543](https://github.com/ignitionrobotics/ign-gazebo/pull/543)

1. Added missing namespace in `detail/EntityComponentManager.hh`.
    * [Pull Request 541](https://github.com/ignitionrobotics/ign-gazebo/pull/541)

1. Automatically load a subset of world plugins.
    * [Pull Request 281](https://github.com/ignitionrobotics/ign-gazebo/pull/281)

1. Update gtest to 1.10.0 for Windows compilation.
    * [Pull Request 506](https://github.com/ignitionrobotics/ign-gazebo/pull/506)

1. Updates to ardupilot migration tutorial.
    * [Pull Request 525](https://github.com/ignitionrobotics/ign-gazebo/pull/525)

1. Don't make docs on macOS.
    * [Pull Request 528](https://github.com/ignitionrobotics/ign-gazebo/pull/528)

### Ignition Gazebo 3.6.0 (2020-12-30)

1. Fix pose msg conversion when msg is missing orientation
    * [Pull Request 450](https://github.com/ignitionrobotics/ign-gazebo/pull/450)

1. Address code checker warnings
    * [Pull Request 443](https://github.com/ignitionrobotics/ign-gazebo/pull/443)
    * [Pull Request 491](https://github.com/ignitionrobotics/ign-gazebo/pull/491)
    * [Pull Request 499](https://github.com/ignitionrobotics/ign-gazebo/pull/499)
    * [Pull Request 502](https://github.com/ignitionrobotics/ign-gazebo/pull/502)

1. Test fixes
    * [Pull Request 455](https://github.com/ignitionrobotics/ign-gazebo/pull/455)
    * [Pull Request 463](https://github.com/ignitionrobotics/ign-gazebo/pull/463)
    * [Pull Request 452](https://github.com/ignitionrobotics/ign-gazebo/pull/452)
    * [Pull Request 480](https://github.com/ignitionrobotics/ign-gazebo/pull/480)

1. Documentation updates
    * [Pull Request 472](https://github.com/ignitionrobotics/ign-gazebo/pull/472)

1. Fix segfault in the Breadcrumb system when associated model is unloaded
    * [Pull Request 454](https://github.com/ignitionrobotics/ign-gazebo/pull/454)

1. Added user commands to example thermal camera world
    * [Pull Request 442](https://github.com/ignitionrobotics/ign-gazebo/pull/442)

1. Helper function to set component data
    * [Pull Request 436](https://github.com/ignitionrobotics/ign-gazebo/pull/436)

1. Remove unneeded if statement in EntityComponentManager
    * [Pull Request 432](https://github.com/ignitionrobotics/ign-gazebo/pull/432)

1. Clarify how time is represented in each phase of a System step
    * [Pull Request 467](https://github.com/ignitionrobotics/ign-gazebo/pull/467)

1. Switch to async state service request
    * [Pull Request 461](https://github.com/ignitionrobotics/ign-gazebo/pull/461)

1. Update key event handling
    * [Pull Request 466](https://github.com/ignitionrobotics/ign-gazebo/pull/466)

1. Tape Measure Plugin
    * [Pull Request 456](https://github.com/ignitionrobotics/ign-gazebo/pull/456)

1. Move deselect and preview termination to render thread
    * [Pull Request 493](https://github.com/ignitionrobotics/ign-gazebo/pull/493)

1. Logical audio sensor plugin
    * [Pull Request 401](https://github.com/ignitionrobotics/ign-gazebo/pull/401)

1. add frame_id and child_frame_id attribute support for DiffDrive
    * [Pull Request 361](https://github.com/ignitionrobotics/ign-gazebo/pull/361)

1. Add ability to record video based on sim time
    * [Pull Request 414](https://github.com/ignitionrobotics/ign-gazebo/pull/414)

1. Add lockstep mode to video recording
    * [Pull Request 419](https://github.com/ignitionrobotics/ign-gazebo/pull/419)

1. Disable right click menu when using measuring tool
    * [Pull Request 458](https://github.com/ignitionrobotics/ign-gazebo/pull/458)

### Ignition Gazebo 3.5.0 (2020-11-03)

1. Updated source build instructions
    * [Pull Request 403](https://github.com/ignitionrobotics/ign-gazebo/pull/403)

1. More world APIs, helper function ComponentData
    * [Pull Request 378](https://github.com/ignitionrobotics/ign-gazebo/pull/378)

1. Improve fork experience
    * [Pull Request 411](https://github.com/ignitionrobotics/ign-gazebo/pull/411)

1. Fix a crash in the grid config plugin, set grid material
    * [Pull Request 412](https://github.com/ignitionrobotics/ign-gazebo/pull/412)

1. Document deprecation of log playback `<path>` SDF param
    * [Pull Request 424](https://github.com/ignitionrobotics/ign-gazebo/pull/424)
    * [Pull Request 425](https://github.com/ignitionrobotics/ign-gazebo/pull/425)

1. Enable mouse highlighting selection on resource spawner
    * [Pull Request 402](https://github.com/ignitionrobotics/ign-gazebo/pull/402)

1. Add support for custom render engines
    * [Pull Request 373](https://github.com/ignitionrobotics/ign-gazebo/pull/373)

1. Component Vector -> Map ECM Optimization
    * [Pull Request 416](https://github.com/ignitionrobotics/ign-gazebo/pull/416)

### Ignition Gazebo 3.4.0 (2020-10-14)

1. Fix gui sendEvent memory leaks
    * [Pull Request 365](https://github.com/ignitionrobotics/ign-gazebo/pull/365)

1. Support nested models
    * [Pull Request 258](https://github.com/ignitionrobotics/ign-gazebo/pull/258)

1. Generalize actor count and pose in actor population erb SDF
    * [Pull Request 336](https://github.com/ignitionrobotics/ign-gazebo/pull/336)

1. Add more link APIs, with tutorial
    * [Pull Request 375](https://github.com/ignitionrobotics/ign-gazebo/pull/375)

1. Add screenshots to GUI config tutorial
    * [Pull Request 406](https://github.com/ignitionrobotics/ign-gazebo/pull/406)

1. Fix adding performers to entity tree
    * [Pull Request 374](https://github.com/ignitionrobotics/ign-gazebo/pull/374)

1. Remove sidebar and put world control in bottom left for joint controller examples
    * [Pull Request 384](https://github.com/ignitionrobotics/ign-gazebo/pull/384)

1. Allow executing a blocking single Server run in both paused and unpaused states
    * [Pull Request 297](https://github.com/ignitionrobotics/ign-gazebo/pull/297)

1. Add camera video recorder system
    * [Pull Request 316](https://github.com/ignitionrobotics/ign-gazebo/pull/316)

1. Decrease time step for quadcopter world
    * [Pull Request 372](https://github.com/ignitionrobotics/ign-gazebo/pull/372)

1. Add support for moving the GUI camera to a pose
    * [Pull Request 352](https://github.com/ignitionrobotics/ign-gazebo/pull/352)

1. Remove `lib`+`.so` from plugin's name
    * [Pull Request 279](https://github.com/ignitionrobotics/ign-gazebo/pull/279)
    * [Pull Request 335](https://github.com/ignitionrobotics/ign-gazebo/pull/335)

1. EntityComponentManager::EachRemoved documentation fix.
    * [Pull Request 348](https://github.com/ignitionrobotics/ign-gazebo/pull/348)

1. Add more model APIs.
    * [Pull Request 349](https://github.com/ignitionrobotics/ign-gazebo/pull/349)

1. Update dimensions of the grid config.
    * [Pull Request 383](https://github.com/ignitionrobotics/ign-gazebo/pull/383)

1. Fix top-left toolbar layout so magnet shows.
    * [Pull Request 381](https://github.com/ignitionrobotics/ign-gazebo/pull/381)

1. Add instructions to bitmask world.
    * [Pull Request 377](https://github.com/ignitionrobotics/ign-gazebo/pull/377)

1. Add search and sort for resource spawner.
    * [Pull Request 359](https://github.com/ignitionrobotics/ign-gazebo/pull/359)

1. Fix source build instructions for ign-gazebo3.
    * [Pull Request 395](https://github.com/ignitionrobotics/ign-gazebo/pull/395)

1. Added playback scrubber GUI
    * [Pull Request 299](https://github.com/ignitionrobotics/ign-gazebo/pull/299)
    * [Pull Request 362](https://github.com/ignitionrobotics/ign-gazebo/pull/362)

1. Added wheel slip system plugin.
    * [Pull Request 134](https://github.com/ignitionrobotics/ign-gazebo/pull/134)
    * [Pull Request 357](https://github.com/ignitionrobotics/ign-gazebo/pull/357)
    * [Pull Request 362](https://github.com/ignitionrobotics/ign-gazebo/pull/362)

1. Enhanced log playback performance.
    * [Pull Request 351](https://github.com/ignitionrobotics/ign-gazebo/pull/351)
    * [Pull Request 362](https://github.com/ignitionrobotics/ign-gazebo/pull/362)

1. Tests & Warnings: Qt 5.14, breadcrumbs, Gui, ign_TEST
    * [Pull Request 327](https://github.com/ignitionrobotics/ign-gazebo/pull/327)

1. Added support for specifying topics to record.
    * [Pull Request 315](https://github.com/ignitionrobotics/ign-gazebo/pull/315)

1. Make sure OpenGL core profile context is used by GzScene3D.
    * [Pull Request 339](https://github.com/ignitionrobotics/ign-gazebo/pull/339)

1. Support relative paths for PBR materials
    * [Pull Request 328](https://github.com/ignitionrobotics/ign-gazebo/pull/328)
    * [Pull Request 362](https://github.com/ignitionrobotics/ign-gazebo/pull/362)

1. Add file extension automatically for record plugin.
    * [Pull Request 303](https://github.com/ignitionrobotics/ign-gazebo/pull/303)
    * [Pull Request 362](https://github.com/ignitionrobotics/ign-gazebo/pull/362)

1. Support spawning during log playback.
    * [Pull Request 346](https://github.com/ignitionrobotics/ign-gazebo/pull/346)

1. Added wheel slip system plugin.
    * [Pull Request 134](https://github.com/ignitionrobotics/ign-gazebo/pull/134)
    * [Pull Request 357](https://github.com/ignitionrobotics/ign-gazebo/pull/357)

1. Add Render Engine Cmd Line option
    * [Pull Request 331](https://github.com/ignitionrobotics/ign-gazebo/pull/331)

### Ignition Gazebo 3.3.0 (2020-08-31)

1. Added marker array service.
    * [pull request 302](https://github.com/ignitionrobotics/ign-gazebo/pull/302)

1. Introduced a new parameter in the scene3D plugin to launch in fullscreen.
    * [pull request 254](https://github.com/ignitionrobotics/ign-gazebo/pull/254)

1. Fix issue #285 by adding checks for a marker's parent.
    * [pull request 290](https://github.com/ignitionrobotics/ign-gazebo/pull/290)

1. Fix non-specified material error.
    * [pull request 292](https://github.com/ignitionrobotics/ign-gazebo/pull/292)

1. Added simulation world with large number of entities.
    * [pull request 283](https://github.com/ignitionrobotics/ign-gazebo/pull/283)

1. Fixed parsing of the touch plugin' enabled flag.
    * [pull request 275](https://github.com/ignitionrobotics/ign-gazebo/pull/275)

1. Added buoyancy system plugin.
    * [pull request 252](https://github.com/ignitionrobotics/ign-gazebo/pull/252)

1. Implemented shift + drag = rotate in the GUI.
    * [pull request 247](https://github.com/ignitionrobotics/ign-gazebo/pull/247)

1. Backport collision bitmask changes
    * [pull request 223](https://github.com/ignitionrobotics/ign-gazebo/pull/223)

1. Added velocity command to TPE.
    * [pull request 169](https://github.com/ignitionrobotics/ign-gazebo/pull/169)

1. This version includes all features in Gazebo 2.23.0

### Ignition Gazebo 3.2.0 (2020-05-20)

1. Merge ign-gazebo2 to ign-gazebo3
    * [pull request 149](https://github.com/ignitionrobotics/ign-gazebo/pull/149)

### Ignition Gazebo 3.1.0 (2020-05-19)

1. Port support for computing model bounding box in physics system
    * [pull request 127](https://github.com/ignitionrobotics/ign-gazebo/pull/127)

1.  Add DetachableJoint: A system that initially attaches two models via a fixed joint and allows for the models to get detached during simulation via a topic.
    * [BitBucket pull request 440](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/440)

1. Update physics state even when paused (not stepping)
    * [BitBucket pull request 556](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/556)

1. Fix entity tree context menu position
    * [BitBucket pull request 567](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/567)

1. Fix moving static model with link offset
    * [BitBucket pull request 566](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/566)

1. Added Link::AddWorldWrench function that adds a wrench to a link.
    * [BitBucket pull request 509](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/509)

1. Fix duplicate marker services and crash due to unset marker field
    * [BitBucket pull request 561](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/561)

1. Support <uri>s from Fuel
    * [BitBucket pull request 532](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/532)

1. Add support for thermal camera
    * [BitBucket pull request 512](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/512)
    * [BitBucket pull request 513](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/513)
    * [BitBucket pull request 514](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/514)

1. Add window focus upon mouse entering the render window
    * [Github pull request 96](https://github.com/ignitionrobotics/ign-gazebo/pull/96)

### Ignition Gazebo 3.0.0 (2019-12-10)

1. Add example world for collide bitmask feature
    * [BitBucket pull request 525](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/525)

1. Remove <emissive> sdf element from visuals that do not emit light in the example worlds
    * [BitBucket pull request 478](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/478)
    * [BitBucket pull request 480](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/480)

1. Support for sdformat frame semantics
    * [BitBucket pull request 456](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/456)

1. Support for relative path URIs for actors
    * [BitBucket pull request 444](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/444)

1. Add rechargeable battery model
    * [BitBucket pull request 457](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/457)

1. Add Marker Manager
    * [BitBucket pull request 442](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/442)

1. Parse material emissive map, bump to msgs5 and transport8
    * [BitBucket pull request 447](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/447)

1. Move function definitions to their correct locations in EntityComponentManager
    * [BitBucket pull request 380](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/380)

1. Depend on ign-rendering3, ign-gui3, ign-sensors3
    * [BitBucket pull request 411](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/411)

1. Rendering and Animating Actors
    * [BitBucket pull request 414](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/414)


## Ignition Gazebo 2.x

### Ignition Gazebo 2.25.0 (2020-09-17)

1. Added wheel slip system plugin.
    * [Pull Request 134](https://github.com/ignitionrobotics/ign-gazebo/pull/134)
    * [Pull Request 357](https://github.com/ignitionrobotics/ign-gazebo/pull/357)

1. Enhanced log playback performance.
    * [Pull Request 351](https://github.com/ignitionrobotics/ign-gazebo/pull/351)

1. Tests & Warnings: Qt 5.14, breadcrumbs, Gui, ign_TEST
    * [Pull Request 327](https://github.com/ignitionrobotics/ign-gazebo/pull/327)

1. Added support for specifying topics to record.
    * [Pull Request 315](https://github.com/ignitionrobotics/ign-gazebo/pull/315)

1. Make sure OpenGL core profile context is used by GzScene3D.
    * [Pull Request 339](https://github.com/ignitionrobotics/ign-gazebo/pull/339)

1. Support relative paths for PBR materials
    * [Pull Request 328](https://github.com/ignitionrobotics/ign-gazebo/pull/328)

1. Add file extension automatically for record plugin.
    * [Pull Request 303](https://github.com/ignitionrobotics/ign-gazebo/pull/303)

1. Support spawning during log playback.
    * [Pull Request 346](https://github.com/ignitionrobotics/ign-gazebo/pull/346)

### Ignition Gazebo 2.24.0 (2020-09-03)

1. Resource env var, with transport interface.
    * [Pull Request 172](https://github.com/ignitionrobotics/ign-gazebo/pull/172)

1. Save http URIs (fix tests)
    * [Pull Request 271](https://github.com/ignitionrobotics/ign-gazebo/pull/271)

1. Insert Local Models.
    * [Pull Request 173](https://github.com/ignitionrobotics/ign-gazebo/pull/173)

1. Modernize actions CI.
    * [Pull Request 269](https://github.com/ignitionrobotics/ign-gazebo/pull/269)

1. Sensor topics available through components and GUI.
    * [Pull Request 266](https://github.com/ignitionrobotics/ign-gazebo/pull/266)

1. Customizable layouts - fully functional.
    * [Pull Request 278](https://github.com/ignitionrobotics/ign-gazebo/pull/278)

1. Add Fuel World Support.
    * [Pull Request 274](https://github.com/ignitionrobotics/ign-gazebo/pull/274)

1. Insert Fuel Models.
    * [Pull Request 263](https://github.com/ignitionrobotics/ign-gazebo/pull/263)

1. Disable rendering tests on macOS that are known to fail.
    * [Pull Request 209](https://github.com/ignitionrobotics/ign-gazebo/pull/209)

1. Fix tests on Blueprint.
    * [Pull Request 295](https://github.com/ignitionrobotics/ign-gazebo/pull/295)

1. Publish remaining breadcrumb deployments.
    * [Pull Request 308](https://github.com/ignitionrobotics/ign-gazebo/pull/308)

### Ignition Gazebo 2.23.0 (2020-07-28)

1. Deactivate PerformerDetector if its parent model gets removed.
    * [Pull Request 260](https://github.com/ignitionrobotics/ign-gazebo/pull/260)

1. Backport support for <uri>s from Fuel #255
    * [Pull Request 255](https://github.com/ignitionrobotics/ign-gazebo/pull/255)

### Ignition Gazebo 2.22.0 (2020-07-22)

1. Allow zero or more key/value pairs to be added to detection header information.
    * [Pull Request 257](https://github.com/ignitionrobotics/ign-gazebo/pull/257)

### Ignition Gazebo 2.21.0 (2020-07-16)

1. Added support for controlling which joints are published by the
   JointStatePublisher.
    * [Pull Request 222](https://github.com/ignitionrobotics/ign-gazebo/pull/222)

1. Added an additional pose offset for the performer detector plugin.
    * [Pull Request 236](https://github.com/ignitionrobotics/ign-gazebo/pull/236)

1. Fixed battery issues and updated tutorial.
    * [Pull Request 230](https://github.com/ignitionrobotics/ign-gazebo/pull/230)

### Ignition Gazebo 2.20.1 (2020-06-18)

1. Properly add new models into the scenegraph. With this fix, when a model is spawned it will be added into the graph and resulting calls to the `scene/info` service will return a correct `msgs::Scene`.
    * [Pull Request 212](https://github.com/ignitionrobotics/ign-gazebo/pull/212)

### Ignition Gazebo 2.20.0 (2020-06-09)

1. Updated battery model to stop battery drain when there is no joint
   velocity/force command, and added a recharging trigger.
    * [Pull Request 183](https://github.com/ignitionrobotics/ign-gazebo/pull/183)

1. Fix segfault in the Breadcrumbs system
    * [Pull Request 180](https://github.com/ignitionrobotics/ign-gazebo/pull/180)

1. Added an `<odom_topic>` element to the DiffDrive system so that a custom odometry topic can be used.
    * [Pull Request 179](https://github.com/ignitionrobotics/ign-gazebo/pull/179)

### Ignition Gazebo 2.19.0 (2020-06-02)

1. Use updated model names for spawned models when generating SDFormat
    * [Pull Request 166](https://github.com/ignitionrobotics/ign-gazebo/pull/166)

1. Allow joint force commands (JointForceCmd) to dscharge a battery.
    * [Pull Request 165](https://github.com/ignitionrobotics/ign-gazebo/pull/165)

1. Allow renaming breadcrumb models if there is a name conflict
    * [Pull Request 155](https://github.com/ignitionrobotics/ign-gazebo/pull/155)

1. Add TriggeredPublisher system
    * [Pull Request 139](https://github.com/ignitionrobotics/ign-gazebo/pull/139)

1. Add PerformerDetector, a system for detecting when performers enter a specified region
    * [Pull Request 125](https://github.com/ignitionrobotics/ign-gazebo/pull/125)

### Ignition Gazebo 2.18.0 (2020-05-20)

1. Added a `/world/<world_name>/create_multiple` service that parallels the current `/world/<world_name>/create` service. The `create_multiple` service can handle an `ignition::msgs::EntityFactory_V` message that may contain one or more entities to spawn.
    * [Pull Request 146](https://github.com/ignitionrobotics/ign-gazebo/pull/146)

1. DetachableJoint system: Add option to suppress warning about missing child model
    * [Pull Request 132](https://github.com/ignitionrobotics/ign-gazebo/pull/132)

### Ignition Gazebo 2.17.0 (2020-05-13)

1. Allow battery plugin to work with joint force systems.
    * [Pull Request 120](https://github.com/ignitionrobotics/ign-gazebo/pull/120)

1. Make breadcrumb static after specified time
    * [Pull Request 90](https://github.com/ignitionrobotics/ign-gazebo/pull/90)

1. Disable breadcrumbs if the `max_deployments` == 0.
    * [Pull Request 88](https://github.com/ignitionrobotics/ign-gazebo/pull/88)

1. Add static pose publisher and support pose\_v msg type in pose publisher system
    * [Pull Request 65](https://github.com/ignitionrobotics/ign-gazebo/pull/65)

1. Refactor Gui.hh so that the Gazebo GUI can be ran from other packages
    * [Pull Request 79](https://github.com/ignitionrobotics/ign-gazebo/pull/79)

1. Add ability to save worlds to SDFormat
    * [BitBucket pull request 545](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/545)

1. Add window focus upon mouse entering the render window
    * [Github pull request 95](https://github.com/ignitionrobotics/ign-gazebo/pull/95)

### Ignition Gazebo 2.16.0 (2020-03-24)

1. Add support for computing model bounding box in physics system
    * [BitBucket pull request 546](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/546)

1. Add DetachableJoint: A system that initially attaches two models via a fixed joint and allows for the models to get detached during simulation via a topic.
    * [BitBucket pull request 440](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/440)

1. Update physics state even when paused (not stepping)
    * [BitBucket pull request 556](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/556)

1. Fix entity tree context menu position
    * [BitBucket pull request 567](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/567)

1. Fix moving static model with link offset
    * [BitBucket pull request 566](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/566)

1. Add support for setting visual transparency through SDF
    * [BitBucket pull request 547](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/547)

1. Add `JointPositionReset` and `JointVelocityReset` components to reset the joint state.
    * [BitBucket pull request 437](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/437)

1. Logging meshes and materials
    * [BitBucket pull request 367](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/367)

1. List plugin env vars
    * [BitBucket pull request 560](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/560)

1. Fix protobuf / clang warnings
    * [BitBucket pull request 555](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/555)

1. Component inspector
    * [BitBucket pull request 528](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/528)

1. Log compress
    * [BitBucket pull request 500](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/500)

1. Set process titles
    * [BitBucket pull request 530](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/530)

1. Add custom user snapping
    * [BitBucket pull request 493](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/493)

1. Add GUI to configure grid
    * [BitBucket pull request 507](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/507)

1. Add multiple entity selection to view angle
    * [BitBucket pull request 531](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/531)

1. Highlight selected entities
    * [BitBucket pull request 515](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/515)

1. Log record overwrite
    * [BitBucket pull request 497](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/497)

1. Add copyright to QML files
    * [BitBucket pull request 527](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/527)

1. Fix shift translation bug
    * [BitBucket pull request 529](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/529)

### Ignition Gazebo 2.15.0 (2020-02-07)

1. Fix seeking back in time in log playback
    * [BitBucket pull request 523](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/523)

1. Fix the deprecated ign-gazebo command line
    * [BitBucket pull request 499](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/499)

1. Always use the latest render texture in scene3d
    * [BitBucket pull request 518](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/518)

1. Remove redundent messages when levels get unloaded
    * [BitBucket pull request 522](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/522)

1. View angle plugin
    * [BitBucket pull request 516](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/516)

1. Support breadcrumb performers
    * [BitBucket pull request 484](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/484)

1. Drag and drop Fuel object into mouse position
    * [BitBucket pull request 511](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/511)

1. Add hotkey keybindings
    * [BitBucket pull request 486](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/486)

### Ignition Gazebo 2.14.0 (2020-01-10)

1. Use Actuator component to communicate between MulticopterVelocityControl and MulticopterMotorModel systems
    * [BitBucket pull request 498](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/498)

1.  Backport fix to insert multiple lights with same name
    * [BitBucket pull request 502](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/502)

1.  Get all component types attached to an entity
    * [BitBucket pull request 494](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/494)

1.  Fix tooltips on entity tree
    * [BitBucket pull request 496](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/496)

### Ignition Gazebo 2.13.0 (2019-12-17)

1. Add Multicopter velocity controller
    * [BitBucket pull request 487](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/487)

1. Fix crash when removing an entity being followed
    * [BitBucket pull request 465](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/465)

1. Add option to right click and remove nodes
    * [BitBucket pull request 458](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/458)

1. Fix jumpy log playback
    * [BitBucket pull request 488](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/488)

1. Remove Scene3d Text anchors
    * [BitBucket pull request 467](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/467)

1. Show grid using SDF file
    * [BitBucket pull request 461](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/461)

### Ignition Gazebo 2.12.0 (2019-11-25)

1. Parse visual cast shadows and add CastShadows component
    * [BitBucket pull request 453](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/453)

1. Update SceneBroadcaster to publish state msg for world with only static models
    * [BitBucket pull request 450](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/450)

1. Add log video recorder
    * [BitBucket pull request 441](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/441)

1. Rechargeable battery model
    * [BitBucket pull request 455](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/455)

1. Add Breadcrumbs system
    * [BitBucket pull request 459](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/459)

1. Drag models from Fuel
    * [BitBucket pull request 454](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/454)

1. Improvements to GUI configuration
    * [BitBucket pull request 451](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/451)

1. Prevent crash when attempting to load more than one render engine per process
    * [BitBucket pull request 463](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/463)

### Ignition Gazebo 2.11.0 (2019-10-23)

1.  Handle Relative URIs
    * [BitBucket pull request 433](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/433)

1.  Avoid using invalid/unsupported joints
    * [BitBucket pull request 438](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/438)

1.  Add mutex to protect views from potential concurrent access
    * [BitBucket pull request 435](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/435)

1.  Add `Link::WorldKineticEnergy` for computing total kinetic energy of a link with respect to the world frame.
    * [BitBucket pull request 434](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/434)

1.  Improve steering behavior of example tracked vehicle
    * [BitBucket pull request 432](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/432)

1.  Rewind / reset and seek
    * [BitBucket pull request 429](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/429)

1.  Add Follow mode to GUI
    * [BitBucket pull request 430](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/430)
    * [BitBucket pull request 436](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/436)

### Ignition Gazebo 2.10.0 (2019-09-08)

1.  Custom odom frequency in sim time
    * [BitBucket pull request 427](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/427)

1.  Add Move To gui plugin
    * [BitBucket pull request 426](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/426)

### Ignition Gazebo 2.9.0

1.  Use the JointSetVelocityCommand feature to set joint velocities
    * [BitBucket pull request 424](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/424)

### Ignition Gazebo 2.8.0 (2019-08-23)

1. Add video recorder gui plugin
    * [BitBucket pull request 422](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/422)

1. Vertical rays for lidar demo
    * [BitBucket pull request 419](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/419)

1. Print world path when using cli
    * [BitBucket pull request 420](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/420)

### Ignition Gazebo 2.7.1

1. Fix order of adding and removing rendering entities, and clean up mesh
   materials in the SceneManager.
    * [BitBucket pull request 415](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/415)
    * [BitBucket pull request 416](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/416)

### Ignition Gazebo 2.7.0

1. Move creation of default log path to ServerConfig. This lets both console logs and state logs to be stored in the same directory.  The console messages are always logged.  Allow state log files to be overwritten.
    * [BitBucket pull request 413](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/413)

1. Baseline for stereo cameras
    * [BitBucket pull request 406](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/406)

1. Fix log playback with levels. This drops support for logs created before v2.0.0.
    * [BitBucket pull request 407](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/407)

1. Add worker threads for System PostUpdate phase
    * [BitBucket pull request 387](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/387)

1. Added a test runner for executing an SDF and recording simulation rates.
   See the `test/performance/READEM.md` file for more info.
    * [BitBucket pull request 389](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/389)

### Ignition Gazebo 2.6.1 (2019-07-26)

1. Clear stepMsg before populating it
    * [BitBucket pull request 398](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/398)

### Ignition Gazebo 2.6.0 (2019-07-24)

1.  Improve performance of Pose Publisher
    * [BitBucket pull request 392](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/392)

1. Fix distributed sim
    * [BitBucket pull request 385](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/385)

### Ignition Gazebo 2.5.0 (2019-07-19)

1. The LinearBatteryPlugin system publishes battery state
    * [BitBucket pull request 388](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/388)

### Ignition Gazebo 2.4.0 (2019-07-17)

1. Bundle scene updates in sensor system
    * [BitBucket pull request 386](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/386)

### Ignition Gazebo 2.3.0 (2019-07-13)

1. Improve physics system peformance by skipping static model updates.
   Components state information has been incorporated, which is used to
   indicate if a component change is periodic (such as through a physics
   update) or a one-time change (such as through a user command).
    * [BitBucket pull request 384](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/384)

1. Add sdf parameter to battery to start draining only when robot has started moving
    * [BitBucket pull request 370](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/370)

1. Improve SceneBroadcaster peformance by 1) Limit message generation if
   subscribers to pose topics are not present, 2) Set world stats message
   instead of copying the message, 3) Suppress scenegraph updates when there
   are no new entities, 4) Make better use of const functions, 5) Prevent
   creation of msgs::SerializedStep every PostUpdate, 6) Only serialized and
   transmit components that have changed.
    * [BitBucket pull request 371](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/371)
    * [BitBucket pull request 372](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/372)
    * [BitBucket pull request 373](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/373)
    * [BitBucket pull request 374](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/374)
    * [BitBucket pull request 375](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/375)
    * [BitBucket pull request 376](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/376)

### Ignition Gazebo 2.2.0

1. The DiffDrive system publishes odometry information.
    * [BitBucket pull request 368](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/368)

1. Allow attaching plugins to sensors from a server config.
    * [BitBucket pull request 366](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/366)

1. Remove world name from frame_ids
    * [BitBucket pull request 364](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/364)

1. Fix deadlock when spawning robots
    * [BitBucket pull request 365](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/365)

1. Set default topics for rendering sensors
    * [BitBucket pull request 363](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/363)

1. Support custom random seed from the command line.
    * [BitBucket pull request 362](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/362)

### Ignition Gazebo 2.1.0

1. RenderUtil fix bad merge: check for existing entities in GzScene3D on initialization.
    * [BitBucket pull request 360](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/360)

1. Allow sensors to load plugins.
    * [BitBucket pull request 356](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/356)
    * [BitBucket pull request 366](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/366)

1. Parse and load submesh geometry in visuals.
    * [BitBucket pull request 353](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/353)

1. Allow setting the update frequency of pose publisher.
    * [BitBucket pull request 352](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/352)

1. Added RGBD camera sensor.
    * [BitBucket pull request 351](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/351)

1. Fix Docker scripts.
    * [BitBucket pull request 347](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/347)

1. Support log playback from a different path
    * [BitBucket pull request 355](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/355)

### Ignition Gazebo 2.0.0

1. RenderUtil: check for existing entities in GzScene3D on initialization.
    * [BitBucket pull request 350](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/350)

1. SceneBroadcaster: only send pose state periodically.
    * [BitBucket pull request 345](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/345)

1. PeerTracker: increase distributed simulation peer tracking timeout.
    * [BitBucket pull request 344](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/344)

1. MultiCopterMotorModel: add mutex to protect motor velocity command.
    * [BitBucket pull request 341](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/341)

1. Tweaks to example worlds
    * [BitBucket pull request 342](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/342)

1. DiffDrive system: add topic as system parameter.
    * [BitBucket pull request 343](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/343)

1. Log entity creation and deletion
    * [BitBucket pull request 337](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/337)

1. Multicopter motor model
    * [BitBucket pull request 322](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/322)

1. Fix removing selected entity
    * [BitBucket pull request 339](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/339)

1. Collision serialization
    * [BitBucket pull request 326](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/326)

1. Add support for moving and rotating models
    * [BitBucket pull request 316](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/316)

1. Pose commands
    * [BitBucket pull request 334](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/334)

1. Level performers can be added at runtime using a service call. See the
   levels tutorial for more information.
    * [BitBucket pull request 264](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/264)

1. Update worlds to GzScene3D
    * [BitBucket pull request 333](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/333)

1. Reduce logging file size
    * [BitBucket pull request 332](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/332)

1. Update PosePublisher system to publish sensor poses and to use scoped names for frame ids
    * [BitBucket pull request 331](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/331)

1. Fix gui plugin linking issue
    * [BitBucket pull request 327](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/327)
    * [BitBucket pull request 330](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/330)

1. Toolbar colors
    * [BitBucket pull request 329](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/329)

1. Rename Scene3D gui plugin to GzScene3D
    * [BitBucket pull request 328](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/328)

1. Fix distributed sim documentation
    * [BitBucket pull request 318](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/318)

1. Port Scene3D gui plugin from ign-gui. Renamed to GzScene3D.
    * [BitBucket pull request 315](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/315)

1. Entity tree UI
    * [BitBucket pull request 285](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/285)

1. Add rendering component
    * [BitBucket pull request 306](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/306)

1. Update Camera and DepthCamera components to use sdf::Sensor object instead of an sdf::ElementPtr.
    * [BitBucket pull request 299](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/299)

1. Added system for ignition::sensors::AirPressureSensor.
    * [BitBucket pull request 300](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/300)

1. Support conversion and serialization of Imu components. IMU sensors are
   loaded from an SDF DOM object.
    * [BitBucket pull request 302](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/302)

1. Throttle sensors update rate
    * [BitBucket pull request 323](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/323)

1. Fix changing themes
    * [BitBucket pull request 321](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/321)

1. Battery tweaks
    * [BitBucket pull request 314](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/314)

1. Support conversion and serialization of PBR parameters in a material component
    * [BitBucket pull request 304](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/304)

1. Joint state pub
    * [BitBucket pull request 260](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/260)

1. Update Altimeter component to use sdf::Sensor object instead of an
   sdf::ElementPtr.
    * [BitBucket pull request 286](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/286)

1. Update docker nightly dependencies
    * [BitBucket pull request 310](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/310)

1. Ign tool
    * [BitBucket pull request 296](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/296)
    * [BitBucket pull request 336](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/336)

1. State broadcast
    * [BitBucket pull request 307](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/307)

1. Use world statistics message on network
    * [BitBucket pull request 305](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/305)

1. Update Magnetometer component to use sdf::Sensor object instead of an sdf::ElementPtr.
    * [BitBucket pull request 272](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/272)

1. Fix Scene3D loading empty world
    * [BitBucket pull request 308](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/308)

1. Support conversion and serialization of scene and light components
    * [BitBucket pull request 297](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/297)

1. Operators instead of De/Serialize
    * [BitBucket pull request 293](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/293)

1. Remove PIMPL from Component
    * [BitBucket pull request 267](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/267)

1. Delay scene broadcaster transport setup
    * [BitBucket pull request 292](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/292)

1. Report link poses from secondaries during distributed simulation, using a cache
    * [BitBucket pull request 276](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/276)
    * [BitBucket pull request 265](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/265)

1. Restore log playback
    * [BitBucket pull request 288](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/288)

1. ECM changed state
    * [BitBucket pull request 287](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/287)

1. Joint serialization
    * [BitBucket pull request 281](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/281)

1. Use scene ambient and background color information in sensor
   configuration.
    * [BitBucket pull request 268](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/268)

1. Performance benchmarking
    * [BitBucket pull request 220](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/220)
    * [BitBucket pull request 253](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/253)
    * [BitBucket pull request 258](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/258)
    * [BitBucket pull request 283](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/283)
    * [BitBucket pull request 312](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/312)

1. Remove emissive component from visual materials
    * [BitBucket pull request 271](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/271)

1. Serialization for more components
    * [BitBucket pull request 255](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/255)

1. Added an SDF message to the start of log files.
    * [BitBucket pull request 257](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/257)

1. Unify network and sync managers
    * [BitBucket pull request 261](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/261)

1. Add PerformerLevels component
    * [BitBucket pull request 262](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/262)

1. Distributed sim deprecate envs
    * [BitBucket pull request 240](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/240)

1. Use ign-sensors magnetometer sensor plugin
    * [BitBucket pull request 221](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/221)

1. Use ign-sensors altimeter sensor plugin
    * [BitBucket pull request 215](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/215)

1. Use ign-sensors imu sensor plugin
    * [BitBucket pull request 219](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/219)

1. Depend on ign-sensors rendering component
    * [BitBucket pull request 212](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/212)

## Ignition Gazebo 1.x

### Ignition Gazebo 1.X.X

1. Add Wind Plugin (Ported from Gazebo classic)
    * [BitBucket pull request 273](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/273/)

1. Port battery plugin from Gazebo classic
    * [BitBucket pull request 234](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/234)
    * [BitBucket pull request 317](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/317)
    * [BitBucket pull request 324](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/324)

1. Use ISO timestamp for default log path
    * [BitBucket pull request 289](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/289)

1. Logging tutorial
    * [BitBucket pull request 280](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/280)

1. Joystick SDF small typos
    * [BitBucket pull request 284](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/284)

1. Add `Link`: a convenience class for interfacing with link entities
    * [BitBucket pull request 269](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/269)

1. Added LiftDragPlugin (ported from Gazebo classic)
    * [BitBucket pull request 256](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/256)

1. Logging refactor unique path functions to ign-common
    * [BitBucket pull request 270](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/270)

1. Added test for log record and playback.
    * [BitBucket pull request 263](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/263)

1. Add ApplyJointForce system
    * [BitBucket pull request 254](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/254)

1. More ign-msgs <-> SDF conversions: Inertial, Geometry, Material
    * [BitBucket pull request 251](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/251)

1. Logging command line support
    * [BitBucket pull request 249](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/249)

1. Remove inactive performers instead of setting static
    * [BitBucket pull request 247](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/247)

1. Use state instead of pose in distributed simulation
    * [BitBucket pull request 242](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/242)

1. Distributed implies levels
    * [BitBucket pull request 243](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/243)

1. Add a basic JointController system
    * [BitBucket pull request 246](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/246)

1. Enforce component type uniqueness
    * [BitBucket pull request 236](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/236)

1. Clean CI: disable test known to fail on OSX
    * [BitBucket pull request 244](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/244)

1. Logical camera topic name check
    * [BitBucket pull request 245](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/245)

1. Added command line options to configure distributed simulation. These
   will replace the environment variables.
    * [BitBucket pull request 238](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/238)

1. Add systems to queue before actually adding them to runner
    * [BitBucket pull request 241](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/241)

1. Added a docker image that uses the ignition meta package
    * [BitBucket pull request 237](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/237)

1. Move some design docs to tutorials
    * [BitBucket pull request 230](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/230)

1. Disable GUI when using distributed simulation
    * [BitBucket pull request 235](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/235)

1. Bring component type names back
    * [BitBucket pull request 232](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/232)

1. A few tweaks to logging
    * [BitBucket pull request 228](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/228)

1. Handle friction coefficients
    * [BitBucket pull request 227](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/227)

1. Change private msgs namespace
    * [BitBucket pull request 233](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/233)

1. Set tutorial titles
    * [BitBucket pull request 231](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/231)

1. Example tunnel world
    * [BitBucket pull request 205](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/205)

1. Conversion from chrono to ign-msgs
    * [BitBucket pull request 223](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/223)

1. Prevent error message when using levels
    * [BitBucket pull request 229](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/229)

### Ignition Gazebo 1.1.0 (2019-03-15)

1. Distributed performers running in lockstep
    * [BitBucket pull request 186](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/186)
    * [BitBucket pull request 201](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/201)
    * [BitBucket pull request 209](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/209)
    * [BitBucket pull request 213](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/213)

1. Fix documentation tagfiles
    * [BitBucket pull request 214](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/214)

1. Convert gui library into a component
    * [BitBucket pull request 206](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/206)

1. include <cstdint> wherever special int types like uint64_t are used
    * [BitBucket pull request 208](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/208)

1. Move network internal
    * [BitBucket pull request 211](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/211)

1. Logging / playback
    * [BitBucket pull request 181](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/181)

1. ECM state streaming
    * [BitBucket pull request 184](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/184)

1. Unversioned system libraries
    * [BitBucket pull request 222](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/222)

### Ignition Gazebo 1.0.2 (2019-03-12)

1. Use TARGET_SO_NAME to fix finding dartsim plugin
    * [BitBucket pull request 217](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/217)

### Ignition Gazebo 1.0.1 (2019-03-01)

1. Update gazebo version number in sdf files
    * [BitBucket pull request 207](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/207)

### Ignition Gazebo 1.0.0 (2019-03-01)

1. Initial release

## Ignition Gazebo 0.x

### Ignition Gazebo 0.1.0

1. Add support for joints
    * [BitBucket pull request 77](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/77)

1. Use SimpleWrapper for more component types
    * [BitBucket pull request 78](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/78)

1. Create EventManager and delegate System instantiation to SimulationRunner
    * [BitBucket pull request 79](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/79)

1. Integrate ign-gui
    * [BitBucket pull request 11](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/11)

1. Remove some build dependencies.
    * [BitBucket pull request 6](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/6)

1. Added basic Entity class.
    * [BitBucket pull request 3](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/3)

1. Added a basic System class.
    * [BitBucket pull request 4](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/4)
