## Ignition Gazebo 6.x

### Gazebo Sim 6.12.0 (2022-08-30)

1. Add topic parameter to thrust plugin
    * [Pull request #1681](https://github.com/gazebosim/gz-sim/pull/1681)

1. Add information about `<topic>` system parameter
    * [Pull request #1671](https://github.com/gazebosim/gz-sim/pull/1671)

1. Adding tests for hydrodynamics
    * [Pull request #1617](https://github.com/gazebosim/gz-sim/pull/1617)

1. Fix Windows and Doxygen
    * [Pull request #1643](https://github.com/gazebosim/gz-sim/pull/1643)

### Gazebo Sim 6.11.0 (2022-08-17)

1. Add support for specifying log record period
    * [Pull request #1636](https://github.com/gazebosim/gz-sim/pull/1636)

1. Common widget GzColor replacement
    * [Pull request #1530](https://github.com/gazebosim/gz-sim/pull/1530)

1. Replace plotIcon in ComponentInspector with GzPlotIcon
    * [Pull request #1638](https://github.com/gazebosim/gz-sim/pull/1638)

1. Component Inspector with common widget pose plotting
    * [Pull request #1607](https://github.com/gazebosim/gz-sim/pull/1607)

1. Change CODEOWNERS and maintainer to Michael
    * [Pull request #1644](https://github.com/gazebosim/gz-sim/pull/1644)

1. Replace pose in ViewAngle with GzPose
    * [Pull request #1641](https://github.com/gazebosim/gz-sim/pull/1641)

1. Add system to an entity through Component Inspector
    * [Pull request #1549](https://github.com/gazebosim/gz-sim/pull/1549)

1. Quick start dialog
    * [Pull request #1536](https://github.com/gazebosim/gz-sim/pull/1536)
    * [Pull request #1627](https://github.com/gazebosim/gz-sim/pull/1627)

1. Quiet libSDFormat console on --verbose 0
    * [Pull request #1621](https://github.com/gazebosim/gz-sim/pull/1621)

1. New Apply Link Wrench system
    * [Pull request #1593](https://github.com/gazebosim/gz-sim/pull/1593)

1. Add Tf publishing to AckermannSteering system
    * [Pull request #1576](https://github.com/gazebosim/gz-sim/pull/1576)

1. Fix component updates
    * [Pull request #1580](https://github.com/gazebosim/gz-sim/pull/1580)

1. Implement vector3 with common widget vector3
    * [Pull request #1569](https://github.com/gazebosim/gz-sim/pull/1569)

1. Fix to modelphotoshoot test
    * [Pull request #1570](https://github.com/gazebosim/gz-sim/pull/1570)

1. Update log playback gui config
    * [Pull request #1590](https://github.com/gazebosim/gz-sim/pull/1590)

1. Helper function to get an entity from an entity message
    * [Pull request #1595](https://github.com/gazebosim/gz-sim/pull/1595)

1. Fix compilation of scene broadcaster test
    * [Pull request #1599](https://github.com/gazebosim/gz-sim/pull/1599)

1. Ignition -> Gazebo
    * [Pull request #1596](https://github.com/gazebosim/gz-sim/pull/1596)

1. Add Model::CanonicalLink getter
    * [Pull request #1594](https://github.com/gazebosim/gz-sim/pull/1594)

1. Implement Pose3d with common widget pose
    * [Pull request #1571](https://github.com/gazebosim/gz-sim/pull/1571)

1. Fix UNIT_Server_TEST on Windows
    * [Pull request #1577](https://github.com/gazebosim/gz-sim/pull/1577)

1. Use pytest to generate junit xml files for python tests
    * [Pull request #1562](https://github.com/gazebosim/gz-sim/pull/1562)

1. Refactor: Utilizes function to load animations
    * [Pull request #1568](https://github.com/gazebosim/gz-sim/pull/1568)

1. Utilizes function to sequence trajectories
    * [Pull request #1565](https://github.com/gazebosim/gz-sim/pull/1565)

1. Disable MacOS flakies Citadel
    * [Pull request #1545](https://github.com/gazebosim/gz-sim/pull/1545)

### Gazebo Sim 6.10.0 (2022-06-24)

1. Expose the ability to stop a server from C++
    * [Pull request #1551](https://github.com/gazebosim/gz-sim/pull/1551)

1. Fix various Protobuf Windows warnings
    * [Pull request #1299](https://github.com/gazebosim/gz-sim/pull/1299)

1. New service for adding systems to an entity
    * [Pull request #1524](https://github.com/gazebosim/gz-sim/pull/1524)

1. Added particle emitters to scene broadcaster
    * [Pull request #1516](https://github.com/gazebosim/gz-sim/pull/1516)

1. Use more `sdf::Plugin` instead of `sdf::ElementPtr`
    * [Pull request #1352](https://github.com/gazebosim/gz-sim/pull/1352)

1. Depend on common 4.5.1
    * [Pull request #1547](https://github.com/gazebosim/gz-sim/pull/1547)

1. Update README links
    * [Pull request #1546](https://github.com/gazebosim/gz-sim/pull/1546)

1. Add bounding boxes into the label system plugin
    * [Pull request #1040](https://github.com/gazebosim/gz-sim/pull/1040)

1. Odometry publisher: also publish `Pose_V` (TF)
    * [Pull request #1534](https://github.com/gazebosim/gz-sim/pull/1534)

1. Fix clang warning from Thruster plugin
    * [Pull request #1540](https://github.com/gazebosim/gz-sim/pull/1540)

1. Fix locks in Visualize Lidar GUI plugin
    * [Pull request #1538](https://github.com/gazebosim/gz-sim/pull/1538)

1. Bash completion for flags
    * [Pull request #1504](https://github.com/gazebosim/gz-sim/pull/1504)

1. Fix sensors battery state test
    * [Pull request #1529](https://github.com/gazebosim/gz-sim/pull/1529)

1. Add new `GZ_GUI_RESOURCE_PATH` to help message
    * [Pull request #1470](https://github.com/gazebosim/gz-sim/pull/1470)

1. Fix regression with camera sensors not using the background color set in `<scene>`
    * [Pull request #1515](https://github.com/gazebosim/gz-sim/pull/1515)

1. Check RGBD camera sensor connection
    * [Pull request #1513](https://github.com/gazebosim/gz-sim/pull/1513)

1. Optimize sensor updates
    * [Pull request #1480](https://github.com/gazebosim/gz-sim/pull/1480)

1. System inspector GUI widget
    * [Pull request #1404](https://github.com/gazebosim/gz-sim/pull/1404)

1. Scene update resource finder
    * [Pull request #1508](https://github.com/gazebosim/gz-sim/pull/1508)

1. Updating hydrodynamics plugin description
    * [Pull request #1502](https://github.com/gazebosim/gz-sim/pull/1502)

1. Makes thruster stop when battery runs out.
    * [Pull request #1495](https://github.com/gazebosim/gz-sim/pull/1495)

1. Fix Documentation Header.
    * [Pull request #1501](https://github.com/gazebosim/gz-sim/pull/1501)

1. Adding rssi
    * [Pull request #1482](https://github.com/gazebosim/gz-sim/pull/1482)

1. Delete unused gazebo.hh.in
    * [Pull request #1490](https://github.com/gazebosim/gz-sim/pull/1490)

1. :books: Fixed broken URL link to gazebo documentation
    * [Pull request #1486](https://github.com/gazebosim/gz-sim/pull/1486)

1. View polyline collisions on the GUI
    * [Pull request #1481](https://github.com/gazebosim/gz-sim/pull/1481)

1. Extruded 2D polyline geometries
    * [Pull request #1456](https://github.com/gazebosim/gz-sim/pull/1456)

1. Fix fuel url
    * [Pull request #1479](https://github.com/gazebosim/gz-sim/pull/1479)

1. Camera trigger integration test
    * [Pull request #1384](https://github.com/gazebosim/gz-sim/pull/1384)

1. Extend Multicoptor Control system to include nested model inertial params
    * [Pull request #1450](https://github.com/gazebosim/gz-sim/pull/1450)

1. Remove dead ign.cc file
    * [Pull request #1474](https://github.com/gazebosim/gz-sim/pull/1474)

1. Test case to check if velocity limits are applied to joints
    * [Pull request #1445](https://github.com/gazebosim/gz-sim/pull/1445)

1. SceneBroadcaster: Use double for state publish frequency instead of int
    * [Pull request #1417](https://github.com/gazebosim/gz-sim/pull/1417)

1. Revert format change
    * [Pull request #1468](https://github.com/gazebosim/gz-sim/pull/1468)

1. Fix finding DART on macOS
    * [Pull request #1469](https://github.com/gazebosim/gz-sim/pull/1469)

1. Skip serializing nested model with `//pose/@relative_to` attribute
    * [Pull request #1454](https://github.com/gazebosim/gz-sim/pull/1454)

1. Fix running simulation with no world specified on the command line
    * [Pull request #1463](https://github.com/gazebosim/gz-sim/pull/1463)

1. Add repo specific issue templates
    * [Pull request #1461](https://github.com/gazebosim/gz-sim/pull/1461)

1. python: release GIL when running server
    * [Pull request #1458](https://github.com/gazebosim/gz-sim/pull/1458)

1. python: remove semicolons
    * [Pull request #1459](https://github.com/gazebosim/gz-sim/pull/1459)

1. Bump rendering dependency version
    * [Pull request #1455](https://github.com/gazebosim/gz-sim/pull/1455)

1. Improve contact sensor / visualization performance
    * [Pull request #1452](https://github.com/gazebosim/gz-sim/pull/1452)

1. Set simulation time to Rendering
    * [Pull request #1415](https://github.com/gazebosim/gz-sim/pull/1415)

### Ignition Gazebo 6.9.0 (2022-04-14)

1. Add new `RFComms` system
    * [Pull request #1428](https://github.com/ignitionrobotics/ign-gazebo/pull/1428)

1. Add comms infrastructure
    * [Pull request #1416](https://github.com/ignitionrobotics/ign-gazebo/pull/1416)

1. Fix CMake version examples and bump plugin version
    * [Pull request #1442](https://github.com/ignitionrobotics/ign-gazebo/pull/1442)

1. Make sure pose publisher creates valid pose topics
    * [Pull request #1433](https://github.com/ignitionrobotics/ign-gazebo/pull/1433)

1. Add Ubuntu Jammy CI
    * [Pull request #1418](https://github.com/ignitionrobotics/ign-gazebo/pull/1418)

1. Removed `screenToPlane` method and use `rendering::screenToPlane`
    * [Pull request #1432](https://github.com/ignitionrobotics/ign-gazebo/pull/1432)

1. Supply world frame orientation and heading to IMU sensor (#1427)
    * [Pull request #1427](https://github.com/ignitionrobotics/ign-gazebo/pull/1427)

1. Add desktop entry and SVG logo
    * [Pull request #1411](https://github.com/ignitionrobotics/ign-gazebo/pull/1411)
    * [Pull request #1430](https://github.com/ignitionrobotics/ign-gazebo/pull/1430)

1. Fix segfault at exit
    * [Pull request #1317](https://github.com/ignitionrobotics/ign-gazebo/pull/1317)

1. Add Gaussian noise to Odometry Publisher
    * [Pull request #1393](https://github.com/ignitionrobotics/ign-gazebo/pull/1393)

### Ignition Gazebo 6.8.0 (2022-04-04)

1. ServerConfig accepts an sdf::Root DOM object
    * [Pull request #1333](https://github.com/ignitionrobotics/ign-gazebo/pull/1333)

1. Disable sensors in sensors system when battery is drained
    * [Pull request #1385](https://github.com/ignitionrobotics/ign-gazebo/pull/1385)

1. Referring to Fuel assets within a heightmap
    * [Pull request #1419](https://github.com/ignitionrobotics/ign-gazebo/pull/1419)

1. Add the Model Photo Shoot system, port of Modelpropshop plugin from Gazebo classic
    * [Pull request #1331](https://github.com/ignitionrobotics/ign-gazebo/pull/1331)

1. Distortion camera integration test
    * [Pull request #1374](https://github.com/ignitionrobotics/ign-gazebo/pull/1374)

1. Add wheel slip user command
    * [Pull request #1241](https://github.com/ignitionrobotics/ign-gazebo/pull/1241)

1. SceneBroadcaster: only send changed state information for change events
    * [Pull request #1392](https://github.com/ignitionrobotics/ign-gazebo/pull/1392)

1. Fortress: Install Ogre 2.2, simplify docker
    * [Pull request #1395](https://github.com/ignitionrobotics/ign-gazebo/pull/1395)

1. Disable tests that are expected to fail on Windows
    * [Pull request #1408](https://github.com/ignitionrobotics/ign-gazebo/pull/1408)

1. Added user command to set multiple entities
    * [Pull request #1394](https://github.com/ignitionrobotics/ign-gazebo/pull/1394)

1. Fix JointStatePublisher topic name for nested models
    * [Pull request #1405](https://github.com/ignitionrobotics/ign-gazebo/pull/1405)

1. add initial_position param to joint controller system
    * [Pull request #1406](https://github.com/ignitionrobotics/ign-gazebo/pull/1406)

1. Component inspector: refactor Pose3d C++ code into a separate class
    * [Pull request #1400](https://github.com/ignitionrobotics/ign-gazebo/pull/1400)

1. Prevent hanging when world has only non-world plugins
    * [Pull request #1383](https://github.com/ignitionrobotics/ign-gazebo/pull/1383)

1. Toggle Light visuals
    * [Pull request #1387](https://github.com/ignitionrobotics/ign-gazebo/pull/1387)

1. Disable PeerTracker.PeerTrackerStale on macOS
    * [Pull request #1398](https://github.com/ignitionrobotics/ign-gazebo/pull/1398)

1. Disable ModelCommandAPI_TEST.RgbdCameraSensor on macOS
    * [Pull request #1397](https://github.com/ignitionrobotics/ign-gazebo/pull/1397)

1. Don't mark entities with a ComponentState::NoChange component as modified
    * [Pull request #1391](https://github.com/ignitionrobotics/ign-gazebo/pull/1391)

1. Add gazebo Entity id to rendering sensor's user data
    * [Pull request #1381](https://github.com/ignitionrobotics/ign-gazebo/pull/1381)

1. Allow to turn on/off lights
    * [Pull request #1343](https://github.com/ignitionrobotics/ign-gazebo/pull/1343)

1. Added headless rendering tutorial
    * [Pull request #1386](https://github.com/ignitionrobotics/ign-gazebo/pull/1386)

1. Add xyz and rpy offset to published odometry pose
    * [Pull request #1341](https://github.com/ignitionrobotics/ign-gazebo/pull/1341)

1. Fix visualization python tutorial
    * [Pull request #1377](https://github.com/ignitionrobotics/ign-gazebo/pull/1377)

1. Populate GUI plugins that are empty
    * [Pull request #1375](https://github.com/ignitionrobotics/ign-gazebo/pull/1375)

### Ignition Gazebo 6.7.0 (2022-02-24)

1. Added Python interfaces to some Ignition Gazebo methods
    * [Pull request #1219](https://github.com/ignitionrobotics/ign-gazebo/pull/1219)

1. Use pose multiplication instead of addition
    * [Pull request #1369](https://github.com/ignitionrobotics/ign-gazebo/pull/1369)

1. Disables Failing Buoyancy Tests on Win32
    * [Pull request #1368](https://github.com/ignitionrobotics/ign-gazebo/pull/1368)

1. Extend ShaderParam system to support loading different shader languages
    * [Pull request #1335](https://github.com/ignitionrobotics/ign-gazebo/pull/1335)

1. Populate names of colliding entities in contact points message
    * [Pull request #1351](https://github.com/ignitionrobotics/ign-gazebo/pull/1351)

1. Refactor System functionality into SystemManager
    * [Pull request #1340](https://github.com/ignitionrobotics/ign-gazebo/pull/1340)

1. GzSceneManager: Prevent crash boom when inserted from menu
    * [Pull request #1371](https://github.com/ignitionrobotics/ign-gazebo/pull/1371)

### Ignition Gazebo 6.6.0 (2022-02-24)

1. Fix accessing empty JointPosition component in lift drag plugin
    * [Pull request #1366](https://github.com/ignitionrobotics/ign-gazebo/pull/1366)

1. Add parameter to TrajectoryFollower stop rotation when bearing is reached
    * [Pull request #1349](https://github.com/ignitionrobotics/ign-gazebo/pull/1349)

1. Support disabling pose publisher from publishing top level model pose
    * [Pull request #1342](https://github.com/ignitionrobotics/ign-gazebo/pull/1342)

1. Added more sensor properties to scene/info topic
    * [Pull request #1344](https://github.com/ignitionrobotics/ign-gazebo/pull/1344)

1. Adding ability to pause/resume the trajectory follower behavior.
    * [Pull request #1347](https://github.com/ignitionrobotics/ign-gazebo/pull/1347)

1. Logs a warning if a mode is not clearly sepecified.
    * [Pull request #1307](https://github.com/ignitionrobotics/ign-gazebo/pull/1307)

1. JointStatePublisher publish parent, child and axis data
    * [Pull request #1345](https://github.com/ignitionrobotics/ign-gazebo/pull/1345)

1. Fixed light gui component inspector
    * [Pull request #1337](https://github.com/ignitionrobotics/ign-gazebo/pull/1337)

1. Fix UNIT_SdfGenerator_TEST
    * [Pull request #1319](https://github.com/ignitionrobotics/ign-gazebo/pull/1319)

1. Add elevator system
    * [Pull request #535](https://github.com/ignitionrobotics/ign-gazebo/pull/535)

1. Removed unused variables in shapes plugin
    * [Pull request #1321](https://github.com/ignitionrobotics/ign-gazebo/pull/1321)

### Ignition Gazebo 6.5.0 (2022-02-15)

1. New trajectory follower system
    * [Pull request #1332](https://github.com/ignitionrobotics/ign-gazebo/pull/1332)

1. Extend ShaderParam system to support textures
    * [Pull request #1310](https://github.com/ignitionrobotics/ign-gazebo/pull/1310)

1. Adds a `Link::SetLinearVelocity()` method
    * [Pull request #1323](https://github.com/ignitionrobotics/ign-gazebo/pull/1323)

1. Fix weird indentation in `Link.hh`
    * [Pull request #1324](https://github.com/ignitionrobotics/ign-gazebo/pull/1324)

1. Limit thruster system's input thrust cmd
    * [Pull request #1318](https://github.com/ignitionrobotics/ign-gazebo/pull/1318)

1. Load and run visual plugin (system) on GUI side
    * [Pull request #1275](https://github.com/ignitionrobotics/ign-gazebo/pull/1275)

1. Log an error if JointPositionController cannot find the joint. (citadel retarget)
    * [Pull request #1314](https://github.com/ignitionrobotics/ign-gazebo/pull/1314)

1. Update source install instructions
    * [Pull request #1311](https://github.com/ignitionrobotics/ign-gazebo/pull/1311)

1. Document the `<topic>` option for JointPositionController.
    * [Pull request #1309](https://github.com/ignitionrobotics/ign-gazebo/pull/1309)

1. Fix typo in EntityComponentManager
    * [Pull request #1304](https://github.com/ignitionrobotics/ign-gazebo/pull/1304)

1. Buoyancy: fix center of volume's reference frame
    * [Pull request #1302](https://github.com/ignitionrobotics/ign-gazebo/pull/1302)

1. Fix graded buoyancy problems
    * [Pull request #1297](https://github.com/ignitionrobotics/ign-gazebo/pull/1297)

1. Add surface to buoyancy engine. (retarget fortress)
    * [Pull request #1298](https://github.com/ignitionrobotics/ign-gazebo/pull/1298)

1. Remove EachNew calls from sensor PreUpdates
    * [Pull request #1281](https://github.com/ignitionrobotics/ign-gazebo/pull/1281)

1. Prevent GzScene3D 💥 if another scene is already loaded
    * [Pull request #1294](https://github.com/ignitionrobotics/ign-gazebo/pull/1294)

1. Fix various typos on API documentation
    * [Pull request #1291](https://github.com/ignitionrobotics/ign-gazebo/pull/1291)

1. Optional orientation when spawning entity using spherical coordinates
    * [Pull request #1263](https://github.com/ignitionrobotics/ign-gazebo/pull/1263)

1. Cleanup update call for non-rendering sensors
    * [Pull request #1282](https://github.com/ignitionrobotics/ign-gazebo/pull/1282)

1. Documentation Error
    * [Pull request #1285](https://github.com/ignitionrobotics/ign-gazebo/pull/1285)

1. Min and max parameters for velocity, acceleration, and jerk apply to linear and angular separately.
    * [Pull request #1229](https://github.com/ignitionrobotics/ign-gazebo/pull/1229)

1. Add project() call to examples
    * [Pull request #1274](https://github.com/ignitionrobotics/ign-gazebo/pull/1274)

1. Implement /server_control::stop
    * [Pull request #1240](https://github.com/ignitionrobotics/ign-gazebo/pull/1240)

### Ignition Gazebo 6.4.0 (2021-01-13)

1. Disable more tests on Windows
    * [Pull request #1286](https://github.com/ignitionrobotics/ign-gazebo/pull/1286)

1. Adding angular acceleration to the Link class
    * [Pull request #1288](https://github.com/ignitionrobotics/ign-gazebo/pull/1288)

1. Add world force
    * [Pull request #1279](https://github.com/ignitionrobotics/ign-gazebo/pull/1279)

1. Add NavSat sensor (GPS)
    * [Pull request #1248](https://github.com/ignitionrobotics/ign-gazebo/pull/1248)

1. Light Commands via topic
    * [Pull request #1222](https://github.com/ignitionrobotics/ign-gazebo/pull/1222)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/ignitionrobotics/ign-gazebo/pull/1255)

1. Add visibility to ModelEditorAddEntity to fix Windows
    * [Pull request #1246](https://github.com/ignitionrobotics/ign-gazebo/pull/1246)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/ignitionrobotics/ign-gazebo/pull/1194)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/ignitionrobotics/ign-gazebo/pull/1224)

1. Disable user commands light test on macOS
    * [Pull request #1204](https://github.com/ignitionrobotics/ign-gazebo/pull/1204)

1. Skip failing Windows tests
    * [Pull request #1205](https://github.com/ignitionrobotics/ign-gazebo/pull/1205)


### Ignition Gazebo 6.3.0 (2021-12-10)

1. View entity frames from the GUI
    * [Pull request #1105](https://github.com/ignitionrobotics/ign-gazebo/pull/1105)

1. Model editor
    * [Pull request #1231](https://github.com/ignitionrobotics/ign-gazebo/pull/1231)

1. Send state message when components are removed
    * [Pull request #1235](https://github.com/ignitionrobotics/ign-gazebo/pull/1235)

1. Docker fixes for Fortress
    * [Pull request #1238](https://github.com/ignitionrobotics/ign-gazebo/pull/1238)

1. Added sensor plugin to be able to visualize camera in `plane_propeller_demo.sdf`
    * [Pull request #1226](https://github.com/ignitionrobotics/ign-gazebo/pull/1226)

1. Update SdfGenerator to save link and sensor data to file
    * [Pull request #1201](https://github.com/ignitionrobotics/ign-gazebo/pull/1201)

1. Fix buoyancy not being applied for one iteration
    * [Pull request #1211](https://github.com/ignitionrobotics/ign-gazebo/pull/1211)

1. Increase maximum values in ViewAngle widget and increase its size
    * [Pull request #1221](https://github.com/ignitionrobotics/ign-gazebo/pull/1221)
    * [Pull request #1239](https://github.com/ignitionrobotics/ign-gazebo/pull/1239)

1. Fix the force-torque sensor update rate
    * [Pull request #1159](https://github.com/ignitionrobotics/ign-gazebo/pull/1159)

### Ignition Gazebo 6.2.0 (2021-11-16)

1. Configurable joint state publisher's topic
    * [Pull request #1076](https://github.com/ignitionrobotics/ign-gazebo/pull/1076)

1. Thruster plugin: add tests and velocity control
    * [Pull request #1190](https://github.com/ignitionrobotics/ign-gazebo/pull/1190)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/ignitionrobotics/ign-gazebo/pull/1192)

1. Add `sdfString` to `ServerConfig`'s copy constructor.
    * [Pull request #1185](https://github.com/ignitionrobotics/ign-gazebo/pull/1185)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/ignitionrobotics/ign-gazebo/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/ignitionrobotics/ign-gazebo/pull/847)

1. Remove bounding box when entities are removed
    * [Pull request #1053](https://github.com/ignitionrobotics/ign-gazebo/pull/1053)
    * [Pull request #1213](https://github.com/ignitionrobotics/ign-gazebo/pull/1213)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/ignitionrobotics/ign-gazebo/pull/1181)

1.  Extend odom publisher to allow 3D
    * [Pull request #1180](https://github.com/ignitionrobotics/ign-gazebo/pull/1180)

1. Support copy/paste
    * [Pull request #1013](https://github.com/ignitionrobotics/ign-gazebo/pull/1013)

1. Tweaks install instructions
    * [Pull request #1078](https://github.com/ignitionrobotics/ign-gazebo/pull/1078)

1. Publish 10 world stats msgs/sec instead of 5
    * [Pull request #1163](https://github.com/ignitionrobotics/ign-gazebo/pull/1163)

1. Add functionality to add entities via the entity tree
    * [Pull request #1101](https://github.com/ignitionrobotics/ign-gazebo/pull/1101)

1. Get updated GUI ECM info when a user presses 'play'
    * [Pull request #1109](https://github.com/ignitionrobotics/ign-gazebo/pull/1109)

1. Create expanding type header to reduce code duplication
    * [Pull request #1169](https://github.com/ignitionrobotics/ign-gazebo/pull/1169)

1. `minimal_scene.sdf` example: add `camera_clip` params
    * [Pull request #1166](https://github.com/ignitionrobotics/ign-gazebo/pull/1166)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/ignitionrobotics/ign-gazebo/pull/1104)

1. Support printing sensors using `ign model`
    * [Pull request #1157](https://github.com/ignitionrobotics/ign-gazebo/pull/1157)

1. Set camera clipping plane distances from the GUI
    * [Pull request #1162](https://github.com/ignitionrobotics/ign-gazebo/pull/1162)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/ignitionrobotics/ign-gazebo/pull/1160)

1. Add a default value for `isHeadlessRendering`.
    * [Pull request #1151](https://github.com/ignitionrobotics/ign-gazebo/pull/1151)

1. Component inspector

    1. Edit material colors
        * [Pull request #1123](https://github.com/ignitionrobotics/ign-gazebo/pull/1123)
        * [Pull request #1186](https://github.com/ignitionrobotics/ign-gazebo/pull/1186)

    1. Fix integers and floats
        * [Pull request #1143](https://github.com/ignitionrobotics/ign-gazebo/pull/1143)

    1. Prevent a segfault when updating
        * [Pull request #1167](https://github.com/ignitionrobotics/ign-gazebo/pull/1167)

    1. Use `uint64_t` for Entity IDs
        * [Pull request #1144](https://github.com/ignitionrobotics/ign-gazebo/pull/1144)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/ignitionrobotics/ign-gazebo/pull/1147)

1. Select top level entity not visual
    * [Pull request #1150](https://github.com/ignitionrobotics/ign-gazebo/pull/1150)

1. Update create entity offset on GUI side
    * [Pull request #1145](https://github.com/ignitionrobotics/ign-gazebo/pull/1145)

1. Update Select Entities GUI plugin to use Entity type
    * [Pull request #1146](https://github.com/ignitionrobotics/ign-gazebo/pull/1146)

1. Notify other GUI plugins of added/removed entities via GUI events
    * [Pull request #1138](https://github.com/ignitionrobotics/ign-gazebo/pull/1138)
    * [Pull request #1213](https://github.com/ignitionrobotics/ign-gazebo/pull/1213)

### Ignition Gazebo 6.1.0 (2021-10-25)

1. Updates to camera video record from subt
    * [Pull request #1117](https://github.com/ignitionrobotics/ign-gazebo/pull/1117)

1. Use the actor tension parameter
    * [Pull request #1091](https://github.com/ignitionrobotics/ign-gazebo/pull/1091)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull request #1119](https://github.com/ignitionrobotics/ign-gazebo/pull/1119)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/ignitionrobotics/ign-gazebo/pull/1095)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/ignitionrobotics/ign-gazebo/pull/1089)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/ignitionrobotics/ign-gazebo/pull/1098)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/ignitionrobotics/ign-gazebo/pull/1043)

1. Prevent crash and print error
    * [Pull request #1099](https://github.com/ignitionrobotics/ign-gazebo/pull/1099)

1. Create GUI config folder before copying config
    * [Pull request #1092](https://github.com/ignitionrobotics/ign-gazebo/pull/1092)

1. Add support for configuring point size in Visualize Lidar GUI plugin
    * [Pull request #1021](https://github.com/ignitionrobotics/ign-gazebo/pull/1021)

1. Set a cloned joint's parent/child link names to the cloned parent/child link names
    * [Pull request #1075](https://github.com/ignitionrobotics/ign-gazebo/pull/1075)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/ignitionrobotics/ign-gazebo/pull/1083)

1. Fix transform controls
    * [Pull request #1081](https://github.com/ignitionrobotics/ign-gazebo/pull/1081)

1. Fix View Angle's home button
    * [Pull request #1082](https://github.com/ignitionrobotics/ign-gazebo/pull/1082)

1. Fix light control standalone example
    * [Pull request #1077](https://github.com/ignitionrobotics/ign-gazebo/pull/1077)

1. Parse new param for enabling / disabling IMU orientation output
    * [Pull request #899](https://github.com/ignitionrobotics/ign-gazebo/pull/899)

### Ignition Gazebo 6.0.0 (2021-10-01)

1. Deprecated GzScene3D in favor of MinimalScene
    * [Pull request #1065](https://github.com/ignitionrobotics/ign-gazebo/pull/1065)
    * [Pull request #1051](https://github.com/ignitionrobotics/ign-gazebo/pull/1051)
    * [Pull request #1014](https://github.com/ignitionrobotics/ign-gazebo/pull/1014)
    * [Pull request #1034](https://github.com/ignitionrobotics/ign-gazebo/pull/1034)
    * [Pull request #900](https://github.com/ignitionrobotics/ign-gazebo/pull/900)
    * [Pull request #988](https://github.com/ignitionrobotics/ign-gazebo/pull/988)
    * [Pull request #1016](https://github.com/ignitionrobotics/ign-gazebo/pull/1016)
    * [Pull request #983](https://github.com/ignitionrobotics/ign-gazebo/pull/983)
    * [Pull request #854](https://github.com/ignitionrobotics/ign-gazebo/pull/854)
    * [Pull request #813](https://github.com/ignitionrobotics/ign-gazebo/pull/813)
    * [Pull request #905](https://github.com/ignitionrobotics/ign-gazebo/pull/905)

1. Fix GuiRunner initial state and entity spawn timing issue
    * [Pull request #1073](https://github.com/ignitionrobotics/ign-gazebo/pull/1073)

1. Buoyancy plugin upgrade
    * [Pull request #818](https://github.com/ignitionrobotics/ign-gazebo/pull/818)
    * [Pull request #1067](https://github.com/ignitionrobotics/ign-gazebo/pull/1067)
    * [Pull request #1064](https://github.com/ignitionrobotics/ign-gazebo/pull/1064)

1. Fix non desired window opening alongside ignition GUI
    * [Pull request #1063](https://github.com/ignitionrobotics/ign-gazebo/pull/1063)

1. Documentation
    * [Pull request #1074](https://github.com/ignitionrobotics/ign-gazebo/pull/1074)
    * [Pull request #996](https://github.com/ignitionrobotics/ign-gazebo/pull/996)

1. Update to latest SDFormat changes
    * [Pull request #1069](https://github.com/ignitionrobotics/ign-gazebo/pull/1069)
    * [Pull request #1023](https://github.com/ignitionrobotics/ign-gazebo/pull/1023)

1. Suppress missing canonical link error messages for static models
    * [Pull request #1068](https://github.com/ignitionrobotics/ign-gazebo/pull/1068)

1. Heightmap fixes
    * [Pull request #1055](https://github.com/ignitionrobotics/ign-gazebo/pull/1055)
    * [Pull request #1054](https://github.com/ignitionrobotics/ign-gazebo/pull/1054)

1. Place config files in a versioned directory
    * [Pull request #1050](https://github.com/ignitionrobotics/ign-gazebo/pull/1050)
    * [Pull request #1070](https://github.com/ignitionrobotics/ign-gazebo/pull/1070)

1. Fix GUI crash when accessing bad rendering UserData
    * [Pull request #1052](https://github.com/ignitionrobotics/ign-gazebo/pull/1052)

1. Fix performance issue with contact data and AABB updates
    * [Pull request #1048](https://github.com/ignitionrobotics/ign-gazebo/pull/1048)

1. Enable new policy to fix protobuf compilation errors
    * [Pull request #1046](https://github.com/ignitionrobotics/ign-gazebo/pull/1046)

1. Support locked entities, and headless video recording using sim time
    * [Pull request #862](https://github.com/ignitionrobotics/ign-gazebo/pull/862)

1. Label Component & System, segmentation camera support
    * [Pull request #853](https://github.com/ignitionrobotics/ign-gazebo/pull/853)
    * [Pull request #1047](https://github.com/ignitionrobotics/ign-gazebo/pull/1047)

1. Joint Force-Torque Systems Plugin
    * [Pull request #977](https://github.com/ignitionrobotics/ign-gazebo/pull/977)

1. Add support for cloning entities
    * [Pull request #959](https://github.com/ignitionrobotics/ign-gazebo/pull/959)

1. 🌐 Spherical coordinates
    * [Pull request #1008](https://github.com/ignitionrobotics/ign-gazebo/pull/1008)

1. Populate JointConstraintWrench from physics
    * [Pull request #989](https://github.com/ignitionrobotics/ign-gazebo/pull/989)

1. Buoyancy engine
    * [Pull request #1009](https://github.com/ignitionrobotics/ign-gazebo/pull/1009)

1. Infrastructure
    * [Pull request #1033](https://github.com/ignitionrobotics/ign-gazebo/pull/1033)
    * [Pull request #1029](https://github.com/ignitionrobotics/ign-gazebo/pull/1029)
    * [Pull request #991](https://github.com/ignitionrobotics/ign-gazebo/pull/991)
    * [Pull request #809](https://github.com/ignitionrobotics/ign-gazebo/pull/809)

1. Update on resize instead of pre-render / render
    * [Pull request #1028](https://github.com/ignitionrobotics/ign-gazebo/pull/1028)

1. Add a flag to force headless rendering mode
    * [Pull request #701](https://github.com/ignitionrobotics/ign-gazebo/pull/701)

1. Remove unused ignition gui header
    * [Pull request #1026](https://github.com/ignitionrobotics/ign-gazebo/pull/1026)

1. Adds velocity control to JointPositionController.
    * [Pull request #1003](https://github.com/ignitionrobotics/ign-gazebo/pull/1003)

1. Collada world exporter now exporting lights
    * [Pull request #912](https://github.com/ignitionrobotics/ign-gazebo/pull/912)

1. Workaround for setting visual cast shadows without material
    * [Pull request #1015](https://github.com/ignitionrobotics/ign-gazebo/pull/1015)

1. Fix selection buffer crash on resize
    * [Pull request #969](https://github.com/ignitionrobotics/ign-gazebo/pull/969)

1. Remove extra xml version line in pendulum_links example world
    * [Pull request #1002](https://github.com/ignitionrobotics/ign-gazebo/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/ignitionrobotics/ign-gazebo/pull/982)

1. Add ESC to unselect entities in select entities plugin
    * [Pull request #995](https://github.com/ignitionrobotics/ign-gazebo/pull/995)

1. Visualize joints
    * [Pull request #961](https://github.com/ignitionrobotics/ign-gazebo/pull/961)

1. Deprecate particle emitter, and use scatter ratio in new particle mes…
    * [Pull request #986](https://github.com/ignitionrobotics/ign-gazebo/pull/986)

1. Removed unused variable in Shapes plugin
    * [Pull request #984](https://github.com/ignitionrobotics/ign-gazebo/pull/984)

1. Use root.Model()
    * [Pull request #980](https://github.com/ignitionrobotics/ign-gazebo/pull/980)

1. Add ModelSDF serializer
    * [Pull request #851](https://github.com/ignitionrobotics/ign-gazebo/pull/851)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/ignitionrobotics/ign-gazebo/pull/974)

1. Use statically-typed views for better performance
    * [Pull request #856](https://github.com/ignitionrobotics/ign-gazebo/pull/856)
    * [Pull request #1001](https://github.com/ignitionrobotics/ign-gazebo/pull/1001)

1. Upgrade ign-sensors and support custom sensors
    * [Pull request #617](https://github.com/ignitionrobotics/ign-gazebo/pull/617)

1. Fix entity creation console msg
    * [Pull request #972](https://github.com/ignitionrobotics/ign-gazebo/pull/972)

1. Fix crash in the follow_actor example
    * [Pull request #958](https://github.com/ignitionrobotics/ign-gazebo/pull/958)

1. Removed pose topic from log system
    * [Pull request #839](https://github.com/ignitionrobotics/ign-gazebo/pull/839)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/ignitionrobotics/ign-gazebo/pull/965)

1. Complaint if Joint doesn't exists before adding joint controller
    * [Pull request #786](https://github.com/ignitionrobotics/ign-gazebo/pull/786)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/ignitionrobotics/ign-gazebo/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/ignitionrobotics/ign-gazebo/pull/724)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/ignitionrobotics/ign-gazebo/pull/693)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/ignitionrobotics/ign-gazebo/pull/596)

1.  Add missing IGNITION_GAZEBO_VISIBLE macros
    * [Pull request #563](https://github.com/ignitionrobotics/ign-gazebo/pull/563)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/ignitionrobotics/ign-gazebo/pull/564)

1. No install apt recommends and clear cache
    * [Pull request #423](https://github.com/ignitionrobotics/ign-gazebo/pull/423)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/ignitionrobotics/ign-gazebo/pull/936)

1. Fix tests that use multiple root level models or lights
    * [Pull request #931](https://github.com/ignitionrobotics/ign-gazebo/pull/931)

1. Make Gazebo aware of SetCameraPassCountPerGpuFlush
    * [Pull request #921](https://github.com/ignitionrobotics/ign-gazebo/pull/921)

1. Visualize center of mass
    * [Pull request #903](https://github.com/ignitionrobotics/ign-gazebo/pull/903)

1. Transparent mode
    * [Pull request #878](https://github.com/ignitionrobotics/ign-gazebo/pull/878)

1. Visualize inertia
    * [Pull request #861](https://github.com/ignitionrobotics/ign-gazebo/pull/861)

1. Remove deprecations: tock 🕑
    * [Pull request #875](https://github.com/ignitionrobotics/ign-gazebo/pull/875)

1. Removed and moved tape measure and grid config to ign-gui
    * [Pull request #870](https://github.com/ignitionrobotics/ign-gazebo/pull/870)

1. Update wireframe visualization to support nested models
    * [Pull request #832](https://github.com/ignitionrobotics/ign-gazebo/pull/832)

1. Multi-LRAUV Swimming Race Example
    * [Pull request #841](https://github.com/ignitionrobotics/ign-gazebo/pull/841)

1. Add view control gui plugin and support orthographic view
    * [Pull request #815](https://github.com/ignitionrobotics/ign-gazebo/pull/815)

1. Wireframe mode
    * [Pull request #816](https://github.com/ignitionrobotics/ign-gazebo/pull/816)

1. Explain why detail::View symbols are visible
    * [Pull request #788](https://github.com/ignitionrobotics/ign-gazebo/pull/788)

1. Bump dependencies in fortress
    * [Pull request #764](https://github.com/ignitionrobotics/ign-gazebo/pull/764)

## Ignition Gazebo 5.x

### Ignition Gazebo 5.4.0 (2022-03-31)

1. Add the Model Photo Shoot system, port of Modelpropshop plugin from Gazebo classic
    * [Pull request #1331](https://github.com/ignitionrobotics/ign-gazebo/pull/1331)

1. Add wheel slip user command
    * [Pull request #1241](https://github.com/ignitionrobotics/ign-gazebo/pull/1241)

1. Added user command to set multiple entity poses
    * [Pull request #1394](https://github.com/ignitionrobotics/ign-gazebo/pull/1394)

1. Component inspector: refactor Pose3d C++ code into a separate class
    * [Pull request #1400](https://github.com/ignitionrobotics/ign-gazebo/pull/1400)

1. Toggle Light visuals
    * [Pull request #1387](https://github.com/ignitionrobotics/ign-gazebo/pull/1387)

1. Allow to turn on/off lights
    * [Pull request #1343](https://github.com/ignitionrobotics/ign-gazebo/pull/1343)

1. Added more sensor properties to scene/info topic
    * [Pull request #1344](https://github.com/ignitionrobotics/ign-gazebo/pull/1344)

1. JointStatePublisher publish parent, child and axis data
    * [Pull request #1345](https://github.com/ignitionrobotics/ign-gazebo/pull/1345)

1. Fixed light GUI component inspector
    * [Pull request #1337](https://github.com/ignitionrobotics/ign-gazebo/pull/1337)

1. Fix `UNIT_SdfGenerator_TEST`
    * [Pull request #1319](https://github.com/ignitionrobotics/ign-gazebo/pull/1319)

1. Add elevator system
    * [Pull request #535](https://github.com/ignitionrobotics/ign-gazebo/pull/535)

1. Removed unused variables in shapes plugin
    * [Pull request #1321](https://github.com/ignitionrobotics/ign-gazebo/pull/1321)

1. Log an error if JointPositionController cannot find the joint. (citadel retarget)
    * [Pull request #1314](https://github.com/ignitionrobotics/ign-gazebo/pull/1314)

1. Buoyancy: fix center of volume's reference frame
    * [Pull request #1302](https://github.com/ignitionrobotics/ign-gazebo/pull/1302)

1. Remove EachNew calls from sensor PreUpdates
    * [Pull request #1281](https://github.com/ignitionrobotics/ign-gazebo/pull/1281)

1. Prevent GzScene3D 💥 if another scene is already loaded
    * [Pull request #1294](https://github.com/ignitionrobotics/ign-gazebo/pull/1294)

1. Cleanup update call for non-rendering sensors
    * [Pull request #1282](https://github.com/ignitionrobotics/ign-gazebo/pull/1282)

1. Documentation Error
    * [Pull request #1285](https://github.com/ignitionrobotics/ign-gazebo/pull/1285)

1. Min and max parameters for velocity, acceleration, and jerk apply to linear and angular separately.
    * [Pull request #1229](https://github.com/ignitionrobotics/ign-gazebo/pull/1229)

1. Add project() call to examples
    * [Pull request #1274](https://github.com/ignitionrobotics/ign-gazebo/pull/1274)

1. Implement `/server_control::stop`
    * [Pull request #1240](https://github.com/ignitionrobotics/ign-gazebo/pull/1240)

1. 👩‍🌾 Make depth camera tests more robust (#897)
    * [Pull request #897) (#1257](https://github.com/ignitionrobotics/ign-gazebo/pull/897) (#1257)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/ignitionrobotics/ign-gazebo/pull/1255)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/ignitionrobotics/ign-gazebo/pull/1194)
    * [Pull request #1250](https://github.com/ignitionrobotics/ign-gazebo/pull/1250)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/ignitionrobotics/ign-gazebo/pull/1224)

1. Skip failing Windows tests
    * [Pull request #1205](https://github.com/ignitionrobotics/ign-gazebo/pull/1205)
    * [Pull request #1204](https://github.com/ignitionrobotics/ign-gazebo/pull/1204)
    * [Pull request #1259](https://github.com/ignitionrobotics/ign-gazebo/pull/1259)
    * [Pull request #1408](https://github.com/ignitionrobotics/ign-gazebo/pull/1408)

1. Configurable joint state publisher's topic
    * [Pull request #1076](https://github.com/ignitionrobotics/ign-gazebo/pull/1076)

1. Thruster plugin: add tests and velocity control
    * [Pull request #1190](https://github.com/ignitionrobotics/ign-gazebo/pull/1190)

1. Limit thruster system's input thrust cmd
    * [Pull request #1318](https://github.com/ignitionrobotics/ign-gazebo/pull/1318)

### Ignition Gazebo 5.3.0 (2021-11-12)

1. Prevent creation of spurious <plugin> elements when saving worlds
    * [Pull request #1192](https://github.com/ignitionrobotics/ign-gazebo/pull/1192)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/ignitionrobotics/ign-gazebo/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/ignitionrobotics/ign-gazebo/pull/847)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/ignitionrobotics/ign-gazebo/pull/1181)

1.  Extend odom publisher to allow 3D
    * [Pull request #1180](https://github.com/ignitionrobotics/ign-gazebo/pull/1180)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1131](https://github.com/ignitionrobotics/ign-gazebo/pull/1131)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/ignitionrobotics/ign-gazebo/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/ignitionrobotics/ign-gazebo/pull/1160)

1. Edit material colors in component inspector
    * [Pull request #1123](https://github.com/ignitionrobotics/ign-gazebo/pull/1123)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/ignitionrobotics/ign-gazebo/pull/1147)

1. Use `uint64_t` for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/ignitionrobotics/ign-gazebo/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/ignitionrobotics/ign-gazebo/pull/1143)

### Ignition Gazebo 5.2.0 (2021-10-22)

1. Fix performance level test flakiness
    * [Pull request #1129](https://github.com/ignitionrobotics/ign-gazebo/pull/1129)

1. Updates to camera video record from subt
    * [Pull request #1117](https://github.com/ignitionrobotics/ign-gazebo/pull/1117)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull request #1119](https://github.com/ignitionrobotics/ign-gazebo/pull/1119)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/ignitionrobotics/ign-gazebo/pull/1095)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/ignitionrobotics/ign-gazebo/pull/1089)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/ignitionrobotics/ign-gazebo/pull/1098)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/ignitionrobotics/ign-gazebo/pull/1043)

1. Cache top level and static to speed up physics system (Backport #656)
    * [Pull request #993](https://github.com/ignitionrobotics/ign-gazebo/pull/993)

1. Prevent crash and print error
    * [Pull request #1099](https://github.com/ignitionrobotics/ign-gazebo/pull/1099)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/ignitionrobotics/ign-gazebo/pull/1083)

1. Fix light control standalone example
    * [Pull request #1077](https://github.com/ignitionrobotics/ign-gazebo/pull/1077)

1. Parse new param for enabling / disabling IMU orientation output
    * [Pull request #899](https://github.com/ignitionrobotics/ign-gazebo/pull/899)

1. Enable new policy to fix protobuf compilation errors
    * [Pull request #1059](https://github.com/ignitionrobotics/ign-gazebo/pull/1059)

1. Fix performance issue with contact data and AABB updates
    * [Pull request #1048](https://github.com/ignitionrobotics/ign-gazebo/pull/1048)

1. Support locked entities, and headless video recording using sim time
    * [Pull request #862](https://github.com/ignitionrobotics/ign-gazebo/pull/862)

1. Update ign-gazebo4 changelog
    * [Pull request #1031](https://github.com/ignitionrobotics/ign-gazebo/pull/1031)

1. bump version and update changelog
    * [Pull request #1029](https://github.com/ignitionrobotics/ign-gazebo/pull/1029)

1. Remove unused ignition gui header
    * [Pull request #1026](https://github.com/ignitionrobotics/ign-gazebo/pull/1026)

1. Collada world exporter now exporting lights
    * [Pull request #912](https://github.com/ignitionrobotics/ign-gazebo/pull/912)

1. Fixed GUI's ComponentInspector light parameter
    * [Pull request #1018](https://github.com/ignitionrobotics/ign-gazebo/pull/1018)

1. Workaround for setting visual cast shadows without material
    * [Pull request #1015](https://github.com/ignitionrobotics/ign-gazebo/pull/1015)

1. Fix selection buffer crash on resize
    * [Pull request #969](https://github.com/ignitionrobotics/ign-gazebo/pull/969)

1. Update DART deps to local
    * [Pull request #1005](https://github.com/ignitionrobotics/ign-gazebo/pull/1005)

1. Remove extra xml version line in pendulum_links example world
    * [Pull request #1002](https://github.com/ignitionrobotics/ign-gazebo/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/ignitionrobotics/ign-gazebo/pull/982)

1. Make thermal sensor test more robust
    * [Pull request #994](https://github.com/ignitionrobotics/ign-gazebo/pull/994)

1. Improved doxygen
    * [Pull request #996](https://github.com/ignitionrobotics/ign-gazebo/pull/996)

1. Remove bitbucket-pipelines.yml
    * [Pull request #991](https://github.com/ignitionrobotics/ign-gazebo/pull/991)

1. Removed unused variable in Shapes plugin
    * [Pull request #984](https://github.com/ignitionrobotics/ign-gazebo/pull/984)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/ignitionrobotics/ign-gazebo/pull/974)

1. Updates when forward-porting to v4
    * [Pull request #973](https://github.com/ignitionrobotics/ign-gazebo/pull/973)

1. Don't use $HOME on most tests (InternalFixture)
    * [Pull request #971](https://github.com/ignitionrobotics/ign-gazebo/pull/971)

1. Fix entity creation console msg
    * [Pull request #972](https://github.com/ignitionrobotics/ign-gazebo/pull/972)

1. Fix crash in the follow_actor example
    * [Pull request #958](https://github.com/ignitionrobotics/ign-gazebo/pull/958)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/ignitionrobotics/ign-gazebo/pull/965)

1. Drag and drop meshes into scene
    * [Pull request #939](https://github.com/ignitionrobotics/ign-gazebo/pull/939)

1. Allow referencing links in nested models in LiftDrag
    * [Pull request #955](https://github.com/ignitionrobotics/ign-gazebo/pull/955)

1. Complaint if Joint doesn't exists before adding joint controller
    * [Pull request #786](https://github.com/ignitionrobotics/ign-gazebo/pull/786)

1. Set protobuf_MODULE_COMPATIBLE before any find_package call
    * [Pull request #957](https://github.com/ignitionrobotics/ign-gazebo/pull/957)

1. DiffDrive add enable/disable
    * [Pull request #772](https://github.com/ignitionrobotics/ign-gazebo/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/ignitionrobotics/ign-gazebo/pull/724)

1. Add UserCommands Plugin.
    * [Pull request #719](https://github.com/ignitionrobotics/ign-gazebo/pull/719)

1. Expose a test fixture helper class
    * [Pull request #926](https://github.com/ignitionrobotics/ign-gazebo/pull/926)

1. Fix logic to disable server default plugins loading
    * [Pull request #953](https://github.com/ignitionrobotics/ign-gazebo/pull/953)

1. Porting Dome to Edifice: Windows, deprecations
    * [Pull request #948](https://github.com/ignitionrobotics/ign-gazebo/pull/948)

1. removed unneeded plugin update
    * [Pull request #944](https://github.com/ignitionrobotics/ign-gazebo/pull/944)

1. Functions to enable velocity and acceleration checks on Link
    * [Pull request #935](https://github.com/ignitionrobotics/ign-gazebo/pull/935)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/ignitionrobotics/ign-gazebo/pull/936)

1. 3D plot GUI plugin
    * [Pull request #917](https://github.com/ignitionrobotics/ign-gazebo/pull/917)

1. 4 to 5
    * [Pull request #938](https://github.com/ignitionrobotics/ign-gazebo/pull/938)

1. Fix joint controller without joint vel data
    * [Pull request #937](https://github.com/ignitionrobotics/ign-gazebo/pull/937)

1. 3 to 4
    * [Pull request #933](https://github.com/ignitionrobotics/ign-gazebo/pull/933)

1. Model info CLI `ign model`
    * [Pull request #893](https://github.com/ignitionrobotics/ign-gazebo/pull/893)

1. Support Bullet on Edifice
    * [Pull request #919](https://github.com/ignitionrobotics/ign-gazebo/pull/919)

1. Don't create components for entities that don't exist
    * [Pull request #927](https://github.com/ignitionrobotics/ign-gazebo/pull/927)

1. Fix blender sdf export script and remove .material file from collada light export test
    * [Pull request #923](https://github.com/ignitionrobotics/ign-gazebo/pull/923)

1. Heightmap physics (with DART)
    * [Pull request #661](https://github.com/ignitionrobotics/ign-gazebo/pull/661)

1. Adds Mesh Tutorial
    * [Pull request #915](https://github.com/ignitionrobotics/ign-gazebo/pull/915)

1. 4 to 5
    * [Pull request #918](https://github.com/ignitionrobotics/ign-gazebo/pull/918)

1. Fix updating GUI plugin on load
    * [Pull request #904](https://github.com/ignitionrobotics/ign-gazebo/pull/904)

1. 3 to 4
    * [Pull request #916](https://github.com/ignitionrobotics/ign-gazebo/pull/916)

1. Physics system: update link poses if the canonical link pose has been updated
    * [Pull request #876](https://github.com/ignitionrobotics/ign-gazebo/pull/876)

1. Add blender sdf export tutorial
    * [Pull request #895](https://github.com/ignitionrobotics/ign-gazebo/pull/895)

1. Banana for Scale
    * [Pull request #734](https://github.com/ignitionrobotics/ign-gazebo/pull/734)

1. Fix textures not exporting after loading a world that uses obj models
    * [Pull request #874](https://github.com/ignitionrobotics/ign-gazebo/pull/874)

1. Fix documentation for the Sensor component
    * [Pull request #898](https://github.com/ignitionrobotics/ign-gazebo/pull/898)

1. Make depth camera tests more robust
    * [Pull request #897](https://github.com/ignitionrobotics/ign-gazebo/pull/897)

1. Use UINT64_MAX for kComponentTpyeIDInvalid instead of relying on underflow
    * [Pull request #889](https://github.com/ignitionrobotics/ign-gazebo/pull/889)

1. Fix mouse view control target position
    * [Pull request #879](https://github.com/ignitionrobotics/ign-gazebo/pull/879)

### Ignition Gazebo 5.1.0 (2021-06-29)

1. Depend on SDF 11.2.1, rendering 5.1 and GUI 5.1. Fix Windows.
    * [Pull request #877](https://github.com/ignitionrobotics/ign-gazebo/pull/877)

1. Set gui camera pose
    * [Pull request #863](https://github.com/ignitionrobotics/ign-gazebo/pull/863)

1. Refactor RenderUtil::Update with helper functions
    * [Pull request #858](https://github.com/ignitionrobotics/ign-gazebo/pull/858)

1. Enables confirmation dialog when closing Gazebo.
    * [Pull request #850](https://github.com/ignitionrobotics/ign-gazebo/pull/850)

1. Using math::SpeedLimiter on the diff_drive controller.
    * [Pull request #833](https://github.com/ignitionrobotics/ign-gazebo/pull/833)

1. New example: get an ECM snapshot from an external program
    * [Pull request #859](https://github.com/ignitionrobotics/ign-gazebo/pull/859)

1. Fix WindEffects Plugin bug, not configuring new links
    * [Pull request #844](https://github.com/ignitionrobotics/ign-gazebo/pull/844)

1. Set collision detector and solver from SDF
    * [Pull request #684](https://github.com/ignitionrobotics/ign-gazebo/pull/684)

1. Add Particle Emitter tutorial
    * [Pull request #860](https://github.com/ignitionrobotics/ign-gazebo/pull/860)

1. Fix potentially flaky integration component test case
    * [Pull request #848](https://github.com/ignitionrobotics/ign-gazebo/pull/848)

1. Added follow camera offset service
    * [Pull request #855](https://github.com/ignitionrobotics/ign-gazebo/pull/855)

1. Remove unneeded camera follow offset checks
    * [Pull request #857](https://github.com/ignitionrobotics/ign-gazebo/pull/857)

1. Using math::SpeedLimiter on the ackermann_steering controller.
    * [Pull request #837](https://github.com/ignitionrobotics/ign-gazebo/pull/837)

1. Cleanup and alphabetize plugin headers
    * [Pull request #838](https://github.com/ignitionrobotics/ign-gazebo/pull/838)

1. Fix race condition when rendering the UI
    * [Pull request #774](https://github.com/ignitionrobotics/ign-gazebo/pull/774)

1. Removed duplicated code with rendering::sceneFromFirstRenderEngine
    * [Pull request #819](https://github.com/ignitionrobotics/ign-gazebo/pull/819)

1. Remove unused headers in video_recoder plugin
    * [Pull request #834](https://github.com/ignitionrobotics/ign-gazebo/pull/834)

1. Use moveToHelper from ign-rendering
    * [Pull request #825](https://github.com/ignitionrobotics/ign-gazebo/pull/825)

1. Make halt motion act like a brake
    * [Pull request #830](https://github.com/ignitionrobotics/ign-gazebo/pull/830)

1. Update collision visualization to support nested models
    * [Pull request #823](https://github.com/ignitionrobotics/ign-gazebo/pull/823)

1. Adds support for ocean currents
    * [Pull request #800](https://github.com/ignitionrobotics/ign-gazebo/pull/800)

1. Add conversion for particle scatter ratio field
    * [Pull request #791](https://github.com/ignitionrobotics/ign-gazebo/pull/791)

1. Adding HaltMotion to physics plugin
    * [Pull request #728](https://github.com/ignitionrobotics/ign-gazebo/pull/728)

1. ColladaExporter, export submesh selected
    * [Pull request #802](https://github.com/ignitionrobotics/ign-gazebo/pull/802)

1. Remove tools/code_check and update codecov
    * [Pull request #814](https://github.com/ignitionrobotics/ign-gazebo/pull/814)

1. Trigger delay
    * [Pull request #817](https://github.com/ignitionrobotics/ign-gazebo/pull/817)

1. Map canonical links to their models
    * [Pull request #736](https://github.com/ignitionrobotics/ign-gazebo/pull/736)

1. Fix included nested model expansion in SDF generation
    * [Pull request #768](https://github.com/ignitionrobotics/ign-gazebo/pull/768)

1. Util: Use public API from libsdformat for detecting non-file source
    * [Pull request #794](https://github.com/ignitionrobotics/ign-gazebo/pull/794)

1. Contacts visualization
    * [Pull request #234](https://github.com/ignitionrobotics/ign-gazebo/pull/234)

1. Bump to ign-msgs 7.1 / sdformat 11.1, Windows fixes
    * [Pull request #758](https://github.com/ignitionrobotics/ign-gazebo/pull/758)

1. Add functionalities for optical tactile plugin
    * [Pull request #431](https://github.com/ignitionrobotics/ign-gazebo/pull/431)

1. Fix documentation for EntityComponentManager::EachNew
    * [Pull request #795](https://github.com/ignitionrobotics/ign-gazebo/pull/795)

1. Bump ign-physics version to 3.2
    * [Pull request #792](https://github.com/ignitionrobotics/ign-gazebo/pull/792)

1. Prevent crash on Plotting plugin with mutex
    * [Pull request #747](https://github.com/ignitionrobotics/ign-gazebo/pull/747)

1. 👩‍🌾 Fix Windows build and some warnings
    * [Pull request #782](https://github.com/ignitionrobotics/ign-gazebo/pull/782)

1. Fix ColladaExporter submesh index bug
    * [Pull request #763](https://github.com/ignitionrobotics/ign-gazebo/pull/763)

1. Fix macOS build: components::Name in benchmark
    * [Pull request #784](https://github.com/ignitionrobotics/ign-gazebo/pull/784)

1. Feature/hydrodynamics
    * [Pull request #749](https://github.com/ignitionrobotics/ign-gazebo/pull/749)

1. Don't store duplicate ComponentTypeId in ECM
    * [Pull request #751](https://github.com/ignitionrobotics/ign-gazebo/pull/751)

1. [TPE] Support setting individual link velocity
    * [Pull request #427](https://github.com/ignitionrobotics/ign-gazebo/pull/427)

1. 👩‍🌾 Enable Focal CI
    * [Pull request #646](https://github.com/ignitionrobotics/ign-gazebo/pull/646)

1. Patch particle emitter2 service
    * [Pull request #777](https://github.com/ignitionrobotics/ign-gazebo/pull/777)

1. Add odometry publisher system
    * [Pull request #547](https://github.com/ignitionrobotics/ign-gazebo/pull/547)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/ignitionrobotics/ign-gazebo/pull/772)

1. Update benchmark comparison instructions
    * [Pull request #766](https://github.com/ignitionrobotics/ign-gazebo/pull/766)

1. Fix 'invalid animation update data' msg for actors
    * [Pull request #754](https://github.com/ignitionrobotics/ign-gazebo/pull/754)

1. Fixed particle emitter forward playback
    * [Pull request #745](https://github.com/ignitionrobotics/ign-gazebo/pull/745)

1. ECM's ChangedState gets message with modified components
    * [Pull request #742](https://github.com/ignitionrobotics/ign-gazebo/pull/742)

1. Fixed collision visual bounding boxes
    * [Pull request #746](https://github.com/ignitionrobotics/ign-gazebo/pull/746)

1. Fix compute_rtfs arguments
    * [Pull request #737](https://github.com/ignitionrobotics/ign-gazebo/pull/737)

1. Validate step size and RTF parameters
    * [Pull request #740](https://github.com/ignitionrobotics/ign-gazebo/pull/740)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/ignitionrobotics/ign-gazebo/pull/724)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for compatibility with Protobuf CMake config
    * [Pull request #715](https://github.com/ignitionrobotics/ign-gazebo/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler
    * [Pull request #716](https://github.com/ignitionrobotics/ign-gazebo/pull/716)

1. Iterate through changed links only in UpdateSim
    * [Pull request #678](https://github.com/ignitionrobotics/ign-gazebo/pull/678)

1. Update PlaybackScrubber description
    * [Pull request #733](https://github.com/ignitionrobotics/ign-gazebo/pull/733)

1. Support configuring particle scatter ratio in particle emitter system
    * [Pull request #674](https://github.com/ignitionrobotics/ign-gazebo/pull/674)

1. Fix diffuse and ambient values for ackermann example
    * [Pull request #707](https://github.com/ignitionrobotics/ign-gazebo/pull/707)

1. Scenebroadcaster sensors
    * [Pull request #698](https://github.com/ignitionrobotics/ign-gazebo/pull/698)

1. Add test for thermal object temperatures below 0 kelvin
    * [Pull request #621](https://github.com/ignitionrobotics/ign-gazebo/pull/621)

1. [BULLET] Making GetContactsFromLastStepFeature optional in Collision Features
    * [Pull request #690](https://github.com/ignitionrobotics/ign-gazebo/pull/690)

1. Make it so joint state publisher is quieter
    * [Pull request #696](https://github.com/ignitionrobotics/ign-gazebo/pull/696)

### Ignition Gazebo 5.0.0 (2021-03-30)

1. Added Ellipsoid and Capsule geometries
    * [Pull request #581](https://github.com/ignitionrobotics/ign-gazebo/pull/581)

1. Support individual canonical links for nested models
    * [Pull request #685](https://github.com/ignitionrobotics/ign-gazebo/pull/685)

1. Mecanum wheels demo
    * [Pull request #683](https://github.com/ignitionrobotics/ign-gazebo/pull/683)

1. Fixed collision visual bounding boxes
    * [Pull request #702](https://github.com/ignitionrobotics/ign-gazebo/pull/702)

1. Fixed material colors for ackermann sdfs
    * [Pull request #703](https://github.com/ignitionrobotics/ign-gazebo/pull/703)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/ignitionrobotics/ign-gazebo/pull/693)

1. Remove static for maps from Factory.hh
    * [Pull request #635](https://github.com/ignitionrobotics/ign-gazebo/pull/635)

1. Depend on cli component of ignition-utils1
    * [Pull request #671](https://github.com/ignitionrobotics/ign-gazebo/pull/671)

1. Support SDFormat 1.8 Composition
    * [Pull request #542](https://github.com/ignitionrobotics/ign-gazebo/pull/542)

1. Deprecate TmpIface: it's leftover from prototyping
    * [Pull request #654](https://github.com/ignitionrobotics/ign-gazebo/pull/654)

1. Bump in edifice: ign-common4
    * [Pull request #577](https://github.com/ignitionrobotics/ign-gazebo/pull/577)

1. Plugin to spawn lights
    * [Pull request #587](https://github.com/ignitionrobotics/ign-gazebo/pull/587)

1. Added light intensity
    * [Pull request #612](https://github.com/ignitionrobotics/ign-gazebo/pull/612)
    * [Pull request #670](https://github.com/ignitionrobotics/ign-gazebo/pull/670)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/ignitionrobotics/ign-gazebo/pull/596)

1. Prepare GuiRunner to be made private
    * [Pull request #567](https://github.com/ignitionrobotics/ign-gazebo/pull/567)

1. Deprecate some gazebo::gui events in favor of ign-gui events
    * [Pull request #595](https://github.com/ignitionrobotics/ign-gazebo/pull/595)

1. Heightmap (rendering only)
    * [Pull request #487](https://github.com/ignitionrobotics/ign-gazebo/pull/487)

1. Add image suffix to thermal camera topic name
    * [Pull request #606](https://github.com/ignitionrobotics/ign-gazebo/pull/606)

1. Fix build with latest sdformat11 branch
    * [Pull request #607](https://github.com/ignitionrobotics/ign-gazebo/pull/607)

1. Added run to time feature
    * [Pull request #478](https://github.com/ignitionrobotics/ign-gazebo/pull/478)

1. Depend on ignition-utils1
    * [Pull request #591](https://github.com/ignitionrobotics/ign-gazebo/pull/591)

1. Use double sided field in material msg
    * [Pull request #599](https://github.com/ignitionrobotics/ign-gazebo/pull/599)

1. Add lightmap demo
    * [Pull request #471](https://github.com/ignitionrobotics/ign-gazebo/pull/471)

1. Added renderOrder to convert functions
    * [Pull request #514](https://github.com/ignitionrobotics/ign-gazebo/pull/514)

1. Compilation fixes for Windows
    * [Pull request #501](https://github.com/ignitionrobotics/ign-gazebo/pull/501)
    * [Pull request #585](https://github.com/ignitionrobotics/ign-gazebo/pull/585)
    * [Pull request #565](https://github.com/ignitionrobotics/ign-gazebo/pull/565)
    * [Pull request #616](https://github.com/ignitionrobotics/ign-gazebo/pull/616)
    * [Pull request #622](https://github.com/ignitionrobotics/ign-gazebo/pull/622)

1. Documentation fixes
    * [Pull request #727](https://github.com/ignitionrobotics/ign-gazebo/pull/727)
    * [Pull request #710](https://github.com/ignitionrobotics/ign-gazebo/pull/710)

1. Replace deprecated function FreeGroup::CanonicalLink with FreeGroup::RootLink
    * [Pull request #723](https://github.com/ignitionrobotics/ign-gazebo/pull/723)

1. Respect spotlight direction
    * [Pull request #718](https://github.com/ignitionrobotics/ign-gazebo/pull/718)

1. Add UserCommands plugin to fuel.sdf
    * [Pull request #719](https://github.com/ignitionrobotics/ign-gazebo/pull/719)

1. Change SelectedEntities to return a const ref
    * [Pull request #571](https://github.com/ignitionrobotics/ign-gazebo/pull/571)

1. Use common::setenv for portability to Windows
    * [Pull request #561](https://github.com/ignitionrobotics/ign-gazebo/pull/561)

1.  Add missing IGNITION_GAZEBO_VISIBLE macros
    * [Pull request #563](https://github.com/ignitionrobotics/ign-gazebo/pull/563)

1. Fix deprecation warnings
    * [Pull request #572](https://github.com/ignitionrobotics/ign-gazebo/pull/572)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/ignitionrobotics/ign-gazebo/pull/564)

1. Bump edifice sdformat11 and ign-physics4
    * [Pull request #549](https://github.com/ignitionrobotics/ign-gazebo/pull/549)

1. Use ComponentState::PeriodicChange in UpdateState to avoid forcing full scene update
    * [Pull request #486](https://github.com/ignitionrobotics/ign-gazebo/pull/486)

1. Bump in edifice: ign-msgs7
    * [Pull request #546](https://github.com/ignitionrobotics/ign-gazebo/pull/546)

1. Add support for sky
    * [Pull request #445](https://github.com/ignitionrobotics/ign-gazebo/pull/445)

1. Infrastructure
    * [Pull request #423](https://github.com/ignitionrobotics/ign-gazebo/pull/423)

1. Bump in edifice: ign-rendering5
    * [Pull request #430](https://github.com/ignitionrobotics/ign-gazebo/pull/430)

1. Add 25percent darker view angle icons
    * [Pull request #426](https://github.com/ignitionrobotics/ign-gazebo/pull/426)

## Ignition Gazebo 4.x

### Ignition Gazebo 4.14.0 (2021-12-20)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/ignitionrobotics/ign-gazebo/pull/1255)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/ignitionrobotics/ign-gazebo/pull/1194)
    * [Pull request #1250](https://github.com/ignitionrobotics/ign-gazebo/pull/1250)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/ignitionrobotics/ign-gazebo/pull/1224)

1. Disable user commands light test on macOS
    * [Pull request #1204](https://github.com/ignitionrobotics/ign-gazebo/pull/1204)

### Ignition Gazebo 4.13.0 (2021-11-15)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/ignitionrobotics/ign-gazebo/pull/1192)

1. Add support for tracked vehicles
    * [Pull request #869](https://github.com/ignitionrobotics/ign-gazebo/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/ignitionrobotics/ign-gazebo/pull/847)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/ignitionrobotics/ign-gazebo/pull/1181)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1149](https://github.com/ignitionrobotics/ign-gazebo/pull/1149)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/ignitionrobotics/ign-gazebo/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/ignitionrobotics/ign-gazebo/pull/1160)

1. Edit material colors in component inspector
    * [Pull request #1123](https://github.com/ignitionrobotics/ign-gazebo/pull/1123)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/ignitionrobotics/ign-gazebo/pull/1147)

1. Use uint64_t for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/ignitionrobotics/ign-gazebo/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/ignitionrobotics/ign-gazebo/pull/1143)

### Ignition Gazebo 4.12.0 (2021-10-22)

1. Fix performance issue with contact data and AABB updates.
    * [Pull Request 1048](https://github.com/ignitionrobotics/ign-gazebo/pull/1048)

1. Enable new CMake policy to fix protobuf compilation
    * [Pull Request 1059](https://github.com/ignitionrobotics/ign-gazebo/pull/1059)

1. Parse new param for enabling / disabling IMU orientation output.
    * [Pull Request 899](https://github.com/ignitionrobotics/ign-gazebo/pull/899)

1. Fix light control standalone example.
    * [Pull Request 1077](https://github.com/ignitionrobotics/ign-gazebo/pull/1077)

1. Performance: use std::unordered_map where possible in SceneManager.
    * [Pull Request 1083](https://github.com/ignitionrobotics/ign-gazebo/pull/1083)

1. Prevent crash when using <specular> workflow PBR material.
    * [Pull Request 1099](https://github.com/ignitionrobotics/ign-gazebo/pull/1099)

1. JointPositionController: Improve misleading error message.
    * [Pull Request 1098](https://github.com/ignitionrobotics/ign-gazebo/pull/1098)

1. Adjust pose decimals based on element width.
    * [Pull Request 1089](https://github.com/ignitionrobotics/ign-gazebo/pull/1089)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull Request 1119](https://github.com/ignitionrobotics/ign-gazebo/pull/1089)

1. Updates to camera video record from subt.
    * [Pull Request 1117](https://github.com/ignitionrobotics/ign-gazebo/pull/1117)

1. Fix performance level test flakiness.
    * [Pull Request 1129](https://github.com/ignitionrobotics/ign-gazebo/pull/1129)

### Ignition Gazebo 4.11.0 (2021-09-23)

1. Support locked entities, and headless video recording using sim time.
    * [Pull Request 862](https://github.com/ignitionrobotics/ign-gazebo/pull/862)

### Ignition Gazebo 4.10.0 (2021-09-15)

1. Fixed GUI's ComponentInspector light parameter
    * [Pull Request 1018](https://github.com/ignitionrobotics/ign-gazebo/pull/1018)

1. Fix msg in entity_creation example
    * [Pull Request 972](https://github.com/ignitionrobotics/ign-gazebo/pull/972)

1. Fix selection buffer crash on resize
    * [Pull Request 969](https://github.com/ignitionrobotics/ign-gazebo/pull/969)

1. Fix crash in the follow_actor example
    * [Pull Request 958](https://github.com/ignitionrobotics/ign-gazebo/pull/958)

1. Fix joint controller with empty joint velocity data
    * [Pull Request 937](https://github.com/ignitionrobotics/ign-gazebo/pull/937)

1. Scale mode - Part2
    * [Pull Request 881](https://github.com/ignitionrobotics/ign-gazebo/pull/881)

1. Physics system: update link poses if the canonical link pose has been updated
    * [Pull Request 876](https://github.com/ignitionrobotics/ign-gazebo/pull/876)

1. Add Particle Emitter tutorial
    * [Pull Request 860](https://github.com/ignitionrobotics/ign-gazebo/pull/860)

1. Refactor RenderUtil::Update with helper functions
    * [Pull Request 858](https://github.com/ignitionrobotics/ign-gazebo/pull/858)

1. Remove unneeded camera follow offset checks
    * [Pull Request 857](https://github.com/ignitionrobotics/ign-gazebo/pull/857)

1. Added service to set camera's follow offset
    * [Pull Request 855](https://github.com/ignitionrobotics/ign-gazebo/pull/855)

1. Using math::SpeedLimiter on the ackermann_steering controller.
    * [Pull Request 837](https://github.com/ignitionrobotics/ign-gazebo/pull/837)

1. All changes merged forward from ign-gazebo3
    * [Pull Request 866](https://github.com/ignitionrobotics/ign-gazebo/pull/866)
    * [Pull Request 916](https://github.com/ignitionrobotics/ign-gazebo/pull/916)
    * [Pull Request 933](https://github.com/ignitionrobotics/ign-gazebo/pull/933)
    * [Pull Request 946](https://github.com/ignitionrobotics/ign-gazebo/pull/946)
    * [Pull Request 973](https://github.com/ignitionrobotics/ign-gazebo/pull/973)
    * [Pull Request 1017](https://github.com/ignitionrobotics/ign-gazebo/pull/1017)

### Ignition Gazebo 4.9.1 (2021-05-24)

1. Make halt motion act like a brake.
    * [Pull Request 830](https://github.com/ignitionrobotics/ign-gazebo/pull/830)

### Ignition Gazebo 4.9.0 (2021-05-20)

1. Enable Focal CI.
    * [Pull Request 646](https://github.com/ignitionrobotics/ign-gazebo/pull/646)

1. [TPE] Support setting individual link velocity.
    * [Pull Request 427](https://github.com/ignitionrobotics/ign-gazebo/pull/427)

1. Don't store duplicate ComponentTypeId in ECM.
    * [Pull Request 751](https://github.com/ignitionrobotics/ign-gazebo/pull/751)

1. Fix macOS build: components::Name in benchmark.
    * [Pull Request 784](https://github.com/ignitionrobotics/ign-gazebo/pull/784)

1. Fix documentation for EntityComponentManager::EachNew.
    * [Pull Request 795](https://github.com/ignitionrobotics/ign-gazebo/pull/795)

1. Add functionalities for optical tactile plugin.
    * [Pull Request 431](https://github.com/ignitionrobotics/ign-gazebo/pull/431)

1. Visualize ContactSensorData.
    * [Pull Request 234](https://github.com/ignitionrobotics/ign-gazebo/pull/234)

1. Backport PR #763.
    * [Pull Request 804](https://github.com/ignitionrobotics/ign-gazebo/pull/804)

1. Backport PR #536.
    * [Pull Request 812](https://github.com/ignitionrobotics/ign-gazebo/pull/812)

1. Add an optional delay to the TriggeredPublisher system.
    * [Pull Request 817](https://github.com/ignitionrobotics/ign-gazebo/pull/817)

1. Remove tools/code_check and update codecov.
    * [Pull Request 814](https://github.com/ignitionrobotics/ign-gazebo/pull/814)

1. add conversion for particle scatter ratio field.
    * [Pull Request 791](https://github.com/ignitionrobotics/ign-gazebo/pull/791)

### Ignition Gazebo 4.8.0 (2021-04-22)

1. Add odometry publisher system.
    * [Pull Request 547](https://github.com/ignitionrobotics/ign-gazebo/pull/547)

1. Patch particle emitter2 service.
    * [Pull Request 777](https://github.com/ignitionrobotics/ign-gazebo/pull/777)

### Ignition Gazebo 4.7.0 (2021-04-09)

1. Particle emitter based on SDF.
    * [Pull Request 730](https://github.com/ignitionrobotics/ign-gazebo/pull/730)

1. Fix log playback for particle emitters.
    * [Pull Request 745](https://github.com/ignitionrobotics/ign-gazebo/pull/745)

1. ECM's ChangedState gets message with modified components.
    * [Pull Request 742](https://github.com/ignitionrobotics/ign-gazebo/pull/742)

1. Fixed collision visual bounding boxes.
    * [Pull Request 746](https://github.com/ignitionrobotics/ign-gazebo/pull/746)

1. Fix compute_rtfs arguments.
    * [Pull Request 737](https://github.com/ignitionrobotics/ign-gazebo/pull/737)

1. Validate step size and RTF parameters.
    * [Pull Request 740](https://github.com/ignitionrobotics/ign-gazebo/pull/740)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for
   compatibility with Protobuf CMake config.
    * [Pull Request 715](https://github.com/ignitionrobotics/ign-gazebo/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler.
    * [Pull Request 716](https://github.com/ignitionrobotics/ign-gazebo/pull/716)

1. Support configuring particle scatter ratio in particle emitter system.
    * [Pull Request 674](https://github.com/ignitionrobotics/ign-gazebo/pull/674)

1. Fix diffuse and ambient values for ackermann example.
    * [Pull Request 707](https://github.com/ignitionrobotics/ign-gazebo/pull/707)

1. Scenebroadcaster sensors.
    * [Pull Request 698](https://github.com/ignitionrobotics/ign-gazebo/pull/698)

1. Add thermal camera test for object temperature below 0.
    * [Pull Request 621](https://github.com/ignitionrobotics/ign-gazebo/pull/621)

1. [BULLET] Making GetContactsFromLastStepFeature optional in Collision Features
    * [Pull Request 690](https://github.com/ignitionrobotics/ign-gazebo/pull/690)

1. Fix joint controller GUI test.
    * [Pull Request 697](https://github.com/ignitionrobotics/ign-gazebo/pull/697)

1. Quiet warnings from Joint State Publisher.
    * [Pull Request 696](https://github.com/ignitionrobotics/ign-gazebo/pull/696)

1. Ackermann Steering Plugin.
    * [Pull Request 618](https://github.com/ignitionrobotics/ign-gazebo/pull/618)

1. Remove bounding box when model is deleted
    * [Pull Request 675](https://github.com/ignitionrobotics/ign-gazebo/pull/675)

1. Cache link poses to improve performance.
    * [Pull Request 669](https://github.com/ignitionrobotics/ign-gazebo/pull/669)

1. Check empty world name in Scene3d.
    * [Pull Request 662](https://github.com/ignitionrobotics/ign-gazebo/pull/662)

1. All changes up to 3.8.0.

### Ignition Gazebo 4.6.0 (2021-03-01)

1. Use a custom data structure to manage entity feature maps.
    * [Pull Request 586](https://github.com/ignitionrobotics/ign-gazebo/pull/586)

1. Limit scene broadcast publications when paused.
    * [Pull Request 497](https://github.com/ignitionrobotics/ign-gazebo/pull/497)

1. Report performer count in PerformerDetector plugin.
    * [Pull Request 652](https://github.com/ignitionrobotics/ign-gazebo/pull/652)

1. Cache top level and static to speed up physics system.
    * [Pull Request 656](https://github.com/ignitionrobotics/ign-gazebo/pull/656)

1. Support particle emitter modification using partial message.
    * [Pull Request 651](https://github.com/ignitionrobotics/ign-gazebo/pull/651)

1. Set LD_LIBRARY_PATH on Actions CI.
    * [Pull Request 650](https://github.com/ignitionrobotics/ign-gazebo/pull/650)

1. Fix flaky SceneBroadcaster test.
    * [Pull Request 641](https://github.com/ignitionrobotics/ign-gazebo/pull/641)

1. Add a convenience function for getting possibly non-existing components.
    * [Pull Request 629](https://github.com/ignitionrobotics/ign-gazebo/pull/629)

1. Add msg to show the computed temperature range computed from temperature
   gradient.
    * [Pull Request 643](https://github.com/ignitionrobotics/ign-gazebo/pull/643)

1. Add TF/Pose_V pub in DiffDrive.
    * [Pull Request 548](https://github.com/ignitionrobotics/ign-gazebo/pull/548)

1. Relax flaky performance test.
    * [Pull Request 640](https://github.com/ignitionrobotics/ign-gazebo/pull/640)

1. Improve velocity control test.
    * [Pull Request 642](https://github.com/ignitionrobotics/ign-gazebo/pull/642)

1. Validity check for user defined topics in JointPositionController.
    * [Pull Request 639](https://github.com/ignitionrobotics/ign-gazebo/pull/639)

1. Add laser_retro support.
    * [Pull Request 603](https://github.com/ignitionrobotics/ign-gazebo/pull/603)

1. Fix pose of plane visual with non-default normal vector.
    * [Pull Request 574](https://github.com/ignitionrobotics/ign-gazebo/pull/574)

### Ignition Gazebo 4.5.0 (2020-02-17)

1. Added particle system.
    * [Pull Request 516](https://github.com/ignitionrobotics/ign-gazebo/pull/516)

1. Add Light Usercommand and include Light parameters in the componentInspector
    * [Pull Request 482](https://github.com/ignitionrobotics/ign-gazebo/pull/482)

1. Added link to HW-accelerated video recording.
    * [Pull Request 627](https://github.com/ignitionrobotics/ign-gazebo/pull/627)

1. Fix EntityComponentManager race condition.
    * [Pull Request 601](https://github.com/ignitionrobotics/ign-gazebo/pull/601)

1. Add SDF topic validity check.
    * [Pull Request 632](https://github.com/ignitionrobotics/ign-gazebo/pull/632)

1. Add JointTrajectoryController system plugin.
    * [Pull Request 473](https://github.com/ignitionrobotics/ign-gazebo/pull/473)

### Ignition Gazebo 4.4.0 (2020-02-10)

1. Added issue and PR templates
    * [Pull Request 613](https://github.com/ignitionrobotics/ign-gazebo/pull/613)

1. Fix segfault in SetRemovedComponentsMsgs method
    * [Pull Request 495](https://github.com/ignitionrobotics/ign-gazebo/pull/495)

1. Make topics configurable for joint controllers
    * [Pull Request 584](https://github.com/ignitionrobotics/ign-gazebo/pull/584)

1. Add about dialog
    * [Pull Request 609](https://github.com/ignitionrobotics/ign-gazebo/pull/609)

1. Add thermal sensor system for configuring thermal camera properties
    * [Pull Request 614](https://github.com/ignitionrobotics/ign-gazebo/pull/614)

### Ignition Gazebo 4.3.0 (2020-02-02)

1. Non-blocking paths request.
    * [Pull Request 555](https://github.com/ignitionrobotics/ign-gazebo/pull/555)

1. Parallelize State call in ECM.
    * [Pull Request 451](https://github.com/ignitionrobotics/ign-gazebo/pull/451)

1. Allow to create light with the create service.
    * [Pull Request 513](https://github.com/ignitionrobotics/ign-gazebo/pull/513)

1. Added size to ground_plane in examples.
    * [Pull Request 573](https://github.com/ignitionrobotics/ign-gazebo/pull/573)

1. Fix finding PBR materials.
    * [Pull Request 575](https://github.com/ignitionrobotics/ign-gazebo/pull/575)

1. Publish all periodic change components in Scene Broadcaster.
    * [Pull Request 544](https://github.com/ignitionrobotics/ign-gazebo/pull/544)

1. Backport state update changes from pull request [#486](https://github.com/ignitionrobotics/ign-gazebo/pull/486).
    * [Pull Request 583](https://github.com/ignitionrobotics/ign-gazebo/pull/583)

1. Fix code_check errors.
    * [Pull Request 582](https://github.com/ignitionrobotics/ign-gazebo/pull/582)

1. Visualize collisions.
    * [Pull Request 531](https://github.com/ignitionrobotics/ign-gazebo/pull/531)

1. Remove playback <path> SDF param in Dome.
    * [Pull Request 570](https://github.com/ignitionrobotics/ign-gazebo/pull/570)

1. Tutorial on migrating SDF files from Gazebo classic.
    * [Pull Request 400](https://github.com/ignitionrobotics/ign-gazebo/pull/400)

1. World Exporter.
    * [Pull Request 474](https://github.com/ignitionrobotics/ign-gazebo/pull/474)

1. Model Creation tutorial using services.
    * [Pull Request 530](https://github.com/ignitionrobotics/ign-gazebo/pull/530)

1. Fix topLevelModel Method.
    * [Pull Request 600](https://github.com/ignitionrobotics/ign-gazebo/pull/600)

1. Add heat signature option to thermal system.
    * [Pull Request 498](https://github.com/ignitionrobotics/ign-gazebo/pull/498)

1. Add service and GUI to configure physics parameters (step size and RTF).
    * [Pull Request 536](https://github.com/ignitionrobotics/ign-gazebo/pull/536)

1. Refactor UNIT_Server_TEST.
    * [Pull Request 594](https://github.com/ignitionrobotics/ign-gazebo/pull/594)

1. Use Ignition GUI render event.
    * [Pull Request 598](https://github.com/ignitionrobotics/ign-gazebo/pull/598)

### Ignition Gazebo 4.2.0 (2020-01-13)

1. Automatically load a subset of world plugins.
    * [Pull Request 537](https://github.com/ignitionrobotics/ign-gazebo/pull/537)

1. Fix to handle multiple logical cameras.
    * [Pull Request 539](https://github.com/ignitionrobotics/ign-gazebo/pull/539)

1. Improve ign tool support on macOS.
    * [Pull Request 477](https://github.com/ignitionrobotics/ign-gazebo/pull/477)

1. Add support for topic statistics on breadcrumb deployments.
    * [Pull Request 532](https://github.com/ignitionrobotics/ign-gazebo/pull/532)

1. Fix slot in Plotting plugin.
    * [Pull Request 490](https://github.com/ignitionrobotics/ign-gazebo/pull/490)

1. Fix shadow artifacts by disabling double sided rendering.
    * [Pull Request 446](https://github.com/ignitionrobotics/ign-gazebo/pull/446)

1. Kinetic energy monitor plugin.
    * [Pull Request 492](https://github.com/ignitionrobotics/ign-gazebo/pull/492)

1. Change nullptr to a int ptr for qt 5.15.2.
    * [Pull Request 527](https://github.com/ignitionrobotics/ign-gazebo/pull/527)

1. Generate valid topics everywhere (support names with spaces).
    * [Pull Request 522](https://github.com/ignitionrobotics/ign-gazebo/pull/522)

1. All changes up to version 3.7.0.

### Ignition Gazebo 4.1.0 (2020-12-11)

1. Update Dockerfiles to use focal images
    * [pull request 388](https://github.com/ignitionrobotics/ign-gazebo/pull/388)

1. Updated source build instructions for ign-gazebo4
    * [pull request 404](https://github.com/ignitionrobotics/ign-gazebo/pull/404)

1. Add tests for the AnimationTime component
    * [pull request 433](https://github.com/ignitionrobotics/ign-gazebo/pull/433)

1. Fix pose msg conversion when msg is missing orientation
    * [pull request 450](https://github.com/ignitionrobotics/ign-gazebo/pull/450)
    * [pull request 459](https://github.com/ignitionrobotics/ign-gazebo/pull/459)

1. Resolved updated codecheck issues
    * [pull request 443](https://github.com/ignitionrobotics/ign-gazebo/pull/443)
    * [pull request 457](https://github.com/ignitionrobotics/ign-gazebo/pull/457)
    * [pull request 459](https://github.com/ignitionrobotics/ign-gazebo/pull/459)

1. Use new backpack version in tests
    * [pull request 455](https://github.com/ignitionrobotics/ign-gazebo/pull/455)
    * [pull request 457](https://github.com/ignitionrobotics/ign-gazebo/pull/457)
    * [pull request 459](https://github.com/ignitionrobotics/ign-gazebo/pull/459)

1. Fix segfault in the Breadcrumb system when associated model is unloaded
    * [pull request 454](https://github.com/ignitionrobotics/ign-gazebo/pull/454)
    * [pull request 457](https://github.com/ignitionrobotics/ign-gazebo/pull/457)
    * [pull request 459](https://github.com/ignitionrobotics/ign-gazebo/pull/459)

1. Added user commands to example thermal camera world
    * [pull request 442](https://github.com/ignitionrobotics/ign-gazebo/pull/442)
    * [pull request 459](https://github.com/ignitionrobotics/ign-gazebo/pull/459)

1. Helper function to set component data
    * [pull request 436](https://github.com/ignitionrobotics/ign-gazebo/pull/436)
    * [pull request 469](https://github.com/ignitionrobotics/ign-gazebo/pull/469)

1. Remove unneeded if statement
    * [pull request 432](https://github.com/ignitionrobotics/ign-gazebo/pull/432)
    * [pull request 469](https://github.com/ignitionrobotics/ign-gazebo/pull/469)

1. Fix flaky RecordAndPlayback test in INTEGRATION_log_system
    * [pull request 463](https://github.com/ignitionrobotics/ign-gazebo/pull/463)
    * [pull request 469](https://github.com/ignitionrobotics/ign-gazebo/pull/469)

1. Make PeerTracker test more robust
    * [pull request 452](https://github.com/ignitionrobotics/ign-gazebo/pull/452)
    * [pull request 469](https://github.com/ignitionrobotics/ign-gazebo/pull/469)

1. Use a [std::promise](https://en.cppreference.com/w/cpp/thread/promise)/[std::future](https://en.cppreference.com/w/cpp/thread/future) mechanism to avoid waiting in a looop until all `stepAck` messages are received
    * [pull request 470](https://github.com/ignitionrobotics/ign-gazebo/pull/470)

1. Optical Tactile Sensor Plugin
    * [pull request 229](https://github.com/ignitionrobotics/ign-gazebo/pull/229)

1. All changes up to and including those in version 3.5.0 and version 2.25.0

### Ignition Gazebo 4.0.0 (2020-09-30)

1. Names with spaces: add string serializer
    * [pull request 244](https://github.com/ignitionrobotics/ign-gazebo/pull/244)

1. Filter mesh collision based on `collide_bitmask` property
    * [pull request 160](https://github.com/ignitionrobotics/ign-gazebo/pull/160)

1. Add force focus when mouse enters render window
    * [pull request 97](https://github.com/ignitionrobotics/ign-gazebo/pull/97)

1. Fixed docblock showGrid
    * [pull request 152](https://github.com/ignitionrobotics/ign-gazebo/pull/152)

1. More actor components and follow plugin
    * [pull request 157](https://github.com/ignitionrobotics/ign-gazebo/pull/157)

1. Filter the record menu and write the format to the file according to which button the user pushed (mp4 or ogv)
    * [pull request 153](https://github.com/ignitionrobotics/ign-gazebo/pull/153)

1. Fix scene manager losing header file
    * [pull request 211](https://github.com/ignitionrobotics/ign-gazebo/pull/211)

1. Fixed left menu events
    * [pull request 218](https://github.com/ignitionrobotics/ign-gazebo/pull/218)

1. Fix yaw units typo in Component Inspector plugin
    * [pull request 238](https://github.com/ignitionrobotics/ign-gazebo/pull/238)

1. Enable alpha based transparency on PBR materials by default
    * [pull request 249](https://github.com/ignitionrobotics/ign-gazebo/pull/249)

1. Qt auto scale factor for HiDPI displays
    * [pull request 291](https://github.com/ignitionrobotics/ign-gazebo/pull/291)

1. Sync components removal
    * [pull request 272](https://github.com/ignitionrobotics/ign-gazebo/pull/272)

1. Add error handling for JointAxis::SetXyz and remove use of use_parent_model_frame
    * [pull request 288](https://github.com/ignitionrobotics/ign-gazebo/pull/288)

1. Make some tests more robust
    * [pull request 314](https://github.com/ignitionrobotics/ign-gazebo/pull/314)

1. Fix Qt5 warnings for using anchors
    * [pull request 363](https://github.com/ignitionrobotics/ign-gazebo/pull/363)

1. Plotting Components Plugin
    * [pull request 270](https://github.com/ignitionrobotics/ign-gazebo/pull/270)

1. Visualize Lidar Plugin
    * [pull request 301](https://github.com/ignitionrobotics/ign-gazebo/pull/301)
    * [pull request 391](https://github.com/ignitionrobotics/ign-gazebo/pull/391)

1. Replaced common::Time for std::chrono
    * [pull request 309](https://github.com/ignitionrobotics/ign-gazebo/pull/309)

1. Tutorial, examples and documentation updates
    * [pull request 380](https://github.com/ignitionrobotics/ign-gazebo/pull/380)
    * [pull request 386](https://github.com/ignitionrobotics/ign-gazebo/pull/386)
    * [pull request 387](https://github.com/ignitionrobotics/ign-gazebo/pull/387)
    * [pull request 390](https://github.com/ignitionrobotics/ign-gazebo/pull/390)

1. Migration from BitBucket to GitHub
    * [pull request 73](https://github.com/ignitionrobotics/ign-gazebo/pull/73)
    * [pull request 68](https://github.com/ignitionrobotics/ign-gazebo/pull/68)
    * [pull request 67](https://github.com/ignitionrobotics/ign-gazebo/pull/67)
    * [pull request 130](https://github.com/ignitionrobotics/ign-gazebo/pull/130)

1. Use interpolate\_x sdf parameter for actor animations
    * [BitBucket pull request 536](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/536)

1. Actor skeleton animation (auto update mode)
    * [BitBucket pull request 579](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/579)

1. Added support for removing sensors at runtime
    * [BitBucket pull request 558](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/558)

1. Add support for visual visibility flags and camera visibility mask
    * [BitBucket pull request 559](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/559)

1. Support <actor><pose> and <actor><plugin>
    * [BitBucket pull request 542](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/542)

1. Depend on ign-rendering4, ign-gui4, ign-sensors4
    * [BitBucket pull request 540](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/540)

1. Axis-Aligned Bounding Boxes
    * [BitBucket pull request 565](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/565)

1. Add window focus upon mouse entering the render window
    * [Github pull request 97](https://github.com/ignitionrobotics/ign-gazebo/pull/97)

## Ignition Gazebo 3.x

### Ignition Gazebo 3.13.0 (2022-06-01)

1. Extruded 2D polyline geometries
    * [Pull request #1456](https://github.com/gazebosim/gz-sim/pull/1456)

1. Add elevator system
    * [Pull request #535](https://github.com/gazebosim/gz-sim/pull/535)

1. Add desktop entry and svg logo
    * [Pull request #1411](https://github.com/gazebosim/gz-sim/pull/1411)
    * [Pull request #1430](https://github.com/gazebosim/gz-sim/pull/1430)

1. Delete unused `gazebo.hh.in`
    * [Pull request #1490](https://github.com/gazebosim/gz-sim/pull/1490)

1. Add repo specific issue templates
    * [Pull request #1461](https://github.com/gazebosim/gz-sim/pull/1461)

1. Added user command to set multiple entities' poses
    * [Pull request #1394](https://github.com/gazebosim/gz-sim/pull/1394)

1. Component inspector: refactor Pose3d C++ code into a separate class
    * [Pull request #1400](https://github.com/gazebosim/gz-sim/pull/1400)

1. Added more sensor properties to `scene/info` topic
    * [Pull request #1344](https://github.com/gazebosim/gz-sim/pull/1344)

1. `JointStatePublisher` publish parent, child and axis data
    * [Pull request #1345](https://github.com/gazebosim/gz-sim/pull/1345)

1. Removed unused variables in shapes plugin
    * [Pull request #1321](https://github.com/gazebosim/gz-sim/pull/1321)

1. Log an error if `JointPositionController` cannot find the joint. (citadel retarget)
    * [Pull request #1314](https://github.com/gazebosim/gz-sim/pull/1314)

1. `Buoyancy`: fix center of volume's reference frame
    * [Pull request #1302](https://github.com/gazebosim/gz-sim/pull/1302)

1. Remove `EachNew` calls from sensor PreUpdates
    * [Pull request #1281](https://github.com/gazebosim/gz-sim/pull/1281)

1. Prevent `GzScene3D` 💥 if another scene is already loaded
    * [Pull request #1294](https://github.com/gazebosim/gz-sim/pull/1294)

1. Add `project()` call to examples
    * [Pull request #1274](https://github.com/gazebosim/gz-sim/pull/1274)

1. Implement `/server_control::stop`
    * [Pull request #1240](https://github.com/gazebosim/gz-sim/pull/1240)

1. 👩‍🌾 Make depth camera tests more robust
    * [Pull request1257](https://github.com/gazebosim/gz-sim/pull/1257)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/gazebosim/gz-sim/pull/1194)

### Ignition Gazebo 3.12.0 (2021-11-11)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/ignitionrobotics/ign-gazebo/pull/1192)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/ignitionrobotics/ign-gazebo/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/ignitionrobotics/ign-gazebo/pull/847)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1149](https://github.com/ignitionrobotics/ign-gazebo/pull/1149)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/ignitionrobotics/ign-gazebo/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/ignitionrobotics/ign-gazebo/pull/1160)

1. Backport gazebo::Util::validTopic() from ign-gazebo4.
    * [Pull request #1153](https://github.com/ignitionrobotics/ign-gazebo/pull/1153)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/ignitionrobotics/ign-gazebo/pull/1147)

1. Use uint64_t for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/ignitionrobotics/ign-gazebo/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/ignitionrobotics/ign-gazebo/pull/1143)

### Ignition Gazebo 3.11.0 (2021-10-21)

1. Updates to camera video record from subt.
    * [Pull request #1117](https://github.com/ignitionrobotics/ign-gazebo/pull/1117)
1. Fix performance level test flakiness.
    * [Pull request #1129](https://github.com/ignitionrobotics/ign-gazebo/pull/1129)

### Ignition Gazebo 3.10.0 (2021-10-15)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/ignitionrobotics/ign-gazebo/pull/1083)

1. Enable new CMake policy to fix protobuf compilation
    * [Pull request #1059](https://github.com/ignitionrobotics/ign-gazebo/pull/1059)

1. Fix setting cast_shadows for visuals without material
    * [Pull request #1015](https://github.com/ignitionrobotics/ign-gazebo/pull/1015)

1. Remove duplicate XML tag in pendulum_links example world
    * [Pull request #1002](https://github.com/ignitionrobotics/ign-gazebo/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/ignitionrobotics/ign-gazebo/pull/982)

1. Improved doxygen
    * [Pull request #996](https://github.com/ignitionrobotics/ign-gazebo/pull/996)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/ignitionrobotics/ign-gazebo/pull/1098)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/ignitionrobotics/ign-gazebo/pull/1089)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/ignitionrobotics/ign-gazebo/pull/1043)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/ignitionrobotics/ign-gazebo/pull/1095)

### Ignition Gazebo 3.9.0 (2021-08-16)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/ignitionrobotics/ign-gazebo/pull/974)

1. Don't use $HOME on most tests (InternalFixture)
    * [Pull request #971](https://github.com/ignitionrobotics/ign-gazebo/pull/971)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/ignitionrobotics/ign-gazebo/pull/965)

1. Drag and drop meshes into scene
    * [Pull request #939](https://github.com/ignitionrobotics/ign-gazebo/pull/939)

1. Set protobuf_MODULE_COMPATIBLE before any find_package call
    * [Pull request #957](https://github.com/ignitionrobotics/ign-gazebo/pull/957)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/ignitionrobotics/ign-gazebo/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/ignitionrobotics/ign-gazebo/pull/724)

1. Add UserCommands Plugin.
    * [Pull request #719](https://github.com/ignitionrobotics/ign-gazebo/pull/719)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/ignitionrobotics/ign-gazebo/pull/693)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/ignitionrobotics/ign-gazebo/pull/596)

1.  Add missing IGNITION_GAZEBO_VISIBLE macros
    * [Pull request #563](https://github.com/ignitionrobotics/ign-gazebo/pull/563)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/ignitionrobotics/ign-gazebo/pull/564)

1. No install apt recommends and clear cache
    * [Pull request #423](https://github.com/ignitionrobotics/ign-gazebo/pull/423)

1. Add 25percent darker view angle icons
    * [Pull request #426](https://github.com/ignitionrobotics/ign-gazebo/pull/426)

1. Expose a test fixture helper class
    * [Pull request #926](https://github.com/ignitionrobotics/ign-gazebo/pull/926)

1. Fix logic to disable server default plugins loading
    * [Pull request #953](https://github.com/ignitionrobotics/ign-gazebo/pull/953)

1. removed unneeded plugin update
    * [Pull request #944](https://github.com/ignitionrobotics/ign-gazebo/pull/944)

1. Functions to enable velocity and acceleration checks on Link
    * [Pull request #935](https://github.com/ignitionrobotics/ign-gazebo/pull/935)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/ignitionrobotics/ign-gazebo/pull/936)

1. 3D plot GUI plugin
    * [Pull request #917](https://github.com/ignitionrobotics/ign-gazebo/pull/917)

1. Add a convenience function for getting possibly non-existing components.
    * [Pull request #629](https://github.com/ignitionrobotics/ign-gazebo/pull/629)

1. Fix topLevelModel method
    * [Pull request #600](https://github.com/ignitionrobotics/ign-gazebo/pull/600)

1. World exporter
    * [Pull request #474](https://github.com/ignitionrobotics/ign-gazebo/pull/474)

1. Fix finding PBR materials
    * [Pull request #575](https://github.com/ignitionrobotics/ign-gazebo/pull/575)

1. Handle multiple logical cameras
    * [Pull request #539](https://github.com/ignitionrobotics/ign-gazebo/pull/539)

1. Make some tests more robust
    * [Pull request #314](https://github.com/ignitionrobotics/ign-gazebo/pull/314)

1. Fix codecheck
    * [Pull request #887](https://github.com/ignitionrobotics/ign-gazebo/pull/887)

1. Hello world plugin added
    * [Pull request #699](https://github.com/ignitionrobotics/ign-gazebo/pull/699)

1. Model info CLI `ign model`
    * [Pull request #893](https://github.com/ignitionrobotics/ign-gazebo/pull/893)

1. Don't create components for entities that don't exist
    * [Pull request #927](https://github.com/ignitionrobotics/ign-gazebo/pull/927)

1. Adds Mesh Tutorial
    * [Pull request #915](https://github.com/ignitionrobotics/ign-gazebo/pull/915)

1. Fix updating GUI plugin on load
    * [Pull request #904](https://github.com/ignitionrobotics/ign-gazebo/pull/904)

1. Fix documentation for the Sensor component
    * [Pull request #898](https://github.com/ignitionrobotics/ign-gazebo/pull/898)

1. Use UINT64_MAX for kComponentTpyeIDInvalid instead of relying on underflow
    * [Pull request #889](https://github.com/ignitionrobotics/ign-gazebo/pull/889)

1. Fix mouse view control target position
    * [Pull request #879](https://github.com/ignitionrobotics/ign-gazebo/pull/879)

1. Set GUI camera pose
    * [Pull request #863](https://github.com/ignitionrobotics/ign-gazebo/pull/863)

1. Enables confirmation dialog when closing Gazebo.
    * [Pull request #850](https://github.com/ignitionrobotics/ign-gazebo/pull/850)

1. Depend on ign-rendering 3.5
    * [Pull request #867](https://github.com/ignitionrobotics/ign-gazebo/pull/867)

1. Using math::SpeedLimiter on the diff_drive controller.
    * [Pull request #833](https://github.com/ignitionrobotics/ign-gazebo/pull/833)

1. New example: get an ECM snapshot from an external program
    * [Pull request #859](https://github.com/ignitionrobotics/ign-gazebo/pull/859)

1. Fix WindEffects Plugin bug, not configuring new links
    * [Pull request #844](https://github.com/ignitionrobotics/ign-gazebo/pull/844)

1. Fix potentially flaky integration component test case
    * [Pull request #848](https://github.com/ignitionrobotics/ign-gazebo/pull/848)

1. Cleanup and alphabetize plugin headers
    * [Pull request #838](https://github.com/ignitionrobotics/ign-gazebo/pull/838)

1. Removed duplicated code with rendering::sceneFromFirstRenderEngine
    * [Pull request #819](https://github.com/ignitionrobotics/ign-gazebo/pull/819)

1. Remove unused headers in video_recoder plugin
    * [Pull request #834](https://github.com/ignitionrobotics/ign-gazebo/pull/834)

1. Use moveToHelper from ign-rendering
    * [Pull request #825](https://github.com/ignitionrobotics/ign-gazebo/pull/825)

1. Remove tools/code_check and update codecov
    * [Pull request #814](https://github.com/ignitionrobotics/ign-gazebo/pull/814)

1. Add service and GUI to configure physics parameters
    * [Pull request #536](https://github.com/ignitionrobotics/ign-gazebo/pull/536)
    * [Pull request #812](https://github.com/ignitionrobotics/ign-gazebo/pull/812)

1. Fix documentation for EntityComponentManager::EachNew
    * [Pull request #795](https://github.com/ignitionrobotics/ign-gazebo/pull/795)

1. Fix macOS build: components::Name in benchmark
    * [Pull request #784](https://github.com/ignitionrobotics/ign-gazebo/pull/784)

1. Don't store duplicate ComponentTypeId in ECM
    * [Pull request #751](https://github.com/ignitionrobotics/ign-gazebo/pull/751)

1. [TPE] Support setting individual link velocity
    * [Pull request #427](https://github.com/ignitionrobotics/ign-gazebo/pull/427)

1. 👩‍🌾 Enable Focal CI
    * [Pull request #646](https://github.com/ignitionrobotics/ign-gazebo/pull/646)

1. Update benchmark comparison instructions
    * [Pull request #766](https://github.com/ignitionrobotics/ign-gazebo/pull/766)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for compatibility with Protobuf CMake config
    * [Pull request #715](https://github.com/ignitionrobotics/ign-gazebo/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler
    * [Pull request #716](https://github.com/ignitionrobotics/ign-gazebo/pull/716)

1. Scenebroadcaster sensors
    * [Pull request #698](https://github.com/ignitionrobotics/ign-gazebo/pull/698)

1. Make it so joint state publisher is quieter
    * [Pull request #696](https://github.com/ignitionrobotics/ign-gazebo/pull/696)

### Ignition Gazebo 3.8.0 (2021-03-17)

1. Add joint position controller GUI, also enable tests for GUI plugins
    * [Pull request #534](https://github.com/ignitionrobotics/ign-gazebo/pull/534)

1. Remove visibility from headers that are not installed
    * [Pull request #665](https://github.com/ignitionrobotics/ign-gazebo/pull/665)

1. Added screenshot to toolbar
    * [Pull request #588](https://github.com/ignitionrobotics/ign-gazebo/pull/588)

1. Improve ign tool support on macOS
    * [Pull request #477](https://github.com/ignitionrobotics/ign-gazebo/pull/477)

1. change nullptr to a int ptr for qt 5.15.2 bug
    * [Pull request #527](https://github.com/ignitionrobotics/ign-gazebo/pull/527)

1. Kinetic energy monitor plugin
    * [Pull request #492](https://github.com/ignitionrobotics/ign-gazebo/pull/492)

1. Use a std::promise/std::future to avoid busy waiting the step ack messages in NetworkManagerPrimary
    * [Pull request #470](https://github.com/ignitionrobotics/ign-gazebo/pull/470)

1. clarified performer example
    * [Pull request #390](https://github.com/ignitionrobotics/ign-gazebo/pull/390)

1. Add tutorial tweaks
    * [Pull request #380](https://github.com/ignitionrobotics/ign-gazebo/pull/380)

1. Fix Qt5 warnings for using anchors
    * [Pull request #363](https://github.com/ignitionrobotics/ign-gazebo/pull/363)

1. Update codeowners
    * [Pull request #305](https://github.com/ignitionrobotics/ign-gazebo/pull/305)

1. Qt auto scale factor for HiDPI displays
    * [Pull request #291](https://github.com/ignitionrobotics/ign-gazebo/pull/291)

1. Fix yaw units
    * [Pull request #238](https://github.com/ignitionrobotics/ign-gazebo/pull/238)

1. Fixed docblock showGrid
    * [Pull request #152](https://github.com/ignitionrobotics/ign-gazebo/pull/152)

1. Fix entity tree for large worlds
    * [Pull request #673](https://github.com/ignitionrobotics/ign-gazebo/pull/673)

1. Master branch updates
    * [Pull request #672](https://github.com/ignitionrobotics/ign-gazebo/pull/672)

1. Backport #561: Use common::setenv
    * [Pull request #666](https://github.com/ignitionrobotics/ign-gazebo/pull/666)

1. Use a custom data structure to manage entity feature maps
    * [Pull request #586](https://github.com/ignitionrobotics/ign-gazebo/pull/586)

1. Limit scene broadcast publications when paused
    * [Pull request #497](https://github.com/ignitionrobotics/ign-gazebo/pull/497)

1. Fix flaky SceneBoradcaster test
    * [Pull request #641](https://github.com/ignitionrobotics/ign-gazebo/pull/641)

1. Add TF/Pose_V publisher in DiffDrive
    * [Pull request #548](https://github.com/ignitionrobotics/ign-gazebo/pull/548)

1. 👩‍🌾 Relax performance test
    * [Pull request #640](https://github.com/ignitionrobotics/ign-gazebo/pull/640)

1. 👩‍🌾 Improve velocity control test
    * [Pull request #642](https://github.com/ignitionrobotics/ign-gazebo/pull/642)

1. Add `laser_retro` support
    * [Pull request #603](https://github.com/ignitionrobotics/ign-gazebo/pull/603)

1. Fix pose of plane visual with non-default normal vector
    * [Pull request #574](https://github.com/ignitionrobotics/ign-gazebo/pull/574)

1. Add About dialog
    * [Pull request #609](https://github.com/ignitionrobotics/ign-gazebo/pull/609)

1. Make topics configurable for joint controllers
    * [Pull request #584](https://github.com/ignitionrobotics/ign-gazebo/pull/584)

1. Also use Ignition GUI render event
    * [Pull request #598](https://github.com/ignitionrobotics/ign-gazebo/pull/598)

1. Tutorial on migrating SDF files from Gazebo classic
    * [Pull request #400](https://github.com/ignitionrobotics/ign-gazebo/pull/400)

1. Visualize collisions
    * [Pull request #531](https://github.com/ignitionrobotics/ign-gazebo/pull/531)

1. Backport state update changes from pull request #486
    * [Pull request #583](https://github.com/ignitionrobotics/ign-gazebo/pull/583)

1. Publish all periodic change components in Scene Broadcaster
    * [Pull request #544](https://github.com/ignitionrobotics/ign-gazebo/pull/544)

1. added size to `ground_plane` in examples
    * [Pull request #573](https://github.com/ignitionrobotics/ign-gazebo/pull/573)

1. Parallelize State call in ECM
    * [Pull request #451](https://github.com/ignitionrobotics/ign-gazebo/pull/451)

1. Non-blocking paths request
    * [Pull request #555](https://github.com/ignitionrobotics/ign-gazebo/pull/555)

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
