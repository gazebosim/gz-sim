## Gazebo Sim 8.x

### Gazebo Sim 8.9.0 (2025-02-10)

1. Physics: Add components to request and receive Ray intersections
    * [Pull request #2514](https://github.com/gazebosim/gz-sim/pull/2514)

1. Also handle SIGTERM gracefully
    * [Pull request #2747](https://github.com/gazebosim/gz-sim/pull/2747)

1. Register MeshInertialCalculator when loading sim from an SDF string
    * [Pull request #2754](https://github.com/gazebosim/gz-sim/pull/2754)

1. Add debug msg to log auto computed inertial values
    * [Pull request #2749](https://github.com/gazebosim/gz-sim/pull/2749)

1. Reduce/Eliminate `sdf::Model` and `sdf::World` serialization warnings
    * [Pull request #2742](https://github.com/gazebosim/gz-sim/pull/2742)

1. Fix illegal anchor warnings
    * [Pull request #2741](https://github.com/gazebosim/gz-sim/pull/2741)

1. Fix mesh import filters not displaying correctly on KDE #2731 (#2732)
    * [Pull request #2736](https://github.com/gazebosim/gz-sim/pull/2736)

### Gazebo Sim 8.8.0 (2025-01-16)

1. Add parameter for adjusting current sign in battery plugin
    * [Pull request #2696](https://github.com/gazebosim/gz-sim/pull/2696)

1. Extend shapes plugin width to include the ellipsoid button
    * [Pull request #2699](https://github.com/gazebosim/gz-sim/pull/2699)

1. Use same FP limits for TrackedVehicle to avoid self-moving
    * [Pull request #2651](https://github.com/gazebosim/gz-sim/pull/2651)

1. Link.hh: add Sensor accessor APIs
    * [Pull request #2693](https://github.com/gazebosim/gz-sim/pull/2693)

1. Improve load times by skipping serialization of entities when unnecessary.
    * [Pull request #2683](https://github.com/gazebosim/gz-sim/pull/2683)

1. Prepend to `PYTHONPATH` in tests
    * [Pull request #2681](https://github.com/gazebosim/gz-sim/pull/2681)

1. Fix crash in `OpticalTactilePlugin` by checking for valid visualize pointer
    * [Pull request #2674](https://github.com/gazebosim/gz-sim/pull/2674)

### Gazebo Sim 8.7.0 (2024-11-08)

1. Fix crash when multicopter motor system is attached to an empty model
    * [Pull request #2653](https://github.com/gazebosim/gz-sim/pull/2653)

1. Backport SystemConfigurePriority to harmonic
    * [Pull request #2661](https://github.com/gazebosim/gz-sim/pull/2661)

1. Break out server constructor
    * [Pull request #2638](https://github.com/gazebosim/gz-sim/pull/2638)

1. Add check for valid world pose cmd values
    * [Pull request #2640](https://github.com/gazebosim/gz-sim/pull/2640)

1. Enabling Global Illumination (GI VCT) for sensors in SDF
    * [Pull request #2550](https://github.com/gazebosim/gz-sim/pull/2550)

1. Fixed typos in three places
    * [Pull request #2599](https://github.com/gazebosim/gz-sim/pull/2599)

1. Add point cloud data size sanity check to fix crash
    * [Pull request #2549](https://github.com/gazebosim/gz-sim/pull/2549)

1. Corrected command to move the kth_freeflyer spacecraft testbed
    * [Pull request #2565](https://github.com/gazebosim/gz-sim/pull/2565)

1. Backport nonbreaking changes from ionic
    * [Pull request #2552](https://github.com/gazebosim/gz-sim/pull/2552)

1. clang-tidy fixes: use empty(), fix includes
    * [Pull request #2548](https://github.com/gazebosim/gz-sim/pull/2548)

1. Check ranges before access
    * [Pull request #2540](https://github.com/gazebosim/gz-sim/pull/2540)

1. Fixed Odometry Publisher Angular Velocity Singularity in 3D
    * [Pull request #2348](https://github.com/gazebosim/gz-sim/pull/2348)

1. Revert behavior change introduced in #2452
    * [Pull request #2527](https://github.com/gazebosim/gz-sim/pull/2527)

1. Specify System::PreUpdate, Update execution order
    * [Pull request #2487](https://github.com/gazebosim/gz-sim/pull/2487)

1. Force Qt to use xcb plugin on Wayland
    * [Pull request #2526](https://github.com/gazebosim/gz-sim/pull/2526)

1. Remove unused variable in thruster system
    * [Pull request #2524](https://github.com/gazebosim/gz-sim/pull/2524)

1. Make sure steering joints exist before updating velocity / odometry in AckermannSteering plugin
    * [Pull request #2521](https://github.com/gazebosim/gz-sim/pull/2521)

1. Fix ResourceSpawner
    * [Pull request #2490](https://github.com/gazebosim/gz-sim/pull/2490)

1. gui_system_plugin: clarify description in README
    * [Pull request #2253](https://github.com/gazebosim/gz-sim/pull/2253)

1. Fix adding system to non-existent entity
    * [Pull request #2516](https://github.com/gazebosim/gz-sim/pull/2516)

1. Checking linkEnity is empty
    * [Pull request #2509](https://github.com/gazebosim/gz-sim/pull/2509)

1. Initialize threadsNeedCleanUp
    * [Pull request #2503](https://github.com/gazebosim/gz-sim/pull/2503)

1. Added support for spacecraft thrusters
    * [Pull request #2431](https://github.com/gazebosim/gz-sim/pull/2431)

1. Disable rendering tests that are failing on github actions
    * [Pull request #2480](https://github.com/gazebosim/gz-sim/pull/2480)

1. Consolidate entity creation
    * [Pull request #2452](https://github.com/gazebosim/gz-sim/pull/2452)

### Gazebo Sim 8.6.0 (2024-07-25)

1. Fix error resolving gazebo classic material when loading world
    * [Pull request #2492](https://github.com/gazebosim/gz-sim/pull/2492)

1. Remove systems if their parent entity is removed
    * [Pull request #2232](https://github.com/gazebosim/gz-sim/pull/2232)

1. Fix warnings generated by NetworkConfigTest
    * [Pull request #2469](https://github.com/gazebosim/gz-sim/pull/2469)

1. Fix lidar visualization when `gz_frame_id` is specified
    * [Pull request #2481](https://github.com/gazebosim/gz-sim/pull/2481)

1. Backport convex decomposition visualization
    * [Pull request #2454](https://github.com/gazebosim/gz-sim/pull/2454)

1. Add UserCommands plugin to GPU lidar sensor example
    * [Pull request #2479](https://github.com/gazebosim/gz-sim/pull/2479)

1. Check if any entity actually has a ContactSensorData component before calling GetContactsFromLastStep
    * [Pull request #2474](https://github.com/gazebosim/gz-sim/pull/2474)

1. Enable tests on macOS
    * [Pull request #2468](https://github.com/gazebosim/gz-sim/pull/2468)

1. Update description of reset_sensors test
    * [Pull request #2467](https://github.com/gazebosim/gz-sim/pull/2467)

1. Magnetometer: correct field calculation
    * [Pull request #2460](https://github.com/gazebosim/gz-sim/pull/2460)

1. Address a couple of todos in Conversion.cc
    * [Pull request #2461](https://github.com/gazebosim/gz-sim/pull/2461)

1. Correct name of sensor in warning message
    * [Pull request #2457](https://github.com/gazebosim/gz-sim/pull/2457)

1. Set max contacts for collision pairs
    * [Pull request #2270](https://github.com/gazebosim/gz-sim/pull/2270)

1. Add GravityEnabled boolean component
    * [Pull request #2451](https://github.com/gazebosim/gz-sim/pull/2451)

1. Add support for no gravity link
    * [Pull request #2398](https://github.com/gazebosim/gz-sim/pull/2398)

1. Handle sdf::Geometry::EMPTY in conversions
    * [Pull request #2430](https://github.com/gazebosim/gz-sim/pull/2430)

1. Use topicFromScopedName in a few systems
    * [Pull request #2427](https://github.com/gazebosim/gz-sim/pull/2427)

1. Fix typo in a comment
    * [Pull request #2429](https://github.com/gazebosim/gz-sim/pull/2429)

### Gazebo Sim 8.5.0 (2024-06-26)

1. Backport: Adding cone primitives
    * [Pull request #2404](https://github.com/gazebosim/gz-sim/pull/2404)

1. Permit to run gz sim -g on Windows
    * [Pull request #2382](https://github.com/gazebosim/gz-sim/pull/2382)

1. Parse voxel resolution SDF param when decomposing meshes
    * [Pull request #2445](https://github.com/gazebosim/gz-sim/pull/2445)

1. Fix model command api test
    * [Pull request #2444](https://github.com/gazebosim/gz-sim/pull/2444)

1. Add tutorial for using the Pose component
    * [Pull request #2219](https://github.com/gazebosim/gz-sim/pull/2219)

1. Do not update sensors if it a triggered sensor
    * [Pull request #2443](https://github.com/gazebosim/gz-sim/pull/2443)

### Gazebo Sim 8.4.0 (2024-06-12)

1. Add pause run tutorial
    * [Pull request #2383](https://github.com/gazebosim/gz-sim/pull/2383)

1. Fix warning message to show precise jump back in time duration
    * [Pull request #2435](https://github.com/gazebosim/gz-sim/pull/2435)

1. Optimize rendering sensor pose updates
    * [Pull request #2425](https://github.com/gazebosim/gz-sim/pull/2425)

1. Remove a few extra zeros from some sdf files
    * [Pull request #2426](https://github.com/gazebosim/gz-sim/pull/2426)

1. Use VERSION_GREATER_EQUAL in cmake logic
    * [Pull request #2418](https://github.com/gazebosim/gz-sim/pull/2418)

1. Support mesh optimization when using AttachMeshShapeFeature
    * [Pull request #2417](https://github.com/gazebosim/gz-sim/pull/2417)

1. Rephrase cmake comment about CMP0077
    * [Pull request #2419](https://github.com/gazebosim/gz-sim/pull/2419)

1. Fix CMake warnings in Noble
    * [Pull request #2397](https://github.com/gazebosim/gz-sim/pull/2397)

1. Update sensors with pending trigger immediately in Sensors system
    * [Pull request #2408](https://github.com/gazebosim/gz-sim/pull/2408)

1. Add missing algorithm include
    * [Pull request #2414](https://github.com/gazebosim/gz-sim/pull/2414)

1. Add Track and Follow options in gui EntityContextMenu
    * [Pull request #2402](https://github.com/gazebosim/gz-sim/pull/2402)

1. ForceTorque system: improve readability
    * [Pull request #2403](https://github.com/gazebosim/gz-sim/pull/2403)

1. LTA Dynamics System
    * [Pull request #2241](https://github.com/gazebosim/gz-sim/pull/2241)

1. Remove Empty Test File
    * [Pull request #2396](https://github.com/gazebosim/gz-sim/pull/2396)

1. Fix GCC/CMake warnings for Noble
    * [Pull request #2375](https://github.com/gazebosim/gz-sim/pull/2375)

1. Fix warn unused variable in test
    * [Pull request #2388](https://github.com/gazebosim/gz-sim/pull/2388)

1. Fix name of gz-fuel_tools in package.xml
    * [Pull request #2386](https://github.com/gazebosim/gz-sim/pull/2386)

1. Add package.xml
    * [Pull request #2337](https://github.com/gazebosim/gz-sim/pull/2337)

1. Fix namespace and class links in documentation references that use namespace `gz`
    * [Pull request #2385](https://github.com/gazebosim/gz-sim/pull/2385)

1. Fix ModelPhotoShootTest test failures
    * [Pull request #2294](https://github.com/gazebosim/gz-sim/pull/2294)

1. Enable StoreResolvedURIs when loading SDF
    * [Pull request #2349](https://github.com/gazebosim/gz-sim/pull/2349)

1. Drop python3-disttutils from apt packages files
    * [Pull request #2374](https://github.com/gazebosim/gz-sim/pull/2374)

1. Added example world for `DopplerVelocityLogSystem`
    * [Pull request #2373](https://github.com/gazebosim/gz-sim/pull/2373)

1. Fix Gazebo/White and refactored MaterialParser
    * [Pull request #2302](https://github.com/gazebosim/gz-sim/pull/2302)

1. Support for Gazebo materials
    * [Pull request #2269](https://github.com/gazebosim/gz-sim/pull/2269)

### Gazebo Sim 8.3.0 (2024-04-11)

1. Use relative install paths for plugin shared libraries and gz-tools data
    * [Pull request #2358](https://github.com/gazebosim/gz-sim/pull/2358)

1. Use `steer_p_gain` for UpdateVelocity steer joint speed
    * [Pull request #2355](https://github.com/gazebosim/gz-sim/pull/2355)

1. Fix TriggeredPublisher test
    * [Pull request #2354](https://github.com/gazebosim/gz-sim/pull/2354)

1. Use SetComponentData to simplify code and improve coverage
    * [Pull request #2360](https://github.com/gazebosim/gz-sim/pull/2360)

1. Remove unnecessary sleep
    * [Pull request #2357](https://github.com/gazebosim/gz-sim/pull/2357)

1. Fixed undefined behavior in thruster.cc
    * [Pull request #2350](https://github.com/gazebosim/gz-sim/pull/2350)

1. Added mutex to protect stored time variables
    * [Pull request #2345](https://github.com/gazebosim/gz-sim/pull/2345)

1. Fixed turning error in ackermann steering
    * [Pull request #2342](https://github.com/gazebosim/gz-sim/pull/2342)

1. Check null mesh
    * [Pull request #2341](https://github.com/gazebosim/gz-sim/pull/2341)

1. Publish step size in world stats topic
    * [Pull request #2340](https://github.com/gazebosim/gz-sim/pull/2340)

### Gazebo Sim 8.2.0 (2024-03-14)

1. Add reference to joint_controller.md tutorial.
    * [Pull request #2333](https://github.com/gazebosim/gz-sim/pull/2333)

1. Fix wget in maritime tutorials
    * [Pull request #2330](https://github.com/gazebosim/gz-sim/pull/2330)

1. Add entity and sdf parameters to Server's AddSystem interface
    * [Pull request #2324](https://github.com/gazebosim/gz-sim/pull/2324)

1. Add entity validation to OdometryPublisher
    * [Pull request #2326](https://github.com/gazebosim/gz-sim/pull/2326)

1. Fix typo in Joint.hh
    * [Pull request #2310](https://github.com/gazebosim/gz-sim/pull/2310)

### Gazebo Sim 8.1.0 (2024-02-06)

1. Add tutorial for using components in systems
    * [Pull request #2207](https://github.com/gazebosim/gz-sim/pull/2207)

1. Light entity match SDF boolean for UserCommands.
    * [Pull request #2295](https://github.com/gazebosim/gz-sim/pull/2295)

1. Explicitly check SKIP_PYBIND11 for python bindings
    * [Pull request #2298](https://github.com/gazebosim/gz-sim/pull/2298)

1. Fix `INTEGRATION_save_world` on windows
    * [Pull request #2300](https://github.com/gazebosim/gz-sim/pull/2300)

1. Change an entities visual material color by topic.
    * [Pull request #2286](https://github.com/gazebosim/gz-sim/pull/2286)

1. Fix ModelPhotoShootTest test failures
    * [Pull request #2294](https://github.com/gazebosim/gz-sim/pull/2294)

1. Support for Gazebo materials
    * [Pull request #2269](https://github.com/gazebosim/gz-sim/pull/2269)
    * [Pull request #2302](https://github.com/gazebosim/gz-sim/pull/2302)

1. Added tutorial for Gazebo joint controller plugin
    * [Pull request #2263](https://github.com/gazebosim/gz-sim/pull/2263)

1. Lift Drag Bug Fix
    * [Pull request #2189](https://github.com/gazebosim/gz-sim/pull/2189)
    * [Pull request #2272](https://github.com/gazebosim/gz-sim/pull/2272)
    * [Pull request #2273](https://github.com/gazebosim/gz-sim/pull/2273)
    * [Issue #2188](https://github.com/gazebosim/gz-sim/issues/2188)

1. Fix URL in hydrodynamics tutorial
    * [Pull request #2280](https://github.com/gazebosim/gz-sim/pull/2280)

1. Install the Python libs to system site-packages instead of root
    * [Pull request #2274](https://github.com/gazebosim/gz-sim/pull/2274)

1. Maritime tutorials 💧
    * [Pull request #2260](https://github.com/gazebosim/gz-sim/pull/2260)
    * [Pull request #2259](https://github.com/gazebosim/gz-sim/pull/2259)
    * [Pull request #2258](https://github.com/gazebosim/gz-sim/pull/2258)
    * [Pull request #2257](https://github.com/gazebosim/gz-sim/pull/2257)

1. Update CI badges in README
    * [Pull request #2271](https://github.com/gazebosim/gz-sim/pull/2271)

1. Fix incorrect light direction in tunnel.sdf example
    * [Pull request #2264](https://github.com/gazebosim/gz-sim/pull/2264)

1. Allow removal of model that has joint_position_controller plugin.
    * [Pull request #2252](https://github.com/gazebosim/gz-sim/pull/2252)

1. Fix DLL linkage/visibility issues
    * [Pull request #2254](https://github.com/gazebosim/gz-sim/pull/2254)

1. `mecanum_drive`: use mesh wheels in example world
    * [Pull request #2250](https://github.com/gazebosim/gz-sim/pull/2250)

1. Disable distortion camera test on Linux
    * [Pull request #2247](https://github.com/gazebosim/gz-sim/pull/2247)

1. `environment_preload`: fix windows compiler warnings
    * [Pull request #2246](https://github.com/gazebosim/gz-sim/pull/2246)

1. EnvironmentPreload: ignerr -> gzerr
    * [Pull request #2245](https://github.com/gazebosim/gz-sim/pull/2245)

1. Update friction parameters for skid steer example
    * [Pull request #2235](https://github.com/gazebosim/gz-sim/pull/2235)

1. Fixed Centre of Mass and Inertia Matrix Calculation Bug `MeshInertiaCalculator::CalculateMassProperties()` function
    * [Pull request #2182](https://github.com/gazebosim/gz-sim/pull/2182)

1. Backport #2231: Use sdf FindElement API to avoid `const_cast`
    * [Pull request #2236](https://github.com/gazebosim/gz-sim/pull/2236)

1. Add libpython3-dev CI dependency
    * [Pull request #2233](https://github.com/gazebosim/gz-sim/pull/2233)

1.  Standardize Doxygen parameter formatting for systems
    * [Pull request #2212](https://github.com/gazebosim/gz-sim/pull/2212)
    * [Pull request #2183](https://github.com/gazebosim/gz-sim/pull/2183)

1. Use `GZ_PI` instead of `M_PI` to fix windows builds
    * [Pull request #2230](https://github.com/gazebosim/gz-sim/pull/2230)

1. Add note about elevator example
    * [Pull request #2227](https://github.com/gazebosim/gz-sim/pull/2227)

1. Porting Advanced Lift Drag Plugin to Gazebo
    * [Pull request #2185](https://github.com/gazebosim/gz-sim/pull/2185)
    * [Pull request #2226](https://github.com/gazebosim/gz-sim/pull/2226)

1. Fix for sensor pointer null when navsat plugin in included in sdf
    * [Pull request #2176](https://github.com/gazebosim/gz-sim/pull/2176)

1. Implements a method to get the link inertia
    * [Pull request #2218](https://github.com/gazebosim/gz-sim/pull/2218)

1. Fix sensors system parallel updates
    * [Pull request #2201](https://github.com/gazebosim/gz-sim/pull/2201)

1. Fix custom_sensor_system example
    * [Pull request #2208](https://github.com/gazebosim/gz-sim/pull/2208)

1. Relax pose check in actor no mesh test
    * [Pull request #2196](https://github.com/gazebosim/gz-sim/pull/2196)

1. backport component inspector Vector3d width fix
    * [Pull request #2195](https://github.com/gazebosim/gz-sim/pull/2195)

1. fix INTEGRATION_save_world's SdfGeneratorFixture.ModelWithNestedIncludes test
    * [Pull request #2197](https://github.com/gazebosim/gz-sim/pull/2197)

1. Support specifying the name of light associated with lens flares
    * [Pull request #2172](https://github.com/gazebosim/gz-sim/pull/2172)

1. Bump Fuel model version in test
    * [Pull request #2190](https://github.com/gazebosim/gz-sim/pull/2190)

1. Fix environment system loading mechanism
    * [Pull request #1842](https://github.com/gazebosim/gz-sim/pull/1842)

1. Infrastructure
    * [Pull request #2187](https://github.com/gazebosim/gz-sim/pull/2187)
    * [Pull request #2237](https://github.com/gazebosim/gz-sim/pull/2237)
    * [Pull request #2222](https://github.com/gazebosim/gz-sim/pull/2222)

### Gazebo Sim 8.0.0 (2023-09-29)

1. TouchPlugin: Reset the plugin with the initial values
    * [Pull request #2132](https://github.com/gazebosim/gz-sim/pull/2132)

1. Fix another deadlock in sensors system
    * [Pull request #2141](https://github.com/gazebosim/gz-sim/pull/2141)

1. Documentation fixes
    * [Pull request #2157](https://github.com/gazebosim/gz-sim/pull/2157)
    * [Pull request #2150](https://github.com/gazebosim/gz-sim/pull/2150)
    * [Pull request #2148](https://github.com/gazebosim/gz-sim/pull/2148)
    * [Pull request #2147](https://github.com/gazebosim/gz-sim/pull/2147)
    * [Pull request #2143](https://github.com/gazebosim/gz-sim/pull/2143)
    * [Pull request #2133](https://github.com/gazebosim/gz-sim/pull/2133)
    * [Pull request #2130](https://github.com/gazebosim/gz-sim/pull/2130)
    * [Pull request #2128](https://github.com/gazebosim/gz-sim/pull/2128)
    * [Pull request #2124](https://github.com/gazebosim/gz-sim/pull/2124)
    * [Pull request #2114](https://github.com/gazebosim/gz-sim/pull/2114)
    * [Pull request #2107](https://github.com/gazebosim/gz-sim/pull/2107)

1. Fix Examples
    * [Pull request #2151](https://github.com/gazebosim/gz-sim/pull/2151)
    * [Pull request #2149](https://github.com/gazebosim/gz-sim/pull/2149)
    * [Pull request #2145](https://github.com/gazebosim/gz-sim/pull/2145)
    * [Pull request #2144](https://github.com/gazebosim/gz-sim/pull/2144)
    * [Pull request #2129](https://github.com/gazebosim/gz-sim/pull/2129)
    * [Pull request #2127](https://github.com/gazebosim/gz-sim/pull/2127)
    * [Pull request #2123](https://github.com/gazebosim/gz-sim/pull/2123)
    * [Pull request #2122](https://github.com/gazebosim/gz-sim/pull/2122)
    * [Pull request #2111](https://github.com/gazebosim/gz-sim/pull/2111)

1. Load transform control and select entities plugins in thermal camera world
    * [Pull request #2139](https://github.com/gazebosim/gz-sim/pull/2139)

1. Fixed invalid service names
    * [Pull request #2121](https://github.com/gazebosim/gz-sim/pull/2121)

1. Add automatic moment of inertia calculation for meshes
    * [Pull request #2171](https://github.com/gazebosim/gz-sim/pull/2171)
    * [Pull request #2166](https://github.com/gazebosim/gz-sim/pull/2166)
    * [Pull request #2119](https://github.com/gazebosim/gz-sim/pull/2119)
    * [Pull request #2105](https://github.com/gazebosim/gz-sim/pull/2105)
    * [Pull request #2061](https://github.com/gazebosim/gz-sim/pull/2061)

1. ign -> gz
    * [Pull request #2055](https://github.com/gazebosim/gz-sim/pull/2055)

1. Adds python demo examples
    * [Pull request #2044](https://github.com/gazebosim/gz-sim/pull/2044)

1. Add support for writing systems in Python
    * [Pull request #2081](https://github.com/gazebosim/gz-sim/pull/2081)

1. Apply mimic constraint to joints (only Bullet-featherstone)
    * [Pull request #1838](https://github.com/gazebosim/gz-sim/pull/1838)

1. Fix rendering tests
    * [Pull request #2086](https://github.com/gazebosim/gz-sim/pull/2086)

1. Make systems and tests include headers they use
    * [Pull request #2100](https://github.com/gazebosim/gz-sim/pull/2100)

1. Adds Python bindings for the Actor, Joint, Light, Link, Model, Sensor, World convenience class
    * [Pull request #2043](https://github.com/gazebosim/gz-sim/pull/2043)
    * [Pull request #2042](https://github.com/gazebosim/gz-sim/pull/2042)
    * [Pull request #2041](https://github.com/gazebosim/gz-sim/pull/2041)
    * [Pull request #2040](https://github.com/gazebosim/gz-sim/pull/2040)
    * [Pull request #2039](https://github.com/gazebosim/gz-sim/pull/2039)
    * [Pull request #2036](https://github.com/gazebosim/gz-sim/pull/2036)
    * [Pull request #2035](https://github.com/gazebosim/gz-sim/pull/2035)

1. Add version number to gz.common python binding
    * [Pull request #2093](https://github.com/gazebosim/gz-sim/pull/2093)

1. Infrastructure
    * [Pull request #2046](https://github.com/gazebosim/gz-sim/pull/2046)

1. Bumps in harmonic : sdformat14, gz-physics6, gz-sensors8, gz-gui8, gz-rendering8, gz-transport13, gz-msgs10, gz-fuel-tools9
    * [Pull request #2062](https://github.com/gazebosim/gz-sim/pull/2062)
    * [Pull request #1892](https://github.com/gazebosim/gz-sim/pull/1892)
    * [Pull request #1837](https://github.com/gazebosim/gz-sim/pull/1837)

1. Use new sky cubemap, instead of header
    * [Pull request #2060](https://github.com/gazebosim/gz-sim/pull/2060)

1. Remove deprecations and address some todos for Harmonic
    * [Pull request #2054](https://github.com/gazebosim/gz-sim/pull/2054)
    * [Pull request #2053](https://github.com/gazebosim/gz-sim/pull/2053)

1. Use ogre2 in wide angle camera and lens flares worlds
    * [Pull request #2063](https://github.com/gazebosim/gz-sim/pull/2063)

1. Use new API for creating projector
    * [Pull request #2064](https://github.com/gazebosim/gz-sim/pull/2064)

1. Fix const-correctness of the `Model::JointByName` and `Model::LinkByName` APIs
    * [Pull request #2059](https://github.com/gazebosim/gz-sim/pull/2059)

1. Change type of `Component::typeName` and address outstanding todos
    * [Pull request #2049](https://github.com/gazebosim/gz-sim/pull/2049)

1. Add Lens Flare System
    * [Pull request #1933](https://github.com/gazebosim/gz-sim/pull/1933)

1. Fix TopicInfo deprecation warnings in Harmonic
    * [Pull request #1922](https://github.com/gazebosim/gz-sim/pull/1922)

1. Add DopplerVelocityLogSystem plugin
    * [Pull request #1804](https://github.com/gazebosim/gz-sim/pull/1804)

1. GUI for Global Illumination (VCT / CI VCT)
    * [Pull request #1597](https://github.com/gazebosim/gz-sim/pull/1597)

1. Add CLI to switch to Vulkan & Metal backends
    * [Pull request #1735](https://github.com/gazebosim/gz-sim/pull/1735)

1. Remove deprecations for main/gz-sim8
    * [Pull request #1783](https://github.com/gazebosim/gz-sim/pull/1783)

1. Acoustic comms plugin
    * [Pull request #1608](https://github.com/gazebosim/gz-sim/pull/1608)

1. Set seed value using CLI
    * [Pull request #1618](https://github.com/gazebosim/gz-sim/pull/1618)

1. ⬆️  Bump main to 8.0.0~pre1
    * [Pull request #1640](https://github.com/gazebosim/gz-sim/pull/1640)

## Gazebo Sim 7.x

### Gazebo Sim 7.8.0 (2024-07-22)

1. Added support for spacecraft thrusters
    * [Pull request #2431](https://github.com/gazebosim/gz-sim/pull/2431)

1. Disable rendering tests that are failing on github actions
    * [Pull request #2480](https://github.com/gazebosim/gz-sim/pull/2480)

1. Consolidate entity creation.
    * [Pull request #2452](https://github.com/gazebosim/gz-sim/pull/2452)

1. Set max contacts for collision pairs
    * [Pull request #2270](https://github.com/gazebosim/gz-sim/pull/2270)

1. Add GravityEnabled boolean component
    * [Pull request #2451](https://github.com/gazebosim/gz-sim/pull/2451)

1. Add support for no gravity link
    * [Pull request #2398](https://github.com/gazebosim/gz-sim/pull/2398)

1. Handle sdf::Geometry::EMPTY in conversions
    * [Pull request #2430](https://github.com/gazebosim/gz-sim/pull/2430)

1. Use topicFromScopedName in a few systems
    * [Pull request #2427](https://github.com/gazebosim/gz-sim/pull/2427)

1. Fix typo in a comment
    * [Pull request #2429](https://github.com/gazebosim/gz-sim/pull/2429)

1. Remove a few extra zeros from some sdf files
    * [Pull request #2426](https://github.com/gazebosim/gz-sim/pull/2426)

1. Use VERSION_GREATER_EQUAL in cmake logic
    * [Pull request #2418](https://github.com/gazebosim/gz-sim/pull/2418)

1. Rephrase cmake comment about CMP0077
    * [Pull request #2419](https://github.com/gazebosim/gz-sim/pull/2419)

1. ForceTorque system: improve readability
    * [Pull request #2403](https://github.com/gazebosim/gz-sim/pull/2403)

1. LTA Dynamics System
    * [Pull request #2241](https://github.com/gazebosim/gz-sim/pull/2241)

1. Fix namespace and class links in documentation references that use namespace `gz`
    * [Pull request #2385](https://github.com/gazebosim/gz-sim/pull/2385)

1. Fix ModelPhotoShootTest test failures
    * [Pull request #2294](https://github.com/gazebosim/gz-sim/pull/2294)

1. update sdf version
    * [Pull request #2313](https://github.com/gazebosim/gz-sim/pull/2313)

1. Fix Gazebo/White and refactored MaterialParser
    * [Pull request #2302](https://github.com/gazebosim/gz-sim/pull/2302)

1. Support for Gazebo materials
    * [Pull request #2269](https://github.com/gazebosim/gz-sim/pull/2269)

### Gazebo Sim 7.7.0 (2024-01-17)

1. Allow using plugin file names and environment variables compatible with Garden and later
    * [Pull request #2275](https://github.com/gazebosim/gz-sim/pull/2275)

1. Added tutorial for Gazebo joint controller plugin
    * [Pull request #2263](https://github.com/gazebosim/gz-sim/pull/2263)

1. Fix incorrect light direction in tunnel.sdf example
    * [Pull request #2264](https://github.com/gazebosim/gz-sim/pull/2264)

1. Fix DLL linkage/visibility issues
    * [Pull request #2254](https://github.com/gazebosim/gz-sim/pull/2254)

1. mecanum_drive: use mesh wheels in example world
    * [Pull request #2250](https://github.com/gazebosim/gz-sim/pull/2250)

1. environment_preload: fix windows compiler warnings
    * [Pull request #2246](https://github.com/gazebosim/gz-sim/pull/2246)

1. EnvironmentPreload: ignerr -> gzerr
    * [Pull request #2245](https://github.com/gazebosim/gz-sim/pull/2245)

1. Update friction parameters for skid steer example
    * [Pull request #2235](https://github.com/gazebosim/gz-sim/pull/2235)

1. Use sdf FindElement API to avoid const_cast
    * [Pull request #2236](https://github.com/gazebosim/gz-sim/pull/2236)

1. Use `GZ_PI` instead of `M_PI` to fix windows builds
    * [Pull request #2230](https://github.com/gazebosim/gz-sim/pull/2230)

1. Add note about elevator example
    * [Pull request #2227](https://github.com/gazebosim/gz-sim/pull/2227)

1. Porting Advanced Lift Drag Plugin to Gazebo
    * [Pull request #2185](https://github.com/gazebosim/gz-sim/pull/2185)
    * [Pull request #2226](https://github.com/gazebosim/gz-sim/pull/2226)

1. Fix macOS test failures by registering components in the core library
    * [Pull request #2220](https://github.com/gazebosim/gz-sim/pull/2220)

1. Fix for sensor pointer null when navsat plugin in included in sdf
    * [Pull request #2176](https://github.com/gazebosim/gz-sim/pull/2176)

1. Fix another deadlock in sensors system
    * [Pull request #2200](https://github.com/gazebosim/gz-sim/pull/2200)

1. Fix sensors system parallel updates
    * [Pull request #2201](https://github.com/gazebosim/gz-sim/pull/2201)

1. Relax pose check in actor no mesh test
    * [Pull request #2196](https://github.com/gazebosim/gz-sim/pull/2196)

1. backport component inspector Vector3d width fix
    * [Pull request #2195](https://github.com/gazebosim/gz-sim/pull/2195)

1. fix INTEGRATION_save_world's SdfGeneratorFixture.ModelWithNestedIncludes test
    * [Pull request #2197](https://github.com/gazebosim/gz-sim/pull/2197)

1. Lift Drag Bug Fix
    * [Pull request #2189](https://github.com/gazebosim/gz-sim/pull/2189)
    * [Pull request #2272](https://github.com/gazebosim/gz-sim/pull/2272)
    * [Pull request #2273](https://github.com/gazebosim/gz-sim/pull/2273)
    * [Issue #2188](https://github.com/gazebosim/gz-sim/issues/2188)

1. Bump Fuel model version in test
    * [Pull request #2190](https://github.com/gazebosim/gz-sim/pull/2190)

1. Fix enviroment system loading mechanism
    * [Pull request #1842](https://github.com/gazebosim/gz-sim/pull/1842)

1. Infrastructure
    * [Pull request #2237](https://github.com/gazebosim/gz-sim/pull/2237)
    * [Pull request #2222](https://github.com/gazebosim/gz-sim/pull/2222)


### Gazebo Sim 7.6.0 (2023-09-26)

1. Documentation updates
    * [Pull request #2169](https://github.com/gazebosim/gz-sim/pull/2169)
    * [Pull request #2135](https://github.com/gazebosim/gz-sim/pull/2135)
    * [Pull request #2120](https://github.com/gazebosim/gz-sim/pull/2120)
    * [Pull request #2116](https://github.com/gazebosim/gz-sim/pull/2116)
    * [Pull request #2115](https://github.com/gazebosim/gz-sim/pull/2115)
    * [Pull request #2108](https://github.com/gazebosim/gz-sim/pull/2108)
    * [Pull request #2067](https://github.com/gazebosim/gz-sim/pull/2067)
    * [Pull request #1996](https://github.com/gazebosim/gz-sim/pull/1996)

1. Backport reset button fix
    * [Pull request #2159](https://github.com/gazebosim/gz-sim/pull/2159)

1. Fix SDFormat xml output of sdf_exporter
    * [Pull request #2156](https://github.com/gazebosim/gz-sim/pull/2156)

1. Fix duplicate entries in joint position controller GUI plugin
    * [Pull request #2101](https://github.com/gazebosim/gz-sim/pull/2101)

1. Use default physics engine in example worlds
    * [Pull request #2134](https://github.com/gazebosim/gz-sim/pull/2134)

1. Fix move to model
    * [Pull request #2126](https://github.com/gazebosim/gz-sim/pull/2126)

1. Remove GZ_PHYSICS_ENGINE_INSTALL_DIR deprecation warnings
    * [Pull request #2106](https://github.com/gazebosim/gz-sim/pull/2106)

1. Remove forward-ported restriction on model loading
    * [Pull request #2104](https://github.com/gazebosim/gz-sim/pull/2104)

1. Odometry topic for the track controller system
    * [Pull request #2021](https://github.com/gazebosim/gz-sim/pull/2021)

1. Add tutorials for ApplyForceTorque and MouseDrag plugins
    * [Pull request #2083](https://github.com/gazebosim/gz-sim/pull/2083)

1. Prevent crash when viewing heightmap collision
    * [Pull request #2097](https://github.com/gazebosim/gz-sim/pull/2097)

1. Force offset and vector magnitude support in ApplyForceTorque
    * [Pull request #2056](https://github.com/gazebosim/gz-sim/pull/2056)

1. Fix plugin conversion error message
    * [Pull request #2094](https://github.com/gazebosim/gz-sim/pull/2094)

1. Visualization tools for the ApplyForceTorque GUI plugin
    * [Pull request #2051](https://github.com/gazebosim/gz-sim/pull/2051)

1. Improve documentation on how to replace Scene3D plugin
    * [Pull request #1698](https://github.com/gazebosim/gz-sim/pull/1698)

1. Configurable stiffnesses in MouseDrag
    * [Pull request #2057](https://github.com/gazebosim/gz-sim/pull/2057)

1. Infrastructure
    * [Pull request #2075](https://github.com/gazebosim/gz-sim/pull/2075)
    * [Pull request #2066](https://github.com/gazebosim/gz-sim/pull/2066)
    * [Pull request #2012](https://github.com/gazebosim/gz-sim/pull/2012)
    * [Pull request #1988](https://github.com/gazebosim/gz-sim/pull/1988)

1. Add new MouseDrag plugin
    * [Pull request #2038](https://github.com/gazebosim/gz-sim/pull/2038)

1. Relax scene init check in visualize lidar gui plugin
    * [Pull request #2077](https://github.com/gazebosim/gz-sim/pull/2077)

1. Add force offset support to ApplyLinkWrench system and to Link API
    * [Pull request #2026](https://github.com/gazebosim/gz-sim/pull/2026)

1. Proposal to add deadband to thruster
    * [Pull request #1927](https://github.com/gazebosim/gz-sim/pull/1927)

1. Avoid nullptr dereference if TouchPlugin is not attached to a model entity.
    * [Pull request #2069](https://github.com/gazebosim/gz-sim/pull/2069)

1. Remove unnecessary headers to fix ABI checker
    * [Pull request #2070](https://github.com/gazebosim/gz-sim/pull/2070)

1. Fix Joint Position Controller Behaviour Described in #1997
    * [Pull request #2001](https://github.com/gazebosim/gz-sim/pull/2001)

1. Include contact force, normal, and depth in contact message
    * [Pull request #2050](https://github.com/gazebosim/gz-sim/pull/2050)

1. Use sdf::Element::FindElement instead of GetElement in ApplyLinkWrench
    * [Pull request #2052](https://github.com/gazebosim/gz-sim/pull/2052)

1. Backport sensors system threading optimization changes
    * [Pull request #2058](https://github.com/gazebosim/gz-sim/pull/2058)

1. Apply Force and Torque GUI plugin
    * [Pull request #2014](https://github.com/gazebosim/gz-sim/pull/2014)

1. Adds a warning if the `Server` method of a `TestFixture` is called before `Finalize`
    * [Pull request #2047](https://github.com/gazebosim/gz-sim/pull/2047)

1. Support loading mesh by mesh name in `<mesh><uri>`
    * [Pull request #2007](https://github.com/gazebosim/gz-sim/pull/2007)

1. ComponentInspector: display PhysicsEnginePlugin
    * [Pull request #2032](https://github.com/gazebosim/gz-sim/pull/2032)

1. Send BlockOrbit false events only once from TransformControl plugin
    * [Pull request #2030](https://github.com/gazebosim/gz-sim/pull/2030)

1. Categorize tutorials list
    * [Pull request #2028](https://github.com/gazebosim/gz-sim/pull/2028)

1. Add time out to wait to avoid deadlock
    * [Pull request #2025](https://github.com/gazebosim/gz-sim/pull/2025)

1. Add optional binary relocatability
    * [Pull request #1968](https://github.com/gazebosim/gz-sim/pull/1968)

1. Several minor fixes
    * [Pull request #2027](https://github.com/gazebosim/gz-sim/pull/2027)

1. Protobuf: Do not require version 3 do support Protobuf 4.23.2 (23.2)
    * [Pull request #2006](https://github.com/gazebosim/gz-sim/pull/2006)

1. Support world joints (joints inside `<world>` tags)
    * [Pull request #1949](https://github.com/gazebosim/gz-sim/pull/1949)

1. Disable pybind11 on Windows by default
    * [Pull request #2005](https://github.com/gazebosim/gz-sim/pull/2005)

1. Port record topic fix
    * [Pull request #2004](https://github.com/gazebosim/gz-sim/pull/2004)

1. Allow re-attaching detached joint
    * [Pull request #1687](https://github.com/gazebosim/gz-sim/pull/1687)

1.  Enable GzWeb visualization of markers by republishing service requests on a topic
    * [Pull request #1994](https://github.com/gazebosim/gz-sim/pull/1994)

1. Support loading Projectors
    * [Pull request #1979](https://github.com/gazebosim/gz-sim/pull/1979)

1. Small fixes to gz headers
    * [Pull request #1985](https://github.com/gazebosim/gz-sim/pull/1985)

1. Speed up Resource Spawner load time by fetching model list asynchronously
    * [Pull request #1962](https://github.com/gazebosim/gz-sim/pull/1962)

1. Add redirection header gz/sim.hh
    * [Pull request #1983](https://github.com/gazebosim/gz-sim/pull/1983)

1. Add missing cmake exports from core library
    * [Pull request #1978](https://github.com/gazebosim/gz-sim/pull/1978)

1. Add tutorial on migrating the Sensor class from gazebo classic
    * [Pull request #1930](https://github.com/gazebosim/gz-sim/pull/1930)

1. ign -> gz Migrate Ignition Headers : gz-sim
    * [Pull request #1646](https://github.com/gazebosim/gz-sim/pull/1646)


### Gazebo Sim 7.5.0 (2023-05-14)

1. Actuators message input for JointController.
    * [Pull request #1953](https://github.com/gazebosim/gz-sim/pull/1953)

1. fixed a code block in the python interfaces tutorial
    * [Pull request #1982](https://github.com/gazebosim/gz-sim/pull/1982)

1. Add missing cmake exports from core library
    * [Pull request #1978](https://github.com/gazebosim/gz-sim/pull/1978)

1. Actuators message for JointPositionController.
    * [Pull request #1954](https://github.com/gazebosim/gz-sim/pull/1954)

1. Update sdf plugins to use actuator_number.
    * [Pull request #1976](https://github.com/gazebosim/gz-sim/pull/1976)

1. Unload render engine when the sensors system exits
    * [Pull request #1960](https://github.com/gazebosim/gz-sim/pull/1960)

1. Use GzSpinBox.
    * [Pull request #1969](https://github.com/gazebosim/gz-sim/pull/1969)

1. Add tutorial on migrating the Actor class from gazebo classic
    * [Pull request #1929](https://github.com/gazebosim/gz-sim/pull/1929)

1. Add back in the marker example
    * [Pull request #1972](https://github.com/gazebosim/gz-sim/pull/1972)

1. Optimize render updates and use of thread mutexes in Sensors system
    * [Pull request #1938](https://github.com/gazebosim/gz-sim/pull/1938)

1. Fix use of actors that only has trajectory animation
    * [Pull request #1947](https://github.com/gazebosim/gz-sim/pull/1947)

1. Actuators message input for Ackermann Steering.
    * [Pull request #1952](https://github.com/gazebosim/gz-sim/pull/1952)

1. Add tutorial on migrating the Joint class from gazebo classic
    * [Pull request #1925](https://github.com/gazebosim/gz-sim/pull/1925)

1. Add tutorial on migrating the Light class from gazebo classic
    * [Pull request #1931](https://github.com/gazebosim/gz-sim/pull/1931)

1. Remove filtering from realtime factor (RTF) calculation
    * [Pull request #1942](https://github.com/gazebosim/gz-sim/pull/1942)

1. Fix docker/README.md
    * [Pull request #1964](https://github.com/gazebosim/gz-sim/pull/1964)

1. gz_TEST: improve initial sim time test reliability
    * [Pull request #1916](https://github.com/gazebosim/gz-sim/pull/1916)

1. Use a queue to track component registration from mulitiple sources
    * [Pull request #1836](https://github.com/gazebosim/gz-sim/pull/1836)

1. Initialize services in ViewAngle constructor
    * [Pull request #1943](https://github.com/gazebosim/gz-sim/pull/1943)

1. CI workflow: use checkout v3
    * [Pull request #1940](https://github.com/gazebosim/gz-sim/pull/1940)

1. Rename COPYING to LICENSE
    * [Pull request #1937](https://github.com/gazebosim/gz-sim/pull/1937)

1. add comment on center of buoyancy force
    * [Pull request #1935](https://github.com/gazebosim/gz-sim/pull/1935)

1. Get Windows to green on gz-sim7
    * [Pull request #1917](https://github.com/gazebosim/gz-sim/pull/1917)

1. Add Light class
    * [Pull request #1918](https://github.com/gazebosim/gz-sim/pull/1918)

1. Resolve inconsistent visibility on ign-gazebo6
    * [Pull request #1914](https://github.com/gazebosim/gz-sim/pull/1914)

1. relax msg count check in RF comms integration test
    * [Pull request #1920](https://github.com/gazebosim/gz-sim/pull/1920)

1. Fix off-by-one error in physics test
    * [Pull request #1921](https://github.com/gazebosim/gz-sim/pull/1921)

1. Fix formatting of error messages with large mesh file names
    * [Pull request #1654](https://github.com/gazebosim/gz-sim/pull/1654)

1. Add Actor class
    * [Pull request #1913](https://github.com/gazebosim/gz-sim/pull/1913)

1. Update all velocity and acceleration components of non-link entities
    * [Pull request #1868](https://github.com/gazebosim/gz-sim/pull/1868)

1. Add Sensor class
    * [Pull request #1912](https://github.com/gazebosim/gz-sim/pull/1912)

1. Minor vocab fix
    * [Pull request #1915](https://github.com/gazebosim/gz-sim/pull/1915)

1. Allow to change camera user hfov in camera_view plugin
    * [Pull request #1807](https://github.com/gazebosim/gz-sim/pull/1807)

1. Address a few Windows CI Issues
    * [Pull request #1911](https://github.com/gazebosim/gz-sim/pull/1911)

1. Added magnetometer value based on location
    * [Pull request #1907](https://github.com/gazebosim/gz-sim/pull/1907)

1. Allow specifying initial simulation time with a CLI argument
    * [Pull request #1801](https://github.com/gazebosim/gz-sim/pull/1801)

1. Add Joint class
    * [Pull request #1910](https://github.com/gazebosim/gz-sim/pull/1910)

1. Added reset simulation tutorial
    * [Pull request #1824](https://github.com/gazebosim/gz-sim/pull/1824)

1. Add SensorTopic component to rendering sensors
    * [Pull request #1908](https://github.com/gazebosim/gz-sim/pull/1908)

1. Use a queue to track component registration from mulitiple sources
    * [Pull request #1836](https://github.com/gazebosim/gz-sim/pull/1836)

1. Document behaviour changes introduced #1784
    * [Pull request #1888](https://github.com/gazebosim/gz-sim/pull/1888)

1. Fix GUI_clean_exit test by increasing thread delay
    * [Pull request #1902](https://github.com/gazebosim/gz-sim/pull/1902)

1. Partial backport of 1728
    * [Pull request #1901](https://github.com/gazebosim/gz-sim/pull/1901)

1. Fix gz plugin paths in windows
    * [Pull request #1899](https://github.com/gazebosim/gz-sim/pull/1899)

1. Increase timeout for UNIT_Gui_clean_exit_TEST
    * [Pull request #1897](https://github.com/gazebosim/gz-sim/pull/1897)

1. fix triggered camera test by waiting for rendering / scene to be ready
    * [Pull request #1895](https://github.com/gazebosim/gz-sim/pull/1895)

1. cmdsim.rb: fix ruby syntax
    * [Pull request #1884](https://github.com/gazebosim/gz-sim/pull/1884)

1. Fix some windows warnings (C4244 and C4305)
    * [Pull request #1874](https://github.com/gazebosim/gz-sim/pull/1874)

1. Minor optimization to transform control tool
    * [Pull request #1854](https://github.com/gazebosim/gz-sim/pull/1854)

1. inherit material cast shadows property
    * [Pull request #1856](https://github.com/gazebosim/gz-sim/pull/1856)

1. fix record topic
    * [Pull request #1855](https://github.com/gazebosim/gz-sim/pull/1855)

1. Remove duplicate Fuel server used by ResourceSpawner
    * [Pull request #1830](https://github.com/gazebosim/gz-sim/pull/1830)

1. re-add namespace
    * [Pull request #1826](https://github.com/gazebosim/gz-sim/pull/1826)

1. Fix QML warnings regarding binding loops
    * [Pull request #1829](https://github.com/gazebosim/gz-sim/pull/1829)

1. Update documentation on `UpdateInfo::realTime`
    * [Pull request #1817](https://github.com/gazebosim/gz-sim/pull/1817)

1. Add jennuine as GUI codeowner
    * [Pull request #1800](https://github.com/gazebosim/gz-sim/pull/1800)

1. remove PlotIcon
    * [Pull request #1658](https://github.com/gazebosim/gz-sim/pull/1658)

1. Final update of ignitionrobotics to gazebosim for citadel
    * [Pull request #1760](https://github.com/gazebosim/gz-sim/pull/1760)

1. Convert ignitionrobotics to gazebosim in tutorials
    * [Pull request #1759](https://github.com/gazebosim/gz-sim/pull/1759)

1. Convert ignitionrobotics to gazebosim in sources and includes
    * [Pull request #1758](https://github.com/gazebosim/gz-sim/pull/1758)

1. Convert ignitionrobotics to gazebosim in tests directory
    * [Pull request #1757](https://github.com/gazebosim/gz-sim/pull/1757)

1. Added collection name to About Dialog
    * [Pull request #1756](https://github.com/gazebosim/gz-sim/pull/1756)

1. Citadel: Removed warnings
    * [Pull request #1753](https://github.com/gazebosim/gz-sim/pull/1753)

1. Remove actors from screen when they are supposed to
    * [Pull request #1699](https://github.com/gazebosim/gz-sim/pull/1699)

1. readd namespaces for Q_ARGS
    * [Pull request #1670](https://github.com/gazebosim/gz-sim/pull/1670)

1. 🎈 3.14.0
    * [Pull request #1657](https://github.com/gazebosim/gz-sim/pull/1657)

1. Remove redundant namespace references
    * [Pull request #1635](https://github.com/gazebosim/gz-sim/pull/1635)

1. 🎈 3.14.0~pre1
    * [Pull request #1650](https://github.com/gazebosim/gz-sim/pull/1650)

### Gazebo Sim 7.4.0 (2023-02-14)

1. Added airspeed sensor
    * [Pull request #1847](https://github.com/gazebosim/gz-sim/pull/1847)

1. JointPosController: support nested joints
    * [Pull request #1851](https://github.com/gazebosim/gz-sim/pull/1851)

1. cmdsim.rb: fix ruby syntax
    * [Pull request #1884](https://github.com/gazebosim/gz-sim/pull/1884)

1. Fix view angle plugin
    * [Pull request #1877](https://github.com/gazebosim/gz-sim/pull/1877)

1. Fix cmake unrecognized argument warning
    * [Pull request #1882](https://github.com/gazebosim/gz-sim/pull/1882)

### Gazebo Sim 7.3.0 (2023-02-02)

1. Fluid added mass
    * [Pull request #1592](https://github.com/gazebosim/gz-sim/pull/1592)

1. Add P gain value for Ackermann steering.
    * [Pull request #1873](https://github.com/gazebosim/gz-sim/pull/1873)

1. Add orientation to Odom with covariance.
    * [Pull request #1876](https://github.com/gazebosim/gz-sim/pull/1876)

### Gazebo Sim 7.2.0 (2023-01-25)

1. Enable the JointController and JointPositionController to use sub_topics and control multiple joints.
    * [Pull request #1861](https://github.com/gazebosim/gz-sim/pull/1861)

1. Ackermann steering with steering angle and sub_topic.
    * [Pull request #1860](https://github.com/gazebosim/gz-sim/pull/1860)

1. port: 6 to 7 (10-JAN-2023)
    * [Pull request #1857](https://github.com/gazebosim/gz-sim/pull/1857)

1. Add ignition alias back
    * [Pull request #1858](https://github.com/gazebosim/gz-sim/pull/1858)

1. fix SdfGenerator unit test
    * [Pull request #1853](https://github.com/gazebosim/gz-sim/pull/1853)

1. Allow using a CSV file to define currents for hydrodynamic system
    * [Pull request #1839](https://github.com/gazebosim/gz-sim/pull/1839)

1. Add multichannel lookup for environment sensors.
    * [Pull request #1814](https://github.com/gazebosim/gz-sim/pull/1814)

1. Example controller for LRAUV
    * [Pull request #1822](https://github.com/gazebosim/gz-sim/pull/1822)

1. Fix component removal in component inspector
    * [Pull request #1833](https://github.com/gazebosim/gz-sim/pull/1833)

1. port: 6 to 7 (06-DEC-2023)
    * [Pull request #1832](https://github.com/gazebosim/gz-sim/pull/1832)

1. port: 6 to 7 (29-NOV-2023)
    * [Pull request #1821](https://github.com/gazebosim/gz-sim/pull/1821)

1. Fix #1812.
    * [Pull request #1813](https://github.com/gazebosim/gz-sim/pull/1813)

1. Removed unused attributes
    * [Pull request #1809](https://github.com/gazebosim/gz-sim/pull/1809)

1. Fixes buoyancy flakiness when spawning entities
    * [Pull request #1808](https://github.com/gazebosim/gz-sim/pull/1808)

1. Remove fixed width from world control
    * [Pull request #1805](https://github.com/gazebosim/gz-sim/pull/1805)

1. Backport #1748: Adds a tool for environment data visualization and custom environmental sensors
    * [Pull request #1798](https://github.com/gazebosim/gz-sim/pull/1798)

1. Acoustic comms : Propagation model
    * [Pull request #1793](https://github.com/gazebosim/gz-sim/pull/1793)

1. Add pre-commit hooks configuration
    * [Pull request #1792](https://github.com/gazebosim/gz-sim/pull/1792)

1. Add checkbox in view angle plugin for toggling view control reference visual
    * [Pull request #1788](https://github.com/gazebosim/gz-sim/pull/1788)

1. Add EnvironmentalData component
    * [Pull request #1616](https://github.com/gazebosim/gz-sim/pull/1616)

### Gazebo Sim 7.1.0 (2022-11-10)

1. Allow SDF model to be constructed in a single shot
    * [Pull request #1560](https://github.com/gazebosim/gz-sim/pull/1560)

1. Allow loading a model SDF file in the Server class
    * [Pull request #1775](https://github.com/gazebosim/gz-sim/pull/1775)

1. Address flaky UNIT_Gui_TEST
    * [Pull request #1776](https://github.com/gazebosim/gz-sim/pull/1776)

1. Change name of nameFilter of saveDialog to "SDF files"
    * [Pull request #1774](https://github.com/gazebosim/gz-sim/pull/1774)

1. Acoustic comms : Packet collision timeout
    * [Pull request #1755](https://github.com/gazebosim/gz-sim/pull/1755)

1. Fix typo in include inside Export.hh
    * [Pull request #1778](https://github.com/gazebosim/gz-sim/pull/1778)

1. Towards Green CI
    * [Pull request #1771](https://github.com/gazebosim/gz-sim/pull/1771)

1. Refactor: Trajectory loading seperated into external function
    * [Pull request #1744](https://github.com/gazebosim/gz-sim/pull/1744)

1. Add pybind11 module as MODULE
    * [Pull request #1763](https://github.com/gazebosim/gz-sim/pull/1763)

1. Clarify errors when plugins fail to load
    * [Pull request #1727](https://github.com/gazebosim/gz-sim/pull/1727)

1. Fix tutorial for `blender_sdf_exporter`
    * [Pull request #1718](https://github.com/gazebosim/gz-sim/pull/1718)

1. Cherry pick acoustic comms to gz-sim7
    * [Pull request #1704](https://github.com/gazebosim/gz-sim/pull/1704)

1. Update tutorial for `blender_distort_meshes`
    * [Pull request #1719](https://github.com/gazebosim/gz-sim/pull/1719)

1. Removes Actor Visuals When They Are Despawned
    * [Pull request #1697](https://github.com/gazebosim/gz-sim/pull/1697)

1. Update examples to used gazebosim
    * [Pull request #1726](https://github.com/gazebosim/gz-sim/pull/1726)

1. Merge forward 6 to 7, 2022-10-21
    * [Pull request #1768](https://github.com/gazebosim/gz-sim/pull/1768)

1. Merge forward 6 to 7, 2022-10-06
    * [Pull request #1676](https://github.com/gazebosim/gz-sim/pull/1676)

1. Add information about <topic> system paramter
    * [Pull request #1671](https://github.com/gazebosim/gz-sim/pull/1671)

1. Adding tests for hydrodynamics
    * [Pull request #1617](https://github.com/gazebosim/gz-sim/pull/1617)

1. Fix Windows and Doxygen
    * [Pull request #1643](https://github.com/gazebosim/gz-sim/pull/1643)

1. Merge forward 3 to 6, 2022-08-16
    * [Pull request #1651](https://github.com/gazebosim/gz-sim/pull/1651)

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

### Gazebo Sim 7.0.0 (2022-09-27)

1. Fix some warnings in visualize_lidar.sdf
    * [Pull request #1702](https://github.com/gazebosim/gz-sim/pull/1702)

1. Lock step video recording is broken, disabling
    * [Pull request #1707](https://github.com/gazebosim/gz-sim/pull/1707)

1. Add code quotes to TestFixture.hh
    * [Pull request #1723](https://github.com/gazebosim/gz-sim/pull/1723)

1. Update URL in tutorial for `python_interfaces`
    * [Pull request #1721](https://github.com/gazebosim/gz-sim/pull/1721)

1. Rename python library as gz.sim7
    * [Pull request #1716](https://github.com/gazebosim/gz-sim/pull/1716)

1. Fix documentation on visibility.sdf
    * [Pull request #1703](https://github.com/gazebosim/gz-sim/pull/1703)

1. Fix display of Pose3d and Vector3d
    * [Pull request #1680](https://github.com/gazebosim/gz-sim/pull/1680)

1. Use RTLD_NODELETE=true when loading libraries
    * [Pull request #1649](https://github.com/gazebosim/gz-sim/pull/1649)

1. Update headers for missing transitive includes
    * [Pull request #1566](https://github.com/gazebosim/gz-sim/pull/1566)

1. Add topic parameter to thrust plugin
    * [Pull request #1681](https://github.com/gazebosim/gz-sim/pull/1681)

1. Remove unused function
    * [Pull request #1678](https://github.com/gazebosim/gz-sim/pull/1678)

1. Fixed python build in gz-sim7
    * [Pull request #1667](https://github.com/gazebosim/gz-sim/pull/1667)

1. Fix Windows and Doxygen
    * [Pull request #1643](https://github.com/gazebosim/gz-sim/pull/1643)

1. Fix gz_TEST paths
    * [Pull request #1675](https://github.com/gazebosim/gz-sim/pull/1675)

1. Fix two tests
    * [Pull request #1674](https://github.com/gazebosim/gz-sim/pull/1674)

1. Add Metal support to Gazebo for macOS
    * [Pull request #1225](https://github.com/gazebosim/gz-sim/pull/1225)

1. Add information about <topic> system paramter
    * [Pull request #1671](https://github.com/gazebosim/gz-sim/pull/1671)

1. Adding tests for hydrodynamics
    * [Pull request #1617](https://github.com/gazebosim/gz-sim/pull/1617)

1. Fix Windows and Doxygen
    * [Pull request #1643](https://github.com/gazebosim/gz-sim/pull/1643)

1. Introduces new mesh formats to the drag&drop err
    * [Pull request #1653](https://github.com/gazebosim/gz-sim/pull/1653)

1. Add support for specifying log record period
    * [Pull request #1636](https://github.com/gazebosim/gz-sim/pull/1636)

1. Common widget GzColor replacement
    * [Pull request #1530](https://github.com/gazebosim/gz-sim/pull/1530)

1. Add option to disable building python bindings
    * [Pull request #1637](https://github.com/gazebosim/gz-sim/pull/1637)

1. Replace plotIcon in ComponentInspector with GzPlotIcon
    * [Pull request #1638](https://github.com/gazebosim/gz-sim/pull/1638)

1. Component Inspector with common widget pose plotting
    * [Pull request #1607](https://github.com/gazebosim/gz-sim/pull/1607)

1. 🕐 Tock: Remove Fortress deprecations
    * [Pull request #1605](https://github.com/gazebosim/gz-sim/pull/1605)

1. Change CODEOWNERS and maintainer to Michael
    * [Pull request #1644](https://github.com/gazebosim/gz-sim/pull/1644)

1. Replace pose in ViewAngle with GzPose
    * [Pull request #1641](https://github.com/gazebosim/gz-sim/pull/1641)

1. Update gz-sensors branch in example
    * [Pull request #1634](https://github.com/gazebosim/gz-sim/pull/1634)

1. Deprecations, ign -> gz
    * [Pull request #1631](https://github.com/gazebosim/gz-sim/pull/1631)

1. Migrate config and log directories
    * [Pull request #1629](https://github.com/gazebosim/gz-sim/pull/1629)

1. ign to gz
    * [Pull request #1630](https://github.com/gazebosim/gz-sim/pull/1630)

1. Just the ABI breaking parts of #1560
    * [Pull request #1624](https://github.com/gazebosim/gz-sim/pull/1624)

1. Supply spherical coords when loading DEMs
    * [Pull request #1556](https://github.com/gazebosim/gz-sim/pull/1556)

1. Bump actions dependencies to Ogre 2.3
    * [Pull request #1620](https://github.com/gazebosim/gz-sim/pull/1620)

1. Install gz packages instead of ignition
    * [Pull request #1614](https://github.com/gazebosim/gz-sim/pull/1614)

1. Use stepping field in message
    * [Pull request #1612](https://github.com/gazebosim/gz-sim/pull/1612)

1. Don't use 'EachNew' in ForceTorque PreUpdate function
    * [Pull request #1523](https://github.com/gazebosim/gz-sim/pull/1523)

1. Test case for simulation reset with detachable joints
    * [Pull request #1511](https://github.com/gazebosim/gz-sim/pull/1511)

1. Fix Python bindings
    * [Pull request #1606](https://github.com/gazebosim/gz-sim/pull/1606)

1. ign -> gz Provisional Finale: Source Migration : gz-sim
    * [Pull request #1591](https://github.com/gazebosim/gz-sim/pull/1591)

1. ign -> gz CMake, Python, Partial Source, and File Migrations : gz-sim
    * [Pull request #1589](https://github.com/gazebosim/gz-sim/pull/1589)

1. Tutorial for mesh distortion in Blender Python
    * [Pull request #1401](https://github.com/gazebosim/gz-sim/pull/1401)

1. Use new has connections function
    * [Pull request #1528](https://github.com/gazebosim/gz-sim/pull/1528)

1. Fix compilation of scene broadcaster test
    * [Pull request #1599](https://github.com/gazebosim/gz-sim/pull/1599)

1. Restore CXX_STANDARD 17
    * [Pull request #1586](https://github.com/gazebosim/gz-sim/pull/1586)

1. ign -> gz Shared Lib Migration : gz-sim
    * [Pull request #1535](https://github.com/gazebosim/gz-sim/pull/1535)

1. Garden: fix windows CI build
    * [Pull request #1578](https://github.com/gazebosim/gz-sim/pull/1578)

1. Implement system Reset interface for Sensors and SceneBroadcaster
    * [Pull request #1355](https://github.com/gazebosim/gz-sim/pull/1355)

1. ign -> gz Partial Docs Migration and Project Name Followups : gz-sim
    * [Pull request #1558](https://github.com/gazebosim/gz-sim/pull/1558)

1. Update GoogleTest to latest version
    * [Pull request #1559](https://github.com/gazebosim/gz-sim/pull/1559)

1. Rename CMake project to gz
    * [Pull request #1527](https://github.com/gazebosim/gz-sim/pull/1527)

1. Detect gz program instead of using CMake module to check for gz-tools
    * [Pull request #1557](https://github.com/gazebosim/gz-sim/pull/1557)

1. ign -> gz CLI Migration : gz-sim
    * [Pull request #1533](https://github.com/gazebosim/gz-sim/pull/1533)

1. Use new Joint APIs for Parent/Child name
    * [Pull request #1548](https://github.com/gazebosim/gz-sim/pull/1548)

1. Expose rendering teardown event
    * [Pull request #1539](https://github.com/gazebosim/gz-sim/pull/1539)

1. Add QML Debugging support
    * [Pull request #1503](https://github.com/gazebosim/gz-sim/pull/1503)

1. [ign -> gz] CMake functions
    * [Pull request #1542](https://github.com/gazebosim/gz-sim/pull/1542)

1. ign -> gz Macro Migration : gz-sim
    * [Pull request #1520](https://github.com/gazebosim/gz-sim/pull/1520)

1. Allow rendering to be forced externally
    * [Pull request #1475](https://github.com/gazebosim/gz-sim/pull/1475)

1. Apply shininess value to visuals
    * [Pull request #1483](https://github.com/gazebosim/gz-sim/pull/1483)

1. ign -> gz Environment Variable Migration
    * [Pull request #1518](https://github.com/gazebosim/gz-sim/pull/1518)

1. More missing math includes and math::clock fixes
    * [Pull request #1526](https://github.com/gazebosim/gz-sim/pull/1526)

1. Rename ignition to gz in #1519.
    * [Pull request #1519](https://github.com/gazebosim/gz-sim/pull/1519)

1. Use pose multiplication instead of subtraction
    * [Pull request #1521](https://github.com/gazebosim/gz-sim/pull/1521)

1. Add missing gz-math includes
    * [Pull request #1525](https://github.com/gazebosim/gz-sim/pull/1525)

1. [ign ➡️  gz] Logo, docs, tools
    * [Pull request #1517](https://github.com/gazebosim/gz-sim/pull/1517)

1. Remove ign-rendering SetTime hack
    * [Pull request #1514](https://github.com/gazebosim/gz-sim/pull/1514)

1. ign -> gz: namespaces, .gz directory
    * [Pull request #1496](https://github.com/gazebosim/gz-sim/pull/1496)

1. Update documentation in the linear battery plugin example.
    * [Pull request #1500](https://github.com/gazebosim/gz-sim/pull/1500)

1. Use gz/sim/test_config
    * [Pull request #1498](https://github.com/gazebosim/gz-sim/pull/1498)

1. Quality of life improvements for examples_build test
    * [Pull request #1493](https://github.com/gazebosim/gz-sim/pull/1493)

1. Update test log for gz components
    * [Pull request #1477](https://github.com/gazebosim/gz-sim/pull/1477)

1. Migrate CMake files
    * [Pull request #1477](https://github.com/gazebosim/gz-sim/pull/1477)

1. Migrate sources in src, test, examples, and include
    * [Pull request #1477](https://github.com/gazebosim/gz-sim/pull/1477)

1. Create redirection aliases
    * [Pull request #1477](https://github.com/gazebosim/gz-sim/pull/1477)

1. Move header files with git mv
    * [Pull request #1477](https://github.com/gazebosim/gz-sim/pull/1477)

1. Implement reset interface in the physics system
    * [Pull request #1327](https://github.com/gazebosim/gz-sim/pull/1327)

1. Use uint64_t with gazebo-entity user dataa
    * [Pull request #1451](https://github.com/gazebosim/gz-sim/pull/1451)

1. Depend on math7 and remove Bionic packages
    * [Pull request #1446](https://github.com/gazebosim/gz-sim/pull/1446)

1. Used Light ign-msgs is_light_off
    * [Pull request #1449](https://github.com/gazebosim/gz-sim/pull/1449)

1. Use message field visualize_visual
    * [Pull request #1453](https://github.com/gazebosim/gz-sim/pull/1453)

1. Tweaks to python docs
    * [Pull request #1448](https://github.com/gazebosim/gz-sim/pull/1448)

1. Use utils instead of ign-cmake utilities
    * [Pull request #1446](https://github.com/gazebosim/gz-sim/pull/1446)

    * [Pull request #1379](https://github.com/gazebosim/gz-sim/pull/1379)

1. Remove internal python bindings for sdformat
    * [Pull request #1447](https://github.com/gazebosim/gz-sim/pull/1447)

1. Bumps in garden : ign-utils2, ign-plugin2
    * [Pull request #1444](https://github.com/gazebosim/gz-sim/pull/1444)

1. Fix deprecation warnings for ModelPhotoShoot
    * [Pull request #1437](https://github.com/gazebosim/gz-sim/pull/1437)

1. Remove unused View::Clone method
    * [Pull request #1440](https://github.com/gazebosim/gz-sim/pull/1440)

1. replace deprecated common::SubMesh::MaterialIndex() with GetMaterialIndex()
    * [Pull request #1436](https://github.com/gazebosim/gz-sim/pull/1436)

1. Make WindEffects configurable on a location basis
    * [Pull request #1357](https://github.com/gazebosim/gz-sim/pull/1357)

1. Fix faulty assumption in INTEGRATION_log_system
    * [Pull request #1426](https://github.com/gazebosim/gz-sim/pull/1426)

1. Clean up various warnings caught by clang12
    * [Pull request #1421](https://github.com/gazebosim/gz-sim/pull/1421)

1. Fix visibility and add documentation
    * [Pull request #1407](https://github.com/gazebosim/gz-sim/pull/1407)

1. Preserve sign of thrust_coefficient
    * [Pull request #1402](https://github.com/gazebosim/gz-sim/pull/1402)

1. Support world reset
    * [Pull request #1249](https://github.com/gazebosim/gz-sim/pull/1249)

1. Added DEM support
    * [Pull request #1262](https://github.com/gazebosim/gz-sim/pull/1262)

1. Remove Bionic from future releases (Garden+)
    * [Pull request #1388](https://github.com/gazebosim/gz-sim/pull/1388)

1. Add support for wide angle camera in sensors system
    * [Pull request #1215](https://github.com/gazebosim/gz-sim/pull/1215)

1. Enable WorldPose component on TrajectoryFollower
    * [Pull request #1382](https://github.com/gazebosim/gz-sim/pull/1382)

1. Eliminates std::filesystem usage in utils.cc
    * [Pull request #1346](https://github.com/gazebosim/gz-sim/pull/1346)

1. Emitter migration
    * [Pull request #1287](https://github.com/gazebosim/gz-sim/pull/1287)

1. [Garden] Make tests run as fast as possible
    * [Pull request #1293](https://github.com/gazebosim/gz-sim/pull/1293)

1. Mark Component::Clone as const
    * [Pull request #1300](https://github.com/gazebosim/gz-sim/pull/1300)

1. Improve Sensor::Update call
    * [Pull request #1283](https://github.com/gazebosim/gz-sim/pull/1283)

1. Thruster plugin: accept angular velocity and provide feedback on topic
    * [Pull request #1269](https://github.com/gazebosim/gz-sim/pull/1269)

1. Re-enable triggered publisher tests
    * [Pull request #1271](https://github.com/gazebosim/gz-sim/pull/1271)

1. Bumps in garden: use ign-math7 and dependents
    * [Pull request #1264](https://github.com/gazebosim/gz-sim/pull/1264)

1. Update Docker instructions for Garden
    * [Pull request #1244](https://github.com/gazebosim/gz-sim/pull/1244)

1. Bumps in garden : `ci_matching_branch/bump_garden_ign-gazebo7`
    * [Pull request #1234](https://github.com/gazebosim/gz-sim/pull/1234)

1. Bumps in garden : ign-gazebo7
    * [Pull request #1183](https://github.com/gazebosim/gz-sim/pull/1183)

1. Clarify available Ignition Versions
    * [Pull request #1161](https://github.com/gazebosim/gz-sim/pull/1161)

1. Add error message for non-zip files in playback mode
    * [Pull request #1110](https://github.com/gazebosim/gz-sim/pull/1110)

1. Bump main to 7.0.0~pre1
    * [Pull request #1094](https://github.com/gazebosim/gz-sim/pull/1094)

1. update CODEOWNERS
    * [Pull request #1093](https://github.com/gazebosim/gz-sim/pull/1093)

## Gazebo Sim 6.x

### Gazebo Sim 6.17.0 (2025-01-10)

1. Add parameter for adjust current sign in battery plugin
    * [Pull request #2696](https://github.com/gazebosim/gz-sim/pull/2696)

1. Fix uncontrolled cast of size_t to uint
    * [Pull request #2687](https://github.com/gazebosim/gz-sim/pull/2687)

1. Improve load times by skipping serialization of entities when unecessary
    * [Pull request #2596](https://github.com/gazebosim/gz-sim/pull/2596)

1. Fix crash in OpticalTactilePlugin by checking for valid visualize pointer
    * [Pull request #2674](https://github.com/gazebosim/gz-sim/pull/2674)

1. Disable detachable_joint integration test case on Windows
    * [Pull request #2523](https://github.com/gazebosim/gz-sim/pull/2523)

1. Initialize threadsNeedCleanUp
    * [Pull request #2503](https://github.com/gazebosim/gz-sim/pull/2503)

1. Remove systems if their parent entity is removed
    * [Pull request #2232](https://github.com/gazebosim/gz-sim/pull/2232)

1. Disable failing testFixture_TEST for MacOS
    * [Pull request #2499](https://github.com/gazebosim/gz-sim/pull/2499)

1. backport lidar visualization frame_id fix
    * [Pull request #2483](https://github.com/gazebosim/gz-sim/pull/2483)

1. Fix DLL linkage/visibility issues
    * [Pull request #2254](https://github.com/gazebosim/gz-sim/pull/2254)

1. Address a few Windows CI Issues
    * [Pull request #1911](https://github.com/gazebosim/gz-sim/pull/1911)

1. Add GravityEnabled boolean component
    * [Pull request #2451](https://github.com/gazebosim/gz-sim/pull/2451)

1. Add support for no gravity link
    * [Pull request #2398](https://github.com/gazebosim/gz-sim/pull/2398)

1. Use VERSION_GREATER_EQUAL in cmake logic
    * [Pull request #2418](https://github.com/gazebosim/gz-sim/pull/2418)

1. Rephrase cmake comment about CMP0077
    * [Pull request #2419](https://github.com/gazebosim/gz-sim/pull/2419)

1. Fix bug where iterator was used after the underlying item was erased from the container
    * [Pull request #2412](https://github.com/gazebosim/gz-sim/pull/2412)

1. Fix namespace and class links in documentation references that use namespace `gz`
    * [Pull request #2385](https://github.com/gazebosim/gz-sim/pull/2385)

1. Fix ModelPhotoShootTest test failures
    * [Pull request #2294](https://github.com/gazebosim/gz-sim/pull/2294)

1. Setup rendering environment before cmake runs
    * [Pull request #1965](https://github.com/gazebosim/gz-sim/pull/1965)

1. Detachable joint: support for nested models of the same name
    * [Pull request 1097](https://github.com/gazebosim/gz-sim/pull/1097)

### Gazebo Sim 6.16.0 (2024-01-12)

1. Allow using plugin file names and environment variables compatible with Garden and later
    * [Pull request #2275](https://github.com/gazebosim/gz-sim/pull/2275)

1. Update friction parameters for skid steer example
    * [Pull request #2235](https://github.com/gazebosim/gz-sim/pull/2235)

1. Relax pose check in actor no mesh test
    * [Pull request #2196](https://github.com/gazebosim/gz-sim/pull/2196)

1. Fix macOS test failures by registering components in the core library
    * [Pull request #2220](https://github.com/gazebosim/gz-sim/pull/2220)

1. Fix for sensor pointer null when navsat plugin in included in sdf
    * [Pull request #2176](https://github.com/gazebosim/gz-sim/pull/2176)

1. Fix another deadlock in sensors system
    * [Pull request #2200](https://github.com/gazebosim/gz-sim/pull/2200)

1. Backport component inspector Vector3d width fix
    * [Pull request #2195](https://github.com/gazebosim/gz-sim/pull/2195)

1. Bump Fuel model version in test
    * [Pull request #2190](https://github.com/gazebosim/gz-sim/pull/2190)

1. Infrastructure
    * [Pull request #2237](https://github.com/gazebosim/gz-sim/pull/2237)
    * [Pull request #2222](https://github.com/gazebosim/gz-sim/pull/2222)

### Gazebo Sim 6.15.0 (2023-08-16)

1. Fix Joint Position Controller Behaviour Described in #1997
    * [Pull request #2001](https://github.com/gazebosim/gz-sim/pull/2001)

1. Fix a minor issue in the documentation of the server API
    * [Pull request #2067](https://github.com/gazebosim/gz-sim/pull/2067)

1. Use sdf::Element::FindElement instead of GetElement in ApplyLinkWrench
    * [Pull request #2052](https://github.com/gazebosim/gz-sim/pull/2052)

1. Backport sensors system threading optimization changes
    * [Pull request #2058](https://github.com/gazebosim/gz-sim/pull/2058)

1. Adds a warning if the `Server` method of a `TestFixture` is called before `Finalize`
    * [Pull request #2047](https://github.com/gazebosim/gz-sim/pull/2047)

1. Protobuf: Do not require version 3 do support Protobuf 4.23.2 (23.2)
    * [Pull request #2006](https://github.com/gazebosim/gz-sim/pull/2006)

1. Disable pybind11 on Windows by default
    * [Pull request #2005](https://github.com/gazebosim/gz-sim/pull/2005)

1. Print an error message when trying to load SDF files that don't contain a `<world>`
    * [Pull request #1998](https://github.com/gazebosim/gz-sim/pull/1998)

1. Port record topic fix
    * [Pull request #2004](https://github.com/gazebosim/gz-sim/pull/2004)

1. Allow re-attaching detached joint
    * [Pull request #1687](https://github.com/gazebosim/gz-sim/pull/1687)

1.  Enable GzWeb visualization of markers by republishing service requests on a topic
    * [Pull request #1994](https://github.com/gazebosim/gz-sim/pull/1994)

1. Small fixes to gz headers
    * [Pull request #1985](https://github.com/gazebosim/gz-sim/pull/1985)

1. Speed up Resource Spawner load time by fetching model list asynchronously
    * [Pull request #1962](https://github.com/gazebosim/gz-sim/pull/1962)

1. Use ignition::gazebo:: in class instantiation
    * [Pull request #1967](https://github.com/gazebosim/gz-sim/pull/1967)

1. Add missing cmake exports from core library
    * [Pull request #1978](https://github.com/gazebosim/gz-sim/pull/1978)

1. Add tutorial on migrating the Sensor class from gazebo classic
    * [Pull request #1930](https://github.com/gazebosim/gz-sim/pull/1930)

1. Add tutorial on migrating the Actor class from gazebo classic
    * [Pull request #1929](https://github.com/gazebosim/gz-sim/pull/1929)

1. Fix use of actors that only has trajectory animation
    * [Pull request #1947](https://github.com/gazebosim/gz-sim/pull/1947)

1. Add tutorial on migrating the Joint class from gazebo classic
    * [Pull request #1925](https://github.com/gazebosim/gz-sim/pull/1925)

1. Add tutorial on migrating the Light class from gazebo classic
    * [Pull request #1931](https://github.com/gazebosim/gz-sim/pull/1931)

1. Infrastructure
    * [Pull request #1988](https://github.com/gazebosim/gz-sim/pull/1988)
    * [Pull request #1940](https://github.com/gazebosim/gz-sim/pull/1940)

1. Rename COPYING to LICENSE
    * [Pull request #1937](https://github.com/gazebosim/gz-sim/pull/1937)

1. Add Light class
    * [Pull request #1918](https://github.com/gazebosim/gz-sim/pull/1918)

1. Resolve inconsistent visibility on ign-gazebo6
    * [Pull request #1914](https://github.com/gazebosim/gz-sim/pull/1914)

1. Relax msg count check in RF comms integration test
    * [Pull request #1920](https://github.com/gazebosim/gz-sim/pull/1920)

1. Add Actor class
    * [Pull request #1913](https://github.com/gazebosim/gz-sim/pull/1913)

1. Add Sensor class
    * [Pull request #1912](https://github.com/gazebosim/gz-sim/pull/1912)

1. Allow to change camera user hfov in camera_view plugin
    * [Pull request #1807](https://github.com/gazebosim/gz-sim/pull/1807)

1. Add Joint class
    * [Pull request #1910](https://github.com/gazebosim/gz-sim/pull/1910)

1. Add SensorTopic component to rendering sensors
    * [Pull request #1908](https://github.com/gazebosim/gz-sim/pull/1908)

1. Use a queue to track component registration from mulitiple sources
    * [Pull request #1836](https://github.com/gazebosim/gz-sim/pull/1836)

1. Document behaviour changes introduced #1784
    * [Pull request #1888](https://github.com/gazebosim/gz-sim/pull/1888)

1. Partial backport of 1728
    * [Pull request #1901](https://github.com/gazebosim/gz-sim/pull/1901)

1. Fix triggered camera test by waiting for rendering / scene to be ready
    * [Pull request #1895](https://github.com/gazebosim/gz-sim/pull/1895)

1. Backport portion of #1771 to fix command-line test
    * [Pull request #1771](https://github.com/gazebosim/gz-sim/pull/1771)

1. cmdsim.rb: fix ruby syntax
    * [Pull request #1884](https://github.com/gazebosim/gz-sim/pull/1884)

1. Fix some windows warnings (C4244 and C4305)
    * [Pull request #1874](https://github.com/gazebosim/gz-sim/pull/1874)

1. Minor optimization to transform control tool
    * [Pull request #1854](https://github.com/gazebosim/gz-sim/pull/1854)

1. Inherit material cast shadows property
    * [Pull request #1856](https://github.com/gazebosim/gz-sim/pull/1856)

1. Fix record topic
    * [Pull request #1855](https://github.com/gazebosim/gz-sim/pull/1855)

1. Remove duplicate Fuel server used by ResourceSpawner
    * [Pull request #1830](https://github.com/gazebosim/gz-sim/pull/1830)

1. Re-add namespace
    * [Pull request #1826](https://github.com/gazebosim/gz-sim/pull/1826)

1. Fix QML warnings regarding binding loops
    * [Pull request #1829](https://github.com/gazebosim/gz-sim/pull/1829)

1. Update documentation on `UpdateInfo::realTime`
    * [Pull request #1817](https://github.com/gazebosim/gz-sim/pull/1817)

1. Add jennuine as GUI codeowner
    * [Pull request #1800](https://github.com/gazebosim/gz-sim/pull/1800)

1. remove PlotIcon
    * [Pull request #1658](https://github.com/gazebosim/gz-sim/pull/1658)

1. ign -> gz
    * [Pull request #1983](https://github.com/gazebosim/gz-sim/pull/1983)
    * [Pull request #1646](https://github.com/gazebosim/gz-sim/pull/1646)
    * [Pull request #1760](https://github.com/gazebosim/gz-sim/pull/1760)
    * [Pull request #1759](https://github.com/gazebosim/gz-sim/pull/1759)
    * [Pull request #1758](https://github.com/gazebosim/gz-sim/pull/1758)
    * [Pull request #1757](https://github.com/gazebosim/gz-sim/pull/1757)
    * [Pull request #1759](https://github.com/gazebosim/gz-sim/pull/1749)

1. Added collection name to About Dialog
    * [Pull request #1756](https://github.com/gazebosim/gz-sim/pull/1756)

1. Citadel: Removed warnings
    * [Pull request #1753](https://github.com/gazebosim/gz-sim/pull/1753)

1. Remove actors from screen when they are supposed to
    * [Pull request #1699](https://github.com/gazebosim/gz-sim/pull/1699)

1. Readd namespaces for Q_ARGS
    * [Pull request #1670](https://github.com/gazebosim/gz-sim/pull/1670)

1. Remove redundant namespace references
    * [Pull request #1635](https://github.com/gazebosim/gz-sim/pull/1635)


### Gazebo Sim 6.14.0 (2022-12-29)

1. Fix Ackermann plugin zero linVel turningRadius bug
    * [Pull request #1849](https://github.com/gazebosim/gz-sim/pull/1849)

1. Header guard fix for battery power load component
    * [Pull request #1846](https://github.com/gazebosim/gz-sim/pull/1846)

1. Add interface to allow systems to declare parameters
    * [Pull request #1431](https://github.com/gazebosim/gz-sim/pull/1431)

1. Adding battery consumers and extra fixes
    * [Pull request #1811](https://github.com/gazebosim/gz-sim/pull/1811)

1. Disable tests that require dartsim on windows
    * [Pull request #1840](https://github.com/gazebosim/gz-sim/pull/1840)

1. Added move camera to model service
    * [Pull request #1823](https://github.com/gazebosim/gz-sim/pull/1823)

1. Add spin box to View Angle plugin for configuring view control sensitivity
    * [Pull request #1799](https://github.com/gazebosim/gz-sim/pull/1799)

1. Sync View Angle GUI with view controller
    * [Pull request #1825](https://github.com/gazebosim/gz-sim/pull/1825)

1. Hydrodynamics flags test strengthening
    * [Pull request #1819](https://github.com/gazebosim/gz-sim/pull/1819)

1. Fixed Fortress tests related to lights
    * [Pull request #1827](https://github.com/gazebosim/gz-sim/pull/1827)

1. Allow to move to model from Angle view plugin
    * [Pull request #1810](https://github.com/gazebosim/gz-sim/pull/1810)

1. Fixed light entity number
    * [Pull request #1818](https://github.com/gazebosim/gz-sim/pull/1818)

1. Check AddBvnAnimation return value
    * [Pull request #1750](https://github.com/gazebosim/gz-sim/pull/1750)

1. Add checkbox in view angle plugin for toggling view control reference visual
    * [Pull request #1788](https://github.com/gazebosim/gz-sim/pull/1788)

1. Adds support for hydrodynamic cross terms
    * [Pull request #1784](https://github.com/gazebosim/gz-sim/pull/1784)

1. Addresses flakiness in `Hydrodynamics.VelocityTestInOil`.
    * [Pull request #1787](https://github.com/gazebosim/gz-sim/pull/1787)

1. Fix minor bugs in RFComms plugin
    * [Pull request #1743](https://github.com/gazebosim/gz-sim/pull/1743)


### Gazebo Sim 6.13.0 (2022-11-04)

1. Fix two tests on Windows
    * [Pull request #1779](https://github.com/gazebosim/ign-gazebo/pull/1779)

1. 3 to 6 20221013
    * [Pull request #1762](https://github.com/gazebosim/ign-gazebo/pull/1762)

1. Some minor changes to hydrodynamic flags test
    * [Pull request #1772](https://github.com/gazebosim/ign-gazebo/pull/1772)

1. Fix thruster integration test
    * [Pull request #1767](https://github.com/gazebosim/ign-gazebo/pull/1767)

1. Fix scene_broadcaster_system test
    * [Pull request #1766](https://github.com/gazebosim/ign-gazebo/pull/1766)

1. Script and tutorial for generating procedural datasets with Blender
    * [Pull request #1412](https://github.com/gazebosim/ign-gazebo/pull/1412)

1. Enable use of ign gazebo -s on Windows (take two)
    * [Pull request #1764](https://github.com/gazebosim/ign-gazebo/pull/1764)

1. Removed unused speedlimit file
    * [Pull request #1761](https://github.com/gazebosim/ign-gazebo/pull/1761)

1. Fortress: Removed warnings
    * [Pull request #1754](https://github.com/gazebosim/ign-gazebo/pull/1754)

1. Enable/Disable individual hydrodynamic components.
    * [Pull request #1692](https://github.com/gazebosim/ign-gazebo/pull/1692)

1. Adding thrust coefficient calculation
    * [Pull request #1652](https://github.com/gazebosim/ign-gazebo/pull/1652)

1. Restore Add System GUI plugin
    * [Pull request #1685](https://github.com/gazebosim/ign-gazebo/pull/1685)

1. Return absolute path when finding a resource
    * [Pull request #1741](https://github.com/gazebosim/ign-gazebo/pull/1741)

1. Adds sky cubemap URI to the sky.proto's header
    * [Pull request #1739](https://github.com/gazebosim/ign-gazebo/pull/1739)

1. Update triggered_publisher.sdf
    * [Pull request #1737](https://github.com/gazebosim/ign-gazebo/pull/1737)

1. Add ResourceSpawner example file
    * [Pull request #1701](https://github.com/gazebosim/ign-gazebo/pull/1701)

1. Enable inherited model topic name.
    * [Pull request #1689](https://github.com/gazebosim/ign-gazebo/pull/1689)

1. Fix loading render engine plugins in GUI
    * [Pull request #1694](https://github.com/gazebosim/ign-gazebo/pull/1694)

1. Add a service to trigger functionality
    * [Pull request #1611](https://github.com/gazebosim/ign-gazebo/pull/1611)

1. Fix installation instructions on Ubuntu 22.04
    * [Pull request #1686](https://github.com/gazebosim/ign-gazebo/pull/1686)

1. Fix reference link in ackermann steering
    * [Pull request #1683](https://github.com/gazebosim/ign-gazebo/pull/1683)

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

### Gazebo Sim 6.9.0 (2022-04-14)

1. Add new `RFComms` system
    * [Pull request #1428](https://github.com/gazebosim/gz-sim/pull/1428)

1. Add comms infrastructure
    * [Pull request #1416](https://github.com/gazebosim/gz-sim/pull/1416)

1. Fix CMake version examples and bump plugin version
    * [Pull request #1442](https://github.com/gazebosim/gz-sim/pull/1442)

1. Make sure pose publisher creates valid pose topics
    * [Pull request #1433](https://github.com/gazebosim/gz-sim/pull/1433)

1. Add Ubuntu Jammy CI
    * [Pull request #1418](https://github.com/gazebosim/gz-sim/pull/1418)

1. Removed `screenToPlane` method and use `rendering::screenToPlane`
    * [Pull request #1432](https://github.com/gazebosim/gz-sim/pull/1432)

1. Supply world frame orientation and heading to IMU sensor (#1427)
    * [Pull request #1427](https://github.com/gazebosim/gz-sim/pull/1427)

1. Add desktop entry and SVG logo
    * [Pull request #1411](https://github.com/gazebosim/gz-sim/pull/1411)
    * [Pull request #1430](https://github.com/gazebosim/gz-sim/pull/1430)

1. Fix segfault at exit
    * [Pull request #1317](https://github.com/gazebosim/gz-sim/pull/1317)

1. Add Gaussian noise to Odometry Publisher
    * [Pull request #1393](https://github.com/gazebosim/gz-sim/pull/1393)

### Gazebo Sim 6.8.0 (2022-04-04)

1. ServerConfig accepts an sdf::Root DOM object
    * [Pull request #1333](https://github.com/gazebosim/gz-sim/pull/1333)

1. Disable sensors in sensors system when battery is drained
    * [Pull request #1385](https://github.com/gazebosim/gz-sim/pull/1385)

1. Referring to Fuel assets within a heightmap
    * [Pull request #1419](https://github.com/gazebosim/gz-sim/pull/1419)

1. Add the Model Photo Shoot system, port of Modelpropshop plugin from Gazebo classic
    * [Pull request #1331](https://github.com/gazebosim/gz-sim/pull/1331)

1. Distortion camera integration test
    * [Pull request #1374](https://github.com/gazebosim/gz-sim/pull/1374)

1. Add wheel slip user command
    * [Pull request #1241](https://github.com/gazebosim/gz-sim/pull/1241)

1. SceneBroadcaster: only send changed state information for change events
    * [Pull request #1392](https://github.com/gazebosim/gz-sim/pull/1392)

1. Fortress: Install Ogre 2.2, simplify docker
    * [Pull request #1395](https://github.com/gazebosim/gz-sim/pull/1395)

1. Disable tests that are expected to fail on Windows
    * [Pull request #1408](https://github.com/gazebosim/gz-sim/pull/1408)

1. Added user command to set multiple entities
    * [Pull request #1394](https://github.com/gazebosim/gz-sim/pull/1394)

1. Fix JointStatePublisher topic name for nested models
    * [Pull request #1405](https://github.com/gazebosim/gz-sim/pull/1405)

1. add initial_position param to joint controller system
    * [Pull request #1406](https://github.com/gazebosim/gz-sim/pull/1406)

1. Component inspector: refactor Pose3d C++ code into a separate class
    * [Pull request #1400](https://github.com/gazebosim/gz-sim/pull/1400)

1. Prevent hanging when world has only non-world plugins
    * [Pull request #1383](https://github.com/gazebosim/gz-sim/pull/1383)

1. Toggle Light visuals
    * [Pull request #1387](https://github.com/gazebosim/gz-sim/pull/1387)

1. Disable PeerTracker.PeerTrackerStale on macOS
    * [Pull request #1398](https://github.com/gazebosim/gz-sim/pull/1398)

1. Disable ModelCommandAPI_TEST.RgbdCameraSensor on macOS
    * [Pull request #1397](https://github.com/gazebosim/gz-sim/pull/1397)

1. Don't mark entities with a ComponentState::NoChange component as modified
    * [Pull request #1391](https://github.com/gazebosim/gz-sim/pull/1391)

1. Add gazebo Entity id to rendering sensor's user data
    * [Pull request #1381](https://github.com/gazebosim/gz-sim/pull/1381)

1. Allow to turn on/off lights
    * [Pull request #1343](https://github.com/gazebosim/gz-sim/pull/1343)

1. Added headless rendering tutorial
    * [Pull request #1386](https://github.com/gazebosim/gz-sim/pull/1386)

1. Add xyz and rpy offset to published odometry pose
    * [Pull request #1341](https://github.com/gazebosim/gz-sim/pull/1341)

1. Fix visualization python tutorial
    * [Pull request #1377](https://github.com/gazebosim/gz-sim/pull/1377)

1. Populate GUI plugins that are empty
    * [Pull request #1375](https://github.com/gazebosim/gz-sim/pull/1375)

### Gazebo Sim 6.7.0 (2022-02-24)

1. Added Python interfaces to some Gazebo Sim methods
    * [Pull request #1219](https://github.com/gazebosim/gz-sim/pull/1219)

1. Use pose multiplication instead of addition
    * [Pull request #1369](https://github.com/gazebosim/gz-sim/pull/1369)

1. Disables Failing Buoyancy Tests on Win32
    * [Pull request #1368](https://github.com/gazebosim/gz-sim/pull/1368)

1. Extend ShaderParam system to support loading different shader languages
    * [Pull request #1335](https://github.com/gazebosim/gz-sim/pull/1335)

1. Populate names of colliding entities in contact points message
    * [Pull request #1351](https://github.com/gazebosim/gz-sim/pull/1351)

1. Refactor System functionality into SystemManager
    * [Pull request #1340](https://github.com/gazebosim/gz-sim/pull/1340)

1. GzSceneManager: Prevent crash boom when inserted from menu
    * [Pull request #1371](https://github.com/gazebosim/gz-sim/pull/1371)

### Gazebo Sim 6.6.0 (2022-02-24)

1. Fix accessing empty JointPosition component in lift drag plugin
    * [Pull request #1366](https://github.com/gazebosim/gz-sim/pull/1366)

1. Add parameter to TrajectoryFollower stop rotation when bearing is reached
    * [Pull request #1349](https://github.com/gazebosim/gz-sim/pull/1349)

1. Support disabling pose publisher from publishing top level model pose
    * [Pull request #1342](https://github.com/gazebosim/gz-sim/pull/1342)

1. Added more sensor properties to scene/info topic
    * [Pull request #1344](https://github.com/gazebosim/gz-sim/pull/1344)

1. Adding ability to pause/resume the trajectory follower behavior.
    * [Pull request #1347](https://github.com/gazebosim/gz-sim/pull/1347)

1. Logs a warning if a mode is not clearly sepecified.
    * [Pull request #1307](https://github.com/gazebosim/gz-sim/pull/1307)

1. JointStatePublisher publish parent, child and axis data
    * [Pull request #1345](https://github.com/gazebosim/gz-sim/pull/1345)

1. Fixed light gui component inspector
    * [Pull request #1337](https://github.com/gazebosim/gz-sim/pull/1337)

1. Fix UNIT_SdfGenerator_TEST
    * [Pull request #1319](https://github.com/gazebosim/gz-sim/pull/1319)

1. Add elevator system
    * [Pull request #535](https://github.com/gazebosim/gz-sim/pull/535)

1. Removed unused variables in shapes plugin
    * [Pull request #1321](https://github.com/gazebosim/gz-sim/pull/1321)

### Gazebo Sim 6.5.0 (2022-02-15)

1. New trajectory follower system
    * [Pull request #1332](https://github.com/gazebosim/gz-sim/pull/1332)

1. Extend ShaderParam system to support textures
    * [Pull request #1310](https://github.com/gazebosim/gz-sim/pull/1310)

1. Adds a `Link::SetLinearVelocity()` method
    * [Pull request #1323](https://github.com/gazebosim/gz-sim/pull/1323)

1. Fix weird indentation in `Link.hh`
    * [Pull request #1324](https://github.com/gazebosim/gz-sim/pull/1324)

1. Limit thruster system's input thrust cmd
    * [Pull request #1318](https://github.com/gazebosim/gz-sim/pull/1318)

1. Load and run visual plugin (system) on GUI side
    * [Pull request #1275](https://github.com/gazebosim/gz-sim/pull/1275)

1. Log an error if JointPositionController cannot find the joint. (citadel retarget)
    * [Pull request #1314](https://github.com/gazebosim/gz-sim/pull/1314)

1. Update source install instructions
    * [Pull request #1311](https://github.com/gazebosim/gz-sim/pull/1311)

1. Document the `<topic>` option for JointPositionController.
    * [Pull request #1309](https://github.com/gazebosim/gz-sim/pull/1309)

1. Fix typo in EntityComponentManager
    * [Pull request #1304](https://github.com/gazebosim/gz-sim/pull/1304)

1. Buoyancy: fix center of volume's reference frame
    * [Pull request #1302](https://github.com/gazebosim/gz-sim/pull/1302)

1. Fix graded buoyancy problems
    * [Pull request #1297](https://github.com/gazebosim/gz-sim/pull/1297)

1. Add surface to buoyancy engine. (retarget fortress)
    * [Pull request #1298](https://github.com/gazebosim/gz-sim/pull/1298)

1. Remove EachNew calls from sensor PreUpdates
    * [Pull request #1281](https://github.com/gazebosim/gz-sim/pull/1281)

1. Prevent GzScene3D 💥 if another scene is already loaded
    * [Pull request #1294](https://github.com/gazebosim/gz-sim/pull/1294)

1. Fix various typos on API documentation
    * [Pull request #1291](https://github.com/gazebosim/gz-sim/pull/1291)

1. Optional orientation when spawning entity using spherical coordinates
    * [Pull request #1263](https://github.com/gazebosim/gz-sim/pull/1263)

1. Cleanup update call for non-rendering sensors
    * [Pull request #1282](https://github.com/gazebosim/gz-sim/pull/1282)

1. Documentation Error
    * [Pull request #1285](https://github.com/gazebosim/gz-sim/pull/1285)

1. Min and max parameters for velocity, acceleration, and jerk apply to linear and angular separately.
    * [Pull request #1229](https://github.com/gazebosim/gz-sim/pull/1229)

1. Add project() call to examples
    * [Pull request #1274](https://github.com/gazebosim/gz-sim/pull/1274)

1. Implement /server_control::stop
    * [Pull request #1240](https://github.com/gazebosim/gz-sim/pull/1240)

### Gazebo Sim 6.4.0 (2021-01-13)

1. Disable more tests on Windows
    * [Pull request #1286](https://github.com/gazebosim/gz-sim/pull/1286)

1. Adding angular acceleration to the Link class
    * [Pull request #1288](https://github.com/gazebosim/gz-sim/pull/1288)

1. Add world force
    * [Pull request #1279](https://github.com/gazebosim/gz-sim/pull/1279)

1. Add NavSat sensor (GPS)
    * [Pull request #1248](https://github.com/gazebosim/gz-sim/pull/1248)

1. Light Commands via topic
    * [Pull request #1222](https://github.com/gazebosim/gz-sim/pull/1222)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/gazebosim/gz-sim/pull/1255)

1. Add visibility to ModelEditorAddEntity to fix Windows
    * [Pull request #1246](https://github.com/gazebosim/gz-sim/pull/1246)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/gazebosim/gz-sim/pull/1194)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/gazebosim/gz-sim/pull/1224)

1. Disable user commands light test on macOS
    * [Pull request #1204](https://github.com/gazebosim/gz-sim/pull/1204)

1. Skip failing Windows tests
    * [Pull request #1205](https://github.com/gazebosim/gz-sim/pull/1205)


### Gazebo Sim 6.3.0 (2021-12-10)

1. View entity frames from the GUI
    * [Pull request #1105](https://github.com/gazebosim/gz-sim/pull/1105)

1. Model editor
    * [Pull request #1231](https://github.com/gazebosim/gz-sim/pull/1231)

1. Send state message when components are removed
    * [Pull request #1235](https://github.com/gazebosim/gz-sim/pull/1235)

1. Docker fixes for Fortress
    * [Pull request #1238](https://github.com/gazebosim/gz-sim/pull/1238)

1. Added sensor plugin to be able to visualize camera in `plane_propeller_demo.sdf`
    * [Pull request #1226](https://github.com/gazebosim/gz-sim/pull/1226)

1. Update SdfGenerator to save link and sensor data to file
    * [Pull request #1201](https://github.com/gazebosim/gz-sim/pull/1201)

1. Fix buoyancy not being applied for one iteration
    * [Pull request #1211](https://github.com/gazebosim/gz-sim/pull/1211)

1. Increase maximum values in ViewAngle widget and increase its size
    * [Pull request #1221](https://github.com/gazebosim/gz-sim/pull/1221)
    * [Pull request #1239](https://github.com/gazebosim/gz-sim/pull/1239)

1. Fix the force-torque sensor update rate
    * [Pull request #1159](https://github.com/gazebosim/gz-sim/pull/1159)

### Gazebo Sim 6.2.0 (2021-11-16)

1. Configurable joint state publisher's topic
    * [Pull request #1076](https://github.com/gazebosim/gz-sim/pull/1076)

1. Thruster plugin: add tests and velocity control
    * [Pull request #1190](https://github.com/gazebosim/gz-sim/pull/1190)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/gazebosim/gz-sim/pull/1192)

1. Add `sdfString` to `ServerConfig`'s copy constructor.
    * [Pull request #1185](https://github.com/gazebosim/gz-sim/pull/1185)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/gazebosim/gz-sim/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/gazebosim/gz-sim/pull/847)

1. Remove bounding box when entities are removed
    * [Pull request #1053](https://github.com/gazebosim/gz-sim/pull/1053)
    * [Pull request #1213](https://github.com/gazebosim/gz-sim/pull/1213)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/gazebosim/gz-sim/pull/1181)

1.  Extend odom publisher to allow 3D
    * [Pull request #1180](https://github.com/gazebosim/gz-sim/pull/1180)

1. Support copy/paste
    * [Pull request #1013](https://github.com/gazebosim/gz-sim/pull/1013)

1. Tweaks install instructions
    * [Pull request #1078](https://github.com/gazebosim/gz-sim/pull/1078)

1. Publish 10 world stats msgs/sec instead of 5
    * [Pull request #1163](https://github.com/gazebosim/gz-sim/pull/1163)

1. Add functionality to add entities via the entity tree
    * [Pull request #1101](https://github.com/gazebosim/gz-sim/pull/1101)

1. Get updated GUI ECM info when a user presses 'play'
    * [Pull request #1109](https://github.com/gazebosim/gz-sim/pull/1109)

1. Create expanding type header to reduce code duplication
    * [Pull request #1169](https://github.com/gazebosim/gz-sim/pull/1169)

1. `minimal_scene.sdf` example: add `camera_clip` params
    * [Pull request #1166](https://github.com/gazebosim/gz-sim/pull/1166)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/gazebosim/gz-sim/pull/1104)

1. Support printing sensors using `gz model`
    * [Pull request #1157](https://github.com/gazebosim/gz-sim/pull/1157)

1. Set camera clipping plane distances from the GUI
    * [Pull request #1162](https://github.com/gazebosim/gz-sim/pull/1162)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/gazebosim/gz-sim/pull/1160)

1. Add a default value for `isHeadlessRendering`.
    * [Pull request #1151](https://github.com/gazebosim/gz-sim/pull/1151)

1. Component inspector

    1. Edit material colors
        * [Pull request #1123](https://github.com/gazebosim/gz-sim/pull/1123)
        * [Pull request #1186](https://github.com/gazebosim/gz-sim/pull/1186)

    1. Fix integers and floats
        * [Pull request #1143](https://github.com/gazebosim/gz-sim/pull/1143)

    1. Prevent a segfault when updating
        * [Pull request #1167](https://github.com/gazebosim/gz-sim/pull/1167)

    1. Use `uint64_t` for Entity IDs
        * [Pull request #1144](https://github.com/gazebosim/gz-sim/pull/1144)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/gazebosim/gz-sim/pull/1147)

1. Select top level entity not visual
    * [Pull request #1150](https://github.com/gazebosim/gz-sim/pull/1150)

1. Update create entity offset on GUI side
    * [Pull request #1145](https://github.com/gazebosim/gz-sim/pull/1145)

1. Update Select Entities GUI plugin to use Entity type
    * [Pull request #1146](https://github.com/gazebosim/gz-sim/pull/1146)

1. Notify other GUI plugins of added/removed entities via GUI events
    * [Pull request #1138](https://github.com/gazebosim/gz-sim/pull/1138)
    * [Pull request #1213](https://github.com/gazebosim/gz-sim/pull/1213)

### Gazebo Sim 6.1.0 (2021-10-25)

1. Updates to camera video record from subt
    * [Pull request #1117](https://github.com/gazebosim/gz-sim/pull/1117)

1. Use the actor tension parameter
    * [Pull request #1091](https://github.com/gazebosim/gz-sim/pull/1091)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull request #1119](https://github.com/gazebosim/gz-sim/pull/1119)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/gazebosim/gz-sim/pull/1095)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/gazebosim/gz-sim/pull/1089)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/gazebosim/gz-sim/pull/1098)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/gazebosim/gz-sim/pull/1043)

1. Prevent crash and print error
    * [Pull request #1099](https://github.com/gazebosim/gz-sim/pull/1099)

1. Create GUI config folder before copying config
    * [Pull request #1092](https://github.com/gazebosim/gz-sim/pull/1092)

1. Add support for configuring point size in Visualize Lidar GUI plugin
    * [Pull request #1021](https://github.com/gazebosim/gz-sim/pull/1021)

1. Set a cloned joint's parent/child link names to the cloned parent/child link names
    * [Pull request #1075](https://github.com/gazebosim/gz-sim/pull/1075)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/gazebosim/gz-sim/pull/1083)

1. Fix transform controls
    * [Pull request #1081](https://github.com/gazebosim/gz-sim/pull/1081)

1. Fix View Angle's home button
    * [Pull request #1082](https://github.com/gazebosim/gz-sim/pull/1082)

1. Fix light control standalone example
    * [Pull request #1077](https://github.com/gazebosim/gz-sim/pull/1077)

1. Parse new param for enabling / disabling IMU orientation output
    * [Pull request #899](https://github.com/gazebosim/gz-sim/pull/899)

### Gazebo Sim 6.0.0 (2021-10-01)

1. Deprecated GzScene3D in favor of MinimalScene
    * [Pull request #1065](https://github.com/gazebosim/gz-sim/pull/1065)
    * [Pull request #1051](https://github.com/gazebosim/gz-sim/pull/1051)
    * [Pull request #1014](https://github.com/gazebosim/gz-sim/pull/1014)
    * [Pull request #1034](https://github.com/gazebosim/gz-sim/pull/1034)
    * [Pull request #900](https://github.com/gazebosim/gz-sim/pull/900)
    * [Pull request #988](https://github.com/gazebosim/gz-sim/pull/988)
    * [Pull request #1016](https://github.com/gazebosim/gz-sim/pull/1016)
    * [Pull request #983](https://github.com/gazebosim/gz-sim/pull/983)
    * [Pull request #854](https://github.com/gazebosim/gz-sim/pull/854)
    * [Pull request #813](https://github.com/gazebosim/gz-sim/pull/813)
    * [Pull request #905](https://github.com/gazebosim/gz-sim/pull/905)

1. Fix GuiRunner initial state and entity spawn timing issue
    * [Pull request #1073](https://github.com/gazebosim/gz-sim/pull/1073)

1. Buoyancy plugin upgrade
    * [Pull request #818](https://github.com/gazebosim/gz-sim/pull/818)
    * [Pull request #1067](https://github.com/gazebosim/gz-sim/pull/1067)
    * [Pull request #1064](https://github.com/gazebosim/gz-sim/pull/1064)

1. Fix non desired window opening alongside Gazebo GUI
    * [Pull request #1063](https://github.com/gazebosim/gz-sim/pull/1063)

1. Documentation
    * [Pull request #1074](https://github.com/gazebosim/gz-sim/pull/1074)
    * [Pull request #996](https://github.com/gazebosim/gz-sim/pull/996)

1. Update to latest SDFormat changes
    * [Pull request #1069](https://github.com/gazebosim/gz-sim/pull/1069)
    * [Pull request #1023](https://github.com/gazebosim/gz-sim/pull/1023)

1. Suppress missing canonical link error messages for static models
    * [Pull request #1068](https://github.com/gazebosim/gz-sim/pull/1068)

1. Heightmap fixes
    * [Pull request #1055](https://github.com/gazebosim/gz-sim/pull/1055)
    * [Pull request #1054](https://github.com/gazebosim/gz-sim/pull/1054)

1. Place config files in a versioned directory
    * [Pull request #1050](https://github.com/gazebosim/gz-sim/pull/1050)
    * [Pull request #1070](https://github.com/gazebosim/gz-sim/pull/1070)

1. Fix GUI crash when accessing bad rendering UserData
    * [Pull request #1052](https://github.com/gazebosim/gz-sim/pull/1052)

1. Fix performance issue with contact data and AABB updates
    * [Pull request #1048](https://github.com/gazebosim/gz-sim/pull/1048)

1. Enable new policy to fix protobuf compilation errors
    * [Pull request #1046](https://github.com/gazebosim/gz-sim/pull/1046)

1. Support locked entities, and headless video recording using sim time
    * [Pull request #862](https://github.com/gazebosim/gz-sim/pull/862)

1. Label Component & System, segmentation camera support
    * [Pull request #853](https://github.com/gazebosim/gz-sim/pull/853)
    * [Pull request #1047](https://github.com/gazebosim/gz-sim/pull/1047)

1. Joint Force-Torque Systems Plugin
    * [Pull request #977](https://github.com/gazebosim/gz-sim/pull/977)

1. Add support for cloning entities
    * [Pull request #959](https://github.com/gazebosim/gz-sim/pull/959)

1. 🌐 Spherical coordinates
    * [Pull request #1008](https://github.com/gazebosim/gz-sim/pull/1008)

1. Populate JointConstraintWrench from physics
    * [Pull request #989](https://github.com/gazebosim/gz-sim/pull/989)

1. Buoyancy engine
    * [Pull request #1009](https://github.com/gazebosim/gz-sim/pull/1009)

1. Infrastructure
    * [Pull request #1033](https://github.com/gazebosim/gz-sim/pull/1033)
    * [Pull request #1029](https://github.com/gazebosim/gz-sim/pull/1029)
    * [Pull request #991](https://github.com/gazebosim/gz-sim/pull/991)
    * [Pull request #809](https://github.com/gazebosim/gz-sim/pull/809)

1. Update on resize instead of pre-render / render
    * [Pull request #1028](https://github.com/gazebosim/gz-sim/pull/1028)

1. Add a flag to force headless rendering mode
    * [Pull request #701](https://github.com/gazebosim/gz-sim/pull/701)

1. Remove unused Gazebo GUI header
    * [Pull request #1026](https://github.com/gazebosim/gz-sim/pull/1026)

1. Adds velocity control to JointPositionController.
    * [Pull request #1003](https://github.com/gazebosim/gz-sim/pull/1003)

1. Collada world exporter now exporting lights
    * [Pull request #912](https://github.com/gazebosim/gz-sim/pull/912)

1. Workaround for setting visual cast shadows without material
    * [Pull request #1015](https://github.com/gazebosim/gz-sim/pull/1015)

1. Fix selection buffer crash on resize
    * [Pull request #969](https://github.com/gazebosim/gz-sim/pull/969)

1. Remove extra xml version line in pendulum_links example world
    * [Pull request #1002](https://github.com/gazebosim/gz-sim/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/gazebosim/gz-sim/pull/982)

1. Add ESC to unselect entities in select entities plugin
    * [Pull request #995](https://github.com/gazebosim/gz-sim/pull/995)

1. Visualize joints
    * [Pull request #961](https://github.com/gazebosim/gz-sim/pull/961)

1. Deprecate particle emitter, and use scatter ratio in new particle mes…
    * [Pull request #986](https://github.com/gazebosim/gz-sim/pull/986)

1. Removed unused variable in Shapes plugin
    * [Pull request #984](https://github.com/gazebosim/gz-sim/pull/984)

1. Use root.Model()
    * [Pull request #980](https://github.com/gazebosim/gz-sim/pull/980)

1. Add ModelSDF serializer
    * [Pull request #851](https://github.com/gazebosim/gz-sim/pull/851)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/gazebosim/gz-sim/pull/974)

1. Use statically-typed views for better performance
    * [Pull request #856](https://github.com/gazebosim/gz-sim/pull/856)
    * [Pull request #1001](https://github.com/gazebosim/gz-sim/pull/1001)

1. Upgrade gz-sensors and support custom sensors
    * [Pull request #617](https://github.com/gazebosim/gz-sim/pull/617)

1. Fix entity creation console msg
    * [Pull request #972](https://github.com/gazebosim/gz-sim/pull/972)

1. Fix crash in the follow_actor example
    * [Pull request #958](https://github.com/gazebosim/gz-sim/pull/958)

1. Removed pose topic from log system
    * [Pull request #839](https://github.com/gazebosim/gz-sim/pull/839)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/gazebosim/gz-sim/pull/965)

1. Complaint if Joint doesn't exists before adding joint controller
    * [Pull request #786](https://github.com/gazebosim/gz-sim/pull/786)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/gazebosim/gz-sim/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/gazebosim/gz-sim/pull/724)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/gazebosim/gz-sim/pull/693)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/gazebosim/gz-sim/pull/596)

1.  Add missing GZ_SIM_VISIBLE macros
    * [Pull request #563](https://github.com/gazebosim/gz-sim/pull/563)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/gazebosim/gz-sim/pull/564)

1. No install apt recommends and clear cache
    * [Pull request #423](https://github.com/gazebosim/gz-sim/pull/423)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/gazebosim/gz-sim/pull/936)

1. Fix tests that use multiple root level models or lights
    * [Pull request #931](https://github.com/gazebosim/gz-sim/pull/931)

1. Make Gazebo Sim aware of SetCameraPassCountPerGpuFlush
    * [Pull request #921](https://github.com/gazebosim/gz-sim/pull/921)

1. Visualize center of mass
    * [Pull request #903](https://github.com/gazebosim/gz-sim/pull/903)

1. Transparent mode
    * [Pull request #878](https://github.com/gazebosim/gz-sim/pull/878)

1. Visualize inertia
    * [Pull request #861](https://github.com/gazebosim/gz-sim/pull/861)

1. Remove deprecations: tock 🕑
    * [Pull request #875](https://github.com/gazebosim/gz-sim/pull/875)

1. Removed and moved tape measure and grid config to gz-gui
    * [Pull request #870](https://github.com/gazebosim/gz-sim/pull/870)

1. Update wireframe visualization to support nested models
    * [Pull request #832](https://github.com/gazebosim/gz-sim/pull/832)

1. Multi-LRAUV Swimming Race Example
    * [Pull request #841](https://github.com/gazebosim/gz-sim/pull/841)

1. Add view control gui plugin and support orthographic view
    * [Pull request #815](https://github.com/gazebosim/gz-sim/pull/815)

1. Wireframe mode
    * [Pull request #816](https://github.com/gazebosim/gz-sim/pull/816)

1. Explain why detail::View symbols are visible
    * [Pull request #788](https://github.com/gazebosim/gz-sim/pull/788)

1. Bump dependencies in fortress
    * [Pull request #764](https://github.com/gazebosim/gz-sim/pull/764)

## Gazebo Sim 5.x

### Gazebo Sim 5.4.0 (2022-03-31)

1. Add the Model Photo Shoot system, port of Modelpropshop plugin from Gazebo classic
    * [Pull request #1331](https://github.com/gazebosim/gz-sim/pull/1331)

1. Add wheel slip user command
    * [Pull request #1241](https://github.com/gazebosim/gz-sim/pull/1241)

1. Added user command to set multiple entity poses
    * [Pull request #1394](https://github.com/gazebosim/gz-sim/pull/1394)

1. Component inspector: refactor Pose3d C++ code into a separate class
    * [Pull request #1400](https://github.com/gazebosim/gz-sim/pull/1400)

1. Toggle Light visuals
    * [Pull request #1387](https://github.com/gazebosim/gz-sim/pull/1387)

1. Allow to turn on/off lights
    * [Pull request #1343](https://github.com/gazebosim/gz-sim/pull/1343)

1. Added more sensor properties to scene/info topic
    * [Pull request #1344](https://github.com/gazebosim/gz-sim/pull/1344)

1. JointStatePublisher publish parent, child and axis data
    * [Pull request #1345](https://github.com/gazebosim/gz-sim/pull/1345)

1. Fixed light GUI component inspector
    * [Pull request #1337](https://github.com/gazebosim/gz-sim/pull/1337)

1. Fix `UNIT_SdfGenerator_TEST`
    * [Pull request #1319](https://github.com/gazebosim/gz-sim/pull/1319)

1. Add elevator system
    * [Pull request #535](https://github.com/gazebosim/gz-sim/pull/535)

1. Removed unused variables in shapes plugin
    * [Pull request #1321](https://github.com/gazebosim/gz-sim/pull/1321)

1. Log an error if JointPositionController cannot find the joint. (citadel retarget)
    * [Pull request #1314](https://github.com/gazebosim/gz-sim/pull/1314)

1. Buoyancy: fix center of volume's reference frame
    * [Pull request #1302](https://github.com/gazebosim/gz-sim/pull/1302)

1. Remove EachNew calls from sensor PreUpdates
    * [Pull request #1281](https://github.com/gazebosim/gz-sim/pull/1281)

1. Prevent GzScene3D 💥 if another scene is already loaded
    * [Pull request #1294](https://github.com/gazebosim/gz-sim/pull/1294)

1. Cleanup update call for non-rendering sensors
    * [Pull request #1282](https://github.com/gazebosim/gz-sim/pull/1282)

1. Documentation Error
    * [Pull request #1285](https://github.com/gazebosim/gz-sim/pull/1285)

1. Min and max parameters for velocity, acceleration, and jerk apply to linear and angular separately.
    * [Pull request #1229](https://github.com/gazebosim/gz-sim/pull/1229)

1. Add project() call to examples
    * [Pull request #1274](https://github.com/gazebosim/gz-sim/pull/1274)

1. Implement `/server_control::stop`
    * [Pull request #1240](https://github.com/gazebosim/gz-sim/pull/1240)

1. 👩‍🌾 Make depth camera tests more robust (#897)
    * [Pull request #897) (#1257](https://github.com/gazebosim/gz-sim/pull/897) (#1257)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/gazebosim/gz-sim/pull/1255)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/gazebosim/gz-sim/pull/1194)
    * [Pull request #1250](https://github.com/gazebosim/gz-sim/pull/1250)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/gazebosim/gz-sim/pull/1224)

1. Skip failing Windows tests
    * [Pull request #1205](https://github.com/gazebosim/gz-sim/pull/1205)
    * [Pull request #1204](https://github.com/gazebosim/gz-sim/pull/1204)
    * [Pull request #1259](https://github.com/gazebosim/gz-sim/pull/1259)
    * [Pull request #1408](https://github.com/gazebosim/gz-sim/pull/1408)

1. Configurable joint state publisher's topic
    * [Pull request #1076](https://github.com/gazebosim/gz-sim/pull/1076)

1. Thruster plugin: add tests and velocity control
    * [Pull request #1190](https://github.com/gazebosim/gz-sim/pull/1190)

1. Limit thruster system's input thrust cmd
    * [Pull request #1318](https://github.com/gazebosim/gz-sim/pull/1318)

### Gazebo Sim 5.3.0 (2021-11-12)

1. Prevent creation of spurious <plugin> elements when saving worlds
    * [Pull request #1192](https://github.com/gazebosim/gz-sim/pull/1192)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/gazebosim/gz-sim/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/gazebosim/gz-sim/pull/847)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/gazebosim/gz-sim/pull/1181)

1.  Extend odom publisher to allow 3D
    * [Pull request #1180](https://github.com/gazebosim/gz-sim/pull/1180)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1131](https://github.com/gazebosim/gz-sim/pull/1131)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/gazebosim/gz-sim/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/gazebosim/gz-sim/pull/1160)

1. Edit material colors in component inspector
    * [Pull request #1123](https://github.com/gazebosim/gz-sim/pull/1123)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/gazebosim/gz-sim/pull/1147)

1. Use `uint64_t` for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/gazebosim/gz-sim/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/gazebosim/gz-sim/pull/1143)

### Gazebo Sim 5.2.0 (2021-10-22)

1. Fix performance level test flakiness
    * [Pull request #1129](https://github.com/gazebosim/gz-sim/pull/1129)

1. Updates to camera video record from subt
    * [Pull request #1117](https://github.com/gazebosim/gz-sim/pull/1117)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull request #1119](https://github.com/gazebosim/gz-sim/pull/1119)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/gazebosim/gz-sim/pull/1095)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/gazebosim/gz-sim/pull/1089)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/gazebosim/gz-sim/pull/1098)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/gazebosim/gz-sim/pull/1043)

1. Cache top level and static to speed up physics system (Backport #656)
    * [Pull request #993](https://github.com/gazebosim/gz-sim/pull/993)

1. Prevent crash and print error
    * [Pull request #1099](https://github.com/gazebosim/gz-sim/pull/1099)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/gazebosim/gz-sim/pull/1083)

1. Fix light control standalone example
    * [Pull request #1077](https://github.com/gazebosim/gz-sim/pull/1077)

1. Parse new param for enabling / disabling IMU orientation output
    * [Pull request #899](https://github.com/gazebosim/gz-sim/pull/899)

1. Enable new policy to fix protobuf compilation errors
    * [Pull request #1059](https://github.com/gazebosim/gz-sim/pull/1059)

1. Fix performance issue with contact data and AABB updates
    * [Pull request #1048](https://github.com/gazebosim/gz-sim/pull/1048)

1. Support locked entities, and headless video recording using sim time
    * [Pull request #862](https://github.com/gazebosim/gz-sim/pull/862)

1. Update gz-sim4 changelog
    * [Pull request #1031](https://github.com/gazebosim/gz-sim/pull/1031)

1. bump version and update changelog
    * [Pull request #1029](https://github.com/gazebosim/gz-sim/pull/1029)

1. Remove unused Gazebo GUI header
    * [Pull request #1026](https://github.com/gazebosim/gz-sim/pull/1026)

1. Collada world exporter now exporting lights
    * [Pull request #912](https://github.com/gazebosim/gz-sim/pull/912)

1. Fixed GUI's ComponentInspector light parameter
    * [Pull request #1018](https://github.com/gazebosim/gz-sim/pull/1018)

1. Workaround for setting visual cast shadows without material
    * [Pull request #1015](https://github.com/gazebosim/gz-sim/pull/1015)

1. Fix selection buffer crash on resize
    * [Pull request #969](https://github.com/gazebosim/gz-sim/pull/969)

1. Update DART deps to local
    * [Pull request #1005](https://github.com/gazebosim/gz-sim/pull/1005)

1. Remove extra xml version line in pendulum_links example world
    * [Pull request #1002](https://github.com/gazebosim/gz-sim/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/gazebosim/gz-sim/pull/982)

1. Make thermal sensor test more robust
    * [Pull request #994](https://github.com/gazebosim/gz-sim/pull/994)

1. Improved doxygen
    * [Pull request #996](https://github.com/gazebosim/gz-sim/pull/996)

1. Remove bitbucket-pipelines.yml
    * [Pull request #991](https://github.com/gazebosim/gz-sim/pull/991)

1. Removed unused variable in Shapes plugin
    * [Pull request #984](https://github.com/gazebosim/gz-sim/pull/984)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/gazebosim/gz-sim/pull/974)

1. Updates when forward-porting to v4
    * [Pull request #973](https://github.com/gazebosim/gz-sim/pull/973)

1. Don't use $HOME on most tests (InternalFixture)
    * [Pull request #971](https://github.com/gazebosim/gz-sim/pull/971)

1. Fix entity creation console msg
    * [Pull request #972](https://github.com/gazebosim/gz-sim/pull/972)

1. Fix crash in the follow_actor example
    * [Pull request #958](https://github.com/gazebosim/gz-sim/pull/958)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/gazebosim/gz-sim/pull/965)

1. Drag and drop meshes into scene
    * [Pull request #939](https://github.com/gazebosim/gz-sim/pull/939)

1. Allow referencing links in nested models in LiftDrag
    * [Pull request #955](https://github.com/gazebosim/gz-sim/pull/955)

1. Complaint if Joint doesn't exists before adding joint controller
    * [Pull request #786](https://github.com/gazebosim/gz-sim/pull/786)

1. Set protobuf_MODULE_COMPATIBLE before any find_package call
    * [Pull request #957](https://github.com/gazebosim/gz-sim/pull/957)

1. DiffDrive add enable/disable
    * [Pull request #772](https://github.com/gazebosim/gz-sim/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/gazebosim/gz-sim/pull/724)

1. Add UserCommands Plugin.
    * [Pull request #719](https://github.com/gazebosim/gz-sim/pull/719)

1. Expose a test fixture helper class
    * [Pull request #926](https://github.com/gazebosim/gz-sim/pull/926)

1. Fix logic to disable server default plugins loading
    * [Pull request #953](https://github.com/gazebosim/gz-sim/pull/953)

1. Porting Dome to Edifice: Windows, deprecations
    * [Pull request #948](https://github.com/gazebosim/gz-sim/pull/948)

1. removed unneeded plugin update
    * [Pull request #944](https://github.com/gazebosim/gz-sim/pull/944)

1. Functions to enable velocity and acceleration checks on Link
    * [Pull request #935](https://github.com/gazebosim/gz-sim/pull/935)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/gazebosim/gz-sim/pull/936)

1. 3D plot GUI plugin
    * [Pull request #917](https://github.com/gazebosim/gz-sim/pull/917)

1. 4 to 5
    * [Pull request #938](https://github.com/gazebosim/gz-sim/pull/938)

1. Fix joint controller without joint vel data
    * [Pull request #937](https://github.com/gazebosim/gz-sim/pull/937)

1. 3 to 4
    * [Pull request #933](https://github.com/gazebosim/gz-sim/pull/933)

1. Model info CLI `gz model`
    * [Pull request #893](https://github.com/gazebosim/gz-sim/pull/893)

1. Support Bullet on Edifice
    * [Pull request #919](https://github.com/gazebosim/gz-sim/pull/919)

1. Don't create components for entities that don't exist
    * [Pull request #927](https://github.com/gazebosim/gz-sim/pull/927)

1. Fix blender sdf export script and remove .material file from collada light export test
    * [Pull request #923](https://github.com/gazebosim/gz-sim/pull/923)

1. Heightmap physics (with DART)
    * [Pull request #661](https://github.com/gazebosim/gz-sim/pull/661)

1. Adds Mesh Tutorial
    * [Pull request #915](https://github.com/gazebosim/gz-sim/pull/915)

1. 4 to 5
    * [Pull request #918](https://github.com/gazebosim/gz-sim/pull/918)

1. Fix updating GUI plugin on load
    * [Pull request #904](https://github.com/gazebosim/gz-sim/pull/904)

1. 3 to 4
    * [Pull request #916](https://github.com/gazebosim/gz-sim/pull/916)

1. Physics system: update link poses if the canonical link pose has been updated
    * [Pull request #876](https://github.com/gazebosim/gz-sim/pull/876)

1. Add blender sdf export tutorial
    * [Pull request #895](https://github.com/gazebosim/gz-sim/pull/895)

1. Banana for Scale
    * [Pull request #734](https://github.com/gazebosim/gz-sim/pull/734)

1. Fix textures not exporting after loading a world that uses obj models
    * [Pull request #874](https://github.com/gazebosim/gz-sim/pull/874)

1. Fix documentation for the Sensor component
    * [Pull request #898](https://github.com/gazebosim/gz-sim/pull/898)

1. Make depth camera tests more robust
    * [Pull request #897](https://github.com/gazebosim/gz-sim/pull/897)

1. Use UINT64_MAX for kComponentTpyeIDInvalid instead of relying on underflow
    * [Pull request #889](https://github.com/gazebosim/gz-sim/pull/889)

1. Fix mouse view control target position
    * [Pull request #879](https://github.com/gazebosim/gz-sim/pull/879)

### Gazebo Sim 5.1.0 (2021-06-29)

1. Depend on SDF 11.2.1, rendering 5.1 and GUI 5.1. Fix Windows.
    * [Pull request #877](https://github.com/gazebosim/gz-sim/pull/877)

1. Set gui camera pose
    * [Pull request #863](https://github.com/gazebosim/gz-sim/pull/863)

1. Refactor RenderUtil::Update with helper functions
    * [Pull request #858](https://github.com/gazebosim/gz-sim/pull/858)

1. Enables confirmation dialog when closing Gazebo.
    * [Pull request #850](https://github.com/gazebosim/gz-sim/pull/850)

1. Using math::SpeedLimiter on the diff_drive controller.
    * [Pull request #833](https://github.com/gazebosim/gz-sim/pull/833)

1. New example: get an ECM snapshot from an external program
    * [Pull request #859](https://github.com/gazebosim/gz-sim/pull/859)

1. Fix WindEffects Plugin bug, not configuring new links
    * [Pull request #844](https://github.com/gazebosim/gz-sim/pull/844)

1. Set collision detector and solver from SDF
    * [Pull request #684](https://github.com/gazebosim/gz-sim/pull/684)

1. Add Particle Emitter tutorial
    * [Pull request #860](https://github.com/gazebosim/gz-sim/pull/860)

1. Fix potentially flaky integration component test case
    * [Pull request #848](https://github.com/gazebosim/gz-sim/pull/848)

1. Added follow camera offset service
    * [Pull request #855](https://github.com/gazebosim/gz-sim/pull/855)

1. Remove unneeded camera follow offset checks
    * [Pull request #857](https://github.com/gazebosim/gz-sim/pull/857)

1. Using math::SpeedLimiter on the ackermann_steering controller.
    * [Pull request #837](https://github.com/gazebosim/gz-sim/pull/837)

1. Cleanup and alphabetize plugin headers
    * [Pull request #838](https://github.com/gazebosim/gz-sim/pull/838)

1. Fix race condition when rendering the UI
    * [Pull request #774](https://github.com/gazebosim/gz-sim/pull/774)

1. Removed duplicated code with rendering::sceneFromFirstRenderEngine
    * [Pull request #819](https://github.com/gazebosim/gz-sim/pull/819)

1. Remove unused headers in video_recoder plugin
    * [Pull request #834](https://github.com/gazebosim/gz-sim/pull/834)

1. Use moveToHelper from gz-rendering
    * [Pull request #825](https://github.com/gazebosim/gz-sim/pull/825)

1. Make halt motion act like a brake
    * [Pull request #830](https://github.com/gazebosim/gz-sim/pull/830)

1. Update collision visualization to support nested models
    * [Pull request #823](https://github.com/gazebosim/gz-sim/pull/823)

1. Adds support for ocean currents
    * [Pull request #800](https://github.com/gazebosim/gz-sim/pull/800)

1. Add conversion for particle scatter ratio field
    * [Pull request #791](https://github.com/gazebosim/gz-sim/pull/791)

1. Adding HaltMotion to physics plugin
    * [Pull request #728](https://github.com/gazebosim/gz-sim/pull/728)

1. ColladaExporter, export submesh selected
    * [Pull request #802](https://github.com/gazebosim/gz-sim/pull/802)

1. Remove tools/code_check and update codecov
    * [Pull request #814](https://github.com/gazebosim/gz-sim/pull/814)

1. Trigger delay
    * [Pull request #817](https://github.com/gazebosim/gz-sim/pull/817)

1. Map canonical links to their models
    * [Pull request #736](https://github.com/gazebosim/gz-sim/pull/736)

1. Fix included nested model expansion in SDF generation
    * [Pull request #768](https://github.com/gazebosim/gz-sim/pull/768)

1. Util: Use public API from libsdformat for detecting non-file source
    * [Pull request #794](https://github.com/gazebosim/gz-sim/pull/794)

1. Contacts visualization
    * [Pull request #234](https://github.com/gazebosim/gz-sim/pull/234)

1. Bump to gz-msgs 7.1 / sdformat 11.1, Windows fixes
    * [Pull request #758](https://github.com/gazebosim/gz-sim/pull/758)

1. Add functionalities for optical tactile plugin
    * [Pull request #431](https://github.com/gazebosim/gz-sim/pull/431)

1. Fix documentation for EntityComponentManager::EachNew
    * [Pull request #795](https://github.com/gazebosim/gz-sim/pull/795)

1. Bump gz-physics version to 3.2
    * [Pull request #792](https://github.com/gazebosim/gz-sim/pull/792)

1. Prevent crash on Plotting plugin with mutex
    * [Pull request #747](https://github.com/gazebosim/gz-sim/pull/747)

1. 👩‍🌾 Fix Windows build and some warnings
    * [Pull request #782](https://github.com/gazebosim/gz-sim/pull/782)

1. Fix ColladaExporter submesh index bug
    * [Pull request #763](https://github.com/gazebosim/gz-sim/pull/763)

1. Fix macOS build: components::Name in benchmark
    * [Pull request #784](https://github.com/gazebosim/gz-sim/pull/784)

1. Feature/hydrodynamics
    * [Pull request #749](https://github.com/gazebosim/gz-sim/pull/749)

1. Don't store duplicate ComponentTypeId in ECM
    * [Pull request #751](https://github.com/gazebosim/gz-sim/pull/751)

1. [TPE] Support setting individual link velocity
    * [Pull request #427](https://github.com/gazebosim/gz-sim/pull/427)

1. 👩‍🌾 Enable Focal CI
    * [Pull request #646](https://github.com/gazebosim/gz-sim/pull/646)

1. Patch particle emitter2 service
    * [Pull request #777](https://github.com/gazebosim/gz-sim/pull/777)

1. Add odometry publisher system
    * [Pull request #547](https://github.com/gazebosim/gz-sim/pull/547)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/gazebosim/gz-sim/pull/772)

1. Update benchmark comparison instructions
    * [Pull request #766](https://github.com/gazebosim/gz-sim/pull/766)

1. Fix 'invalid animation update data' msg for actors
    * [Pull request #754](https://github.com/gazebosim/gz-sim/pull/754)

1. Fixed particle emitter forward playback
    * [Pull request #745](https://github.com/gazebosim/gz-sim/pull/745)

1. ECM's ChangedState gets message with modified components
    * [Pull request #742](https://github.com/gazebosim/gz-sim/pull/742)

1. Fixed collision visual bounding boxes
    * [Pull request #746](https://github.com/gazebosim/gz-sim/pull/746)

1. Fix compute_rtfs arguments
    * [Pull request #737](https://github.com/gazebosim/gz-sim/pull/737)

1. Validate step size and RTF parameters
    * [Pull request #740](https://github.com/gazebosim/gz-sim/pull/740)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/gazebosim/gz-sim/pull/724)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for compatibility with Protobuf CMake config
    * [Pull request #715](https://github.com/gazebosim/gz-sim/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler
    * [Pull request #716](https://github.com/gazebosim/gz-sim/pull/716)

1. Iterate through changed links only in UpdateSim
    * [Pull request #678](https://github.com/gazebosim/gz-sim/pull/678)

1. Update PlaybackScrubber description
    * [Pull request #733](https://github.com/gazebosim/gz-sim/pull/733)

1. Support configuring particle scatter ratio in particle emitter system
    * [Pull request #674](https://github.com/gazebosim/gz-sim/pull/674)

1. Fix diffuse and ambient values for ackermann example
    * [Pull request #707](https://github.com/gazebosim/gz-sim/pull/707)

1. Scenebroadcaster sensors
    * [Pull request #698](https://github.com/gazebosim/gz-sim/pull/698)

1. Add test for thermal object temperatures below 0 kelvin
    * [Pull request #621](https://github.com/gazebosim/gz-sim/pull/621)

1. [BULLET] Making GetContactsFromLastStepFeature optional in Collision Features
    * [Pull request #690](https://github.com/gazebosim/gz-sim/pull/690)

1. Make it so joint state publisher is quieter
    * [Pull request #696](https://github.com/gazebosim/gz-sim/pull/696)

### Gazebo Sim 5.0.0 (2021-03-30)

1. Added Ellipsoid and Capsule geometries
    * [Pull request #581](https://github.com/gazebosim/gz-sim/pull/581)

1. Support individual canonical links for nested models
    * [Pull request #685](https://github.com/gazebosim/gz-sim/pull/685)

1. Mecanum wheels demo
    * [Pull request #683](https://github.com/gazebosim/gz-sim/pull/683)

1. Fixed collision visual bounding boxes
    * [Pull request #702](https://github.com/gazebosim/gz-sim/pull/702)

1. Fixed material colors for ackermann sdfs
    * [Pull request #703](https://github.com/gazebosim/gz-sim/pull/703)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/gazebosim/gz-sim/pull/693)

1. Remove static for maps from Factory.hh
    * [Pull request #635](https://github.com/gazebosim/gz-sim/pull/635)

1. Depend on cli component of ignition-utils1
    * [Pull request #671](https://github.com/gazebosim/gz-sim/pull/671)

1. Support SDFormat 1.8 Composition
    * [Pull request #542](https://github.com/gazebosim/gz-sim/pull/542)

1. Deprecate TmpIface: it's leftover from prototyping
    * [Pull request #654](https://github.com/gazebosim/gz-sim/pull/654)

1. Bump in edifice: gz-common4
    * [Pull request #577](https://github.com/gazebosim/gz-sim/pull/577)

1. Plugin to spawn lights
    * [Pull request #587](https://github.com/gazebosim/gz-sim/pull/587)

1. Added light intensity
    * [Pull request #612](https://github.com/gazebosim/gz-sim/pull/612)
    * [Pull request #670](https://github.com/gazebosim/gz-sim/pull/670)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/gazebosim/gz-sim/pull/596)

1. Prepare GuiRunner to be made private
    * [Pull request #567](https://github.com/gazebosim/gz-sim/pull/567)

1. Deprecate some sim::gui events in favor of gz-gui events
    * [Pull request #595](https://github.com/gazebosim/gz-sim/pull/595)

1. Heightmap (rendering only)
    * [Pull request #487](https://github.com/gazebosim/gz-sim/pull/487)

1. Add image suffix to thermal camera topic name
    * [Pull request #606](https://github.com/gazebosim/gz-sim/pull/606)

1. Fix build with latest sdformat11 branch
    * [Pull request #607](https://github.com/gazebosim/gz-sim/pull/607)

1. Added run to time feature
    * [Pull request #478](https://github.com/gazebosim/gz-sim/pull/478)

1. Depend on ignition-utils1
    * [Pull request #591](https://github.com/gazebosim/gz-sim/pull/591)

1. Use double sided field in material msg
    * [Pull request #599](https://github.com/gazebosim/gz-sim/pull/599)

1. Add lightmap demo
    * [Pull request #471](https://github.com/gazebosim/gz-sim/pull/471)

1. Added renderOrder to convert functions
    * [Pull request #514](https://github.com/gazebosim/gz-sim/pull/514)

1. Compilation fixes for Windows
    * [Pull request #501](https://github.com/gazebosim/gz-sim/pull/501)
    * [Pull request #585](https://github.com/gazebosim/gz-sim/pull/585)
    * [Pull request #565](https://github.com/gazebosim/gz-sim/pull/565)
    * [Pull request #616](https://github.com/gazebosim/gz-sim/pull/616)
    * [Pull request #622](https://github.com/gazebosim/gz-sim/pull/622)

1. Documentation fixes
    * [Pull request #727](https://github.com/gazebosim/gz-sim/pull/727)
    * [Pull request #710](https://github.com/gazebosim/gz-sim/pull/710)

1. Replace deprecated function FreeGroup::CanonicalLink with FreeGroup::RootLink
    * [Pull request #723](https://github.com/gazebosim/gz-sim/pull/723)

1. Respect spotlight direction
    * [Pull request #718](https://github.com/gazebosim/gz-sim/pull/718)

1. Add UserCommands plugin to fuel.sdf
    * [Pull request #719](https://github.com/gazebosim/gz-sim/pull/719)

1. Change SelectedEntities to return a const ref
    * [Pull request #571](https://github.com/gazebosim/gz-sim/pull/571)

1. Use common::setenv for portability to Windows
    * [Pull request #561](https://github.com/gazebosim/gz-sim/pull/561)

1.  Add missing GZ_SIM_VISIBLE macros
    * [Pull request #563](https://github.com/gazebosim/gz-sim/pull/563)

1. Fix deprecation warnings
    * [Pull request #572](https://github.com/gazebosim/gz-sim/pull/572)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/gazebosim/gz-sim/pull/564)

1. Bump edifice sdformat11 and gz-physics4
    * [Pull request #549](https://github.com/gazebosim/gz-sim/pull/549)

1. Use ComponentState::PeriodicChange in UpdateState to avoid forcing full scene update
    * [Pull request #486](https://github.com/gazebosim/gz-sim/pull/486)

1. Bump in edifice: gz-msgs7
    * [Pull request #546](https://github.com/gazebosim/gz-sim/pull/546)

1. Add support for sky
    * [Pull request #445](https://github.com/gazebosim/gz-sim/pull/445)

1. Infrastructure
    * [Pull request #423](https://github.com/gazebosim/gz-sim/pull/423)

1. Bump in edifice: gz-rendering5
    * [Pull request #430](https://github.com/gazebosim/gz-sim/pull/430)

1. Add 25percent darker view angle icons
    * [Pull request #426](https://github.com/gazebosim/gz-sim/pull/426)

## Gazebo Sim 4.x

### Gazebo Sim 4.14.0 (2021-12-20)

1. Support battery draining start via topics
    * [Pull request #1255](https://github.com/gazebosim/gz-sim/pull/1255)

1. Make tests run as fast as possible
    * [Pull request #1194](https://github.com/gazebosim/gz-sim/pull/1194)
    * [Pull request #1250](https://github.com/gazebosim/gz-sim/pull/1250)

1. Fix visualize lidar
    * [Pull request #1224](https://github.com/gazebosim/gz-sim/pull/1224)

1. Disable user commands light test on macOS
    * [Pull request #1204](https://github.com/gazebosim/gz-sim/pull/1204)

### Gazebo Sim 4.13.0 (2021-11-15)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/gazebosim/gz-sim/pull/1192)

1. Add support for tracked vehicles
    * [Pull request #869](https://github.com/gazebosim/gz-sim/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/gazebosim/gz-sim/pull/847)

1. Fix updating component from state
    * [Pull request #1181](https://github.com/gazebosim/gz-sim/pull/1181)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1149](https://github.com/gazebosim/gz-sim/pull/1149)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/gazebosim/gz-sim/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/gazebosim/gz-sim/pull/1160)

1. Edit material colors in component inspector
    * [Pull request #1123](https://github.com/gazebosim/gz-sim/pull/1123)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/gazebosim/gz-sim/pull/1147)

1. Use uint64_t for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/gazebosim/gz-sim/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/gazebosim/gz-sim/pull/1143)

### Gazebo Sim 4.12.0 (2021-10-22)

1. Fix performance issue with contact data and AABB updates.
    * [Pull Request 1048](https://github.com/gazebosim/gz-sim/pull/1048)

1. Enable new CMake policy to fix protobuf compilation
    * [Pull Request 1059](https://github.com/gazebosim/gz-sim/pull/1059)

1. Parse new param for enabling / disabling IMU orientation output.
    * [Pull Request 899](https://github.com/gazebosim/gz-sim/pull/899)

1. Fix light control standalone example.
    * [Pull Request 1077](https://github.com/gazebosim/gz-sim/pull/1077)

1. Performance: use std::unordered_map where possible in SceneManager.
    * [Pull Request 1083](https://github.com/gazebosim/gz-sim/pull/1083)

1. Prevent crash when using <specular> workflow PBR material.
    * [Pull Request 1099](https://github.com/gazebosim/gz-sim/pull/1099)

1. JointPositionController: Improve misleading error message.
    * [Pull Request 1098](https://github.com/gazebosim/gz-sim/pull/1098)

1. Adjust pose decimals based on element width.
    * [Pull Request 1089](https://github.com/gazebosim/gz-sim/pull/1089)

1. Better protect this->dataPtr->initialized with renderMutex.
    * [Pull Request 1119](https://github.com/gazebosim/gz-sim/pull/1089)

1. Updates to camera video record from subt.
    * [Pull Request 1117](https://github.com/gazebosim/gz-sim/pull/1117)

1. Fix performance level test flakiness.
    * [Pull Request 1129](https://github.com/gazebosim/gz-sim/pull/1129)

### Gazebo Sim 4.11.0 (2021-09-23)

1. Support locked entities, and headless video recording using sim time.
    * [Pull Request 862](https://github.com/gazebosim/gz-sim/pull/862)

### Gazebo Sim 4.10.0 (2021-09-15)

1. Fixed GUI's ComponentInspector light parameter
    * [Pull Request 1018](https://github.com/gazebosim/gz-sim/pull/1018)

1. Fix msg in entity_creation example
    * [Pull Request 972](https://github.com/gazebosim/gz-sim/pull/972)

1. Fix selection buffer crash on resize
    * [Pull Request 969](https://github.com/gazebosim/gz-sim/pull/969)

1. Fix crash in the follow_actor example
    * [Pull Request 958](https://github.com/gazebosim/gz-sim/pull/958)

1. Fix joint controller with empty joint velocity data
    * [Pull Request 937](https://github.com/gazebosim/gz-sim/pull/937)

1. Scale mode - Part2
    * [Pull Request 881](https://github.com/gazebosim/gz-sim/pull/881)

1. Physics system: update link poses if the canonical link pose has been updated
    * [Pull Request 876](https://github.com/gazebosim/gz-sim/pull/876)

1. Add Particle Emitter tutorial
    * [Pull Request 860](https://github.com/gazebosim/gz-sim/pull/860)

1. Refactor RenderUtil::Update with helper functions
    * [Pull Request 858](https://github.com/gazebosim/gz-sim/pull/858)

1. Remove unneeded camera follow offset checks
    * [Pull Request 857](https://github.com/gazebosim/gz-sim/pull/857)

1. Added service to set camera's follow offset
    * [Pull Request 855](https://github.com/gazebosim/gz-sim/pull/855)

1. Using math::SpeedLimiter on the ackermann_steering controller.
    * [Pull Request 837](https://github.com/gazebosim/gz-sim/pull/837)

1. All changes merged forward from gz-sim3
    * [Pull Request 866](https://github.com/gazebosim/gz-sim/pull/866)
    * [Pull Request 916](https://github.com/gazebosim/gz-sim/pull/916)
    * [Pull Request 933](https://github.com/gazebosim/gz-sim/pull/933)
    * [Pull Request 946](https://github.com/gazebosim/gz-sim/pull/946)
    * [Pull Request 973](https://github.com/gazebosim/gz-sim/pull/973)
    * [Pull Request 1017](https://github.com/gazebosim/gz-sim/pull/1017)

### Gazebo Sim 4.9.1 (2021-05-24)

1. Make halt motion act like a brake.
    * [Pull Request 830](https://github.com/gazebosim/gz-sim/pull/830)

### Gazebo Sim 4.9.0 (2021-05-20)

1. Enable Focal CI.
    * [Pull Request 646](https://github.com/gazebosim/gz-sim/pull/646)

1. [TPE] Support setting individual link velocity.
    * [Pull Request 427](https://github.com/gazebosim/gz-sim/pull/427)

1. Don't store duplicate ComponentTypeId in ECM.
    * [Pull Request 751](https://github.com/gazebosim/gz-sim/pull/751)

1. Fix macOS build: components::Name in benchmark.
    * [Pull Request 784](https://github.com/gazebosim/gz-sim/pull/784)

1. Fix documentation for EntityComponentManager::EachNew.
    * [Pull Request 795](https://github.com/gazebosim/gz-sim/pull/795)

1. Add functionalities for optical tactile plugin.
    * [Pull Request 431](https://github.com/gazebosim/gz-sim/pull/431)

1. Visualize ContactSensorData.
    * [Pull Request 234](https://github.com/gazebosim/gz-sim/pull/234)

1. Backport PR #763.
    * [Pull Request 804](https://github.com/gazebosim/gz-sim/pull/804)

1. Backport PR #536.
    * [Pull Request 812](https://github.com/gazebosim/gz-sim/pull/812)

1. Add an optional delay to the TriggeredPublisher system.
    * [Pull Request 817](https://github.com/gazebosim/gz-sim/pull/817)

1. Remove tools/code_check and update codecov.
    * [Pull Request 814](https://github.com/gazebosim/gz-sim/pull/814)

1. add conversion for particle scatter ratio field.
    * [Pull Request 791](https://github.com/gazebosim/gz-sim/pull/791)

### Gazebo Sim 4.8.0 (2021-04-22)

1. Add odometry publisher system.
    * [Pull Request 547](https://github.com/gazebosim/gz-sim/pull/547)

1. Patch particle emitter2 service.
    * [Pull Request 777](https://github.com/gazebosim/gz-sim/pull/777)

### Gazebo Sim 4.7.0 (2021-04-09)

1. Particle emitter based on SDF.
    * [Pull Request 730](https://github.com/gazebosim/gz-sim/pull/730)

1. Fix log playback for particle emitters.
    * [Pull Request 745](https://github.com/gazebosim/gz-sim/pull/745)

1. ECM's ChangedState gets message with modified components.
    * [Pull Request 742](https://github.com/gazebosim/gz-sim/pull/742)

1. Fixed collision visual bounding boxes.
    * [Pull Request 746](https://github.com/gazebosim/gz-sim/pull/746)

1. Fix compute_rtfs arguments.
    * [Pull Request 737](https://github.com/gazebosim/gz-sim/pull/737)

1. Validate step size and RTF parameters.
    * [Pull Request 740](https://github.com/gazebosim/gz-sim/pull/740)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for
   compatibility with Protobuf CMake config.
    * [Pull Request 715](https://github.com/gazebosim/gz-sim/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler.
    * [Pull Request 716](https://github.com/gazebosim/gz-sim/pull/716)

1. Support configuring particle scatter ratio in particle emitter system.
    * [Pull Request 674](https://github.com/gazebosim/gz-sim/pull/674)

1. Fix diffuse and ambient values for ackermann example.
    * [Pull Request 707](https://github.com/gazebosim/gz-sim/pull/707)

1. Scenebroadcaster sensors.
    * [Pull Request 698](https://github.com/gazebosim/gz-sim/pull/698)

1. Add thermal camera test for object temperature below 0.
    * [Pull Request 621](https://github.com/gazebosim/gz-sim/pull/621)

1. [BULLET] Making GetContactsFromLastStepFeature optional in Collision Features
    * [Pull Request 690](https://github.com/gazebosim/gz-sim/pull/690)

1. Fix joint controller GUI test.
    * [Pull Request 697](https://github.com/gazebosim/gz-sim/pull/697)

1. Quiet warnings from Joint State Publisher.
    * [Pull Request 696](https://github.com/gazebosim/gz-sim/pull/696)

1. Ackermann Steering Plugin.
    * [Pull Request 618](https://github.com/gazebosim/gz-sim/pull/618)

1. Remove bounding box when model is deleted
    * [Pull Request 675](https://github.com/gazebosim/gz-sim/pull/675)

1. Cache link poses to improve performance.
    * [Pull Request 669](https://github.com/gazebosim/gz-sim/pull/669)

1. Check empty world name in Scene3d.
    * [Pull Request 662](https://github.com/gazebosim/gz-sim/pull/662)

1. All changes up to 3.8.0.

### Gazebo Sim 4.6.0 (2021-03-01)

1. Use a custom data structure to manage entity feature maps.
    * [Pull Request 586](https://github.com/gazebosim/gz-sim/pull/586)

1. Limit scene broadcast publications when paused.
    * [Pull Request 497](https://github.com/gazebosim/gz-sim/pull/497)

1. Report performer count in PerformerDetector plugin.
    * [Pull Request 652](https://github.com/gazebosim/gz-sim/pull/652)

1. Cache top level and static to speed up physics system.
    * [Pull Request 656](https://github.com/gazebosim/gz-sim/pull/656)

1. Support particle emitter modification using partial message.
    * [Pull Request 651](https://github.com/gazebosim/gz-sim/pull/651)

1. Set LD_LIBRARY_PATH on Actions CI.
    * [Pull Request 650](https://github.com/gazebosim/gz-sim/pull/650)

1. Fix flaky SceneBroadcaster test.
    * [Pull Request 641](https://github.com/gazebosim/gz-sim/pull/641)

1. Add a convenience function for getting possibly non-existing components.
    * [Pull Request 629](https://github.com/gazebosim/gz-sim/pull/629)

1. Add msg to show the computed temperature range computed from temperature
   gradient.
    * [Pull Request 643](https://github.com/gazebosim/gz-sim/pull/643)

1. Add TF/Pose_V pub in DiffDrive.
    * [Pull Request 548](https://github.com/gazebosim/gz-sim/pull/548)

1. Relax flaky performance test.
    * [Pull Request 640](https://github.com/gazebosim/gz-sim/pull/640)

1. Improve velocity control test.
    * [Pull Request 642](https://github.com/gazebosim/gz-sim/pull/642)

1. Validity check for user defined topics in JointPositionController.
    * [Pull Request 639](https://github.com/gazebosim/gz-sim/pull/639)

1. Add laser_retro support.
    * [Pull Request 603](https://github.com/gazebosim/gz-sim/pull/603)

1. Fix pose of plane visual with non-default normal vector.
    * [Pull Request 574](https://github.com/gazebosim/gz-sim/pull/574)

### Gazebo Sim 4.5.0 (2020-02-17)

1. Added particle system.
    * [Pull Request 516](https://github.com/gazebosim/gz-sim/pull/516)

1. Add Light Usercommand and include Light parameters in the componentInspector
    * [Pull Request 482](https://github.com/gazebosim/gz-sim/pull/482)

1. Added link to HW-accelerated video recording.
    * [Pull Request 627](https://github.com/gazebosim/gz-sim/pull/627)

1. Fix EntityComponentManager race condition.
    * [Pull Request 601](https://github.com/gazebosim/gz-sim/pull/601)

1. Add SDF topic validity check.
    * [Pull Request 632](https://github.com/gazebosim/gz-sim/pull/632)

1. Add JointTrajectoryController system plugin.
    * [Pull Request 473](https://github.com/gazebosim/gz-sim/pull/473)

### Gazebo Sim 4.4.0 (2020-02-10)

1. Added issue and PR templates
    * [Pull Request 613](https://github.com/gazebosim/gz-sim/pull/613)

1. Fix segfault in SetRemovedComponentsMsgs method
    * [Pull Request 495](https://github.com/gazebosim/gz-sim/pull/495)

1. Make topics configurable for joint controllers
    * [Pull Request 584](https://github.com/gazebosim/gz-sim/pull/584)

1. Add about dialog
    * [Pull Request 609](https://github.com/gazebosim/gz-sim/pull/609)

1. Add thermal sensor system for configuring thermal camera properties
    * [Pull Request 614](https://github.com/gazebosim/gz-sim/pull/614)

### Gazebo Sim 4.3.0 (2020-02-02)

1. Non-blocking paths request.
    * [Pull Request 555](https://github.com/gazebosim/gz-sim/pull/555)

1. Parallelize State call in ECM.
    * [Pull Request 451](https://github.com/gazebosim/gz-sim/pull/451)

1. Allow to create light with the create service.
    * [Pull Request 513](https://github.com/gazebosim/gz-sim/pull/513)

1. Added size to ground_plane in examples.
    * [Pull Request 573](https://github.com/gazebosim/gz-sim/pull/573)

1. Fix finding PBR materials.
    * [Pull Request 575](https://github.com/gazebosim/gz-sim/pull/575)

1. Publish all periodic change components in Scene Broadcaster.
    * [Pull Request 544](https://github.com/gazebosim/gz-sim/pull/544)

1. Backport state update changes from pull request [#486](https://github.com/gazebosim/gz-sim/pull/486).
    * [Pull Request 583](https://github.com/gazebosim/gz-sim/pull/583)

1. Fix code_check errors.
    * [Pull Request 582](https://github.com/gazebosim/gz-sim/pull/582)

1. Visualize collisions.
    * [Pull Request 531](https://github.com/gazebosim/gz-sim/pull/531)

1. Remove playback <path> SDF param in Dome.
    * [Pull Request 570](https://github.com/gazebosim/gz-sim/pull/570)

1. Tutorial on migrating SDF files from Gazebo classic.
    * [Pull Request 400](https://github.com/gazebosim/gz-sim/pull/400)

1. World Exporter.
    * [Pull Request 474](https://github.com/gazebosim/gz-sim/pull/474)

1. Model Creation tutorial using services.
    * [Pull Request 530](https://github.com/gazebosim/gz-sim/pull/530)

1. Fix topLevelModel Method.
    * [Pull Request 600](https://github.com/gazebosim/gz-sim/pull/600)

1. Add heat signature option to thermal system.
    * [Pull Request 498](https://github.com/gazebosim/gz-sim/pull/498)

1. Add service and GUI to configure physics parameters (step size and RTF).
    * [Pull Request 536](https://github.com/gazebosim/gz-sim/pull/536)

1. Refactor UNIT_Server_TEST.
    * [Pull Request 594](https://github.com/gazebosim/gz-sim/pull/594)

1. Use Gazebo GUI render event.
    * [Pull Request 598](https://github.com/gazebosim/gz-sim/pull/598)

### Gazebo Sim 4.2.0 (2020-01-13)

1. Automatically load a subset of world plugins.
    * [Pull Request 537](https://github.com/gazebosim/gz-sim/pull/537)

1. Fix to handle multiple logical cameras.
    * [Pull Request 539](https://github.com/gazebosim/gz-sim/pull/539)

1. Improve gz tool support on macOS.
    * [Pull Request 477](https://github.com/gazebosim/gz-sim/pull/477)

1. Add support for topic statistics on breadcrumb deployments.
    * [Pull Request 532](https://github.com/gazebosim/gz-sim/pull/532)

1. Fix slot in Plotting plugin.
    * [Pull Request 490](https://github.com/gazebosim/gz-sim/pull/490)

1. Fix shadow artifacts by disabling double sided rendering.
    * [Pull Request 446](https://github.com/gazebosim/gz-sim/pull/446)

1. Kinetic energy monitor plugin.
    * [Pull Request 492](https://github.com/gazebosim/gz-sim/pull/492)

1. Change nullptr to a int ptr for qt 5.15.2.
    * [Pull Request 527](https://github.com/gazebosim/gz-sim/pull/527)

1. Generate valid topics everywhere (support names with spaces).
    * [Pull Request 522](https://github.com/gazebosim/gz-sim/pull/522)

1. All changes up to version 3.7.0.

### Gazebo Sim 4.1.0 (2020-12-11)

1. Update Dockerfiles to use focal images
    * [pull request 388](https://github.com/gazebosim/gz-sim/pull/388)

1. Updated source build instructions for gz-sim4
    * [pull request 404](https://github.com/gazebosim/gz-sim/pull/404)

1. Add tests for the AnimationTime component
    * [pull request 433](https://github.com/gazebosim/gz-sim/pull/433)

1. Fix pose msg conversion when msg is missing orientation
    * [pull request 450](https://github.com/gazebosim/gz-sim/pull/450)
    * [pull request 459](https://github.com/gazebosim/gz-sim/pull/459)

1. Resolved updated codecheck issues
    * [pull request 443](https://github.com/gazebosim/gz-sim/pull/443)
    * [pull request 457](https://github.com/gazebosim/gz-sim/pull/457)
    * [pull request 459](https://github.com/gazebosim/gz-sim/pull/459)

1. Use new backpack version in tests
    * [pull request 455](https://github.com/gazebosim/gz-sim/pull/455)
    * [pull request 457](https://github.com/gazebosim/gz-sim/pull/457)
    * [pull request 459](https://github.com/gazebosim/gz-sim/pull/459)

1. Fix segfault in the Breadcrumb system when associated model is unloaded
    * [pull request 454](https://github.com/gazebosim/gz-sim/pull/454)
    * [pull request 457](https://github.com/gazebosim/gz-sim/pull/457)
    * [pull request 459](https://github.com/gazebosim/gz-sim/pull/459)

1. Added user commands to example thermal camera world
    * [pull request 442](https://github.com/gazebosim/gz-sim/pull/442)
    * [pull request 459](https://github.com/gazebosim/gz-sim/pull/459)

1. Helper function to set component data
    * [pull request 436](https://github.com/gazebosim/gz-sim/pull/436)
    * [pull request 469](https://github.com/gazebosim/gz-sim/pull/469)

1. Remove unneeded if statement
    * [pull request 432](https://github.com/gazebosim/gz-sim/pull/432)
    * [pull request 469](https://github.com/gazebosim/gz-sim/pull/469)

1. Fix flaky RecordAndPlayback test in INTEGRATION_log_system
    * [pull request 463](https://github.com/gazebosim/gz-sim/pull/463)
    * [pull request 469](https://github.com/gazebosim/gz-sim/pull/469)

1. Make PeerTracker test more robust
    * [pull request 452](https://github.com/gazebosim/gz-sim/pull/452)
    * [pull request 469](https://github.com/gazebosim/gz-sim/pull/469)

1. Use a [std::promise](https://en.cppreference.com/w/cpp/thread/promise)/[std::future](https://en.cppreference.com/w/cpp/thread/future) mechanism to avoid waiting in a looop until all `stepAck` messages are received
    * [pull request 470](https://github.com/gazebosim/gz-sim/pull/470)

1. Optical Tactile Sensor Plugin
    * [pull request 229](https://github.com/gazebosim/gz-sim/pull/229)

1. All changes up to and including those in version 3.5.0 and version 2.25.0

### Gazebo Sim 4.0.0 (2020-09-30)

1. Names with spaces: add string serializer
    * [pull request 244](https://github.com/gazebosim/gz-sim/pull/244)

1. Filter mesh collision based on `collide_bitmask` property
    * [pull request 160](https://github.com/gazebosim/gz-sim/pull/160)

1. Add force focus when mouse enters render window
    * [pull request 97](https://github.com/gazebosim/gz-sim/pull/97)

1. Fixed docblock showGrid
    * [pull request 152](https://github.com/gazebosim/gz-sim/pull/152)

1. More actor components and follow plugin
    * [pull request 157](https://github.com/gazebosim/gz-sim/pull/157)

1. Filter the record menu and write the format to the file according to which button the user pushed (mp4 or ogv)
    * [pull request 153](https://github.com/gazebosim/gz-sim/pull/153)

1. Fix scene manager losing header file
    * [pull request 211](https://github.com/gazebosim/gz-sim/pull/211)

1. Fixed left menu events
    * [pull request 218](https://github.com/gazebosim/gz-sim/pull/218)

1. Fix yaw units typo in Component Inspector plugin
    * [pull request 238](https://github.com/gazebosim/gz-sim/pull/238)

1. Enable alpha based transparency on PBR materials by default
    * [pull request 249](https://github.com/gazebosim/gz-sim/pull/249)

1. Qt auto scale factor for HiDPI displays
    * [pull request 291](https://github.com/gazebosim/gz-sim/pull/291)

1. Sync components removal
    * [pull request 272](https://github.com/gazebosim/gz-sim/pull/272)

1. Add error handling for JointAxis::SetXyz and remove use of use_parent_model_frame
    * [pull request 288](https://github.com/gazebosim/gz-sim/pull/288)

1. Make some tests more robust
    * [pull request 314](https://github.com/gazebosim/gz-sim/pull/314)

1. Fix Qt5 warnings for using anchors
    * [pull request 363](https://github.com/gazebosim/gz-sim/pull/363)

1. Plotting Components Plugin
    * [pull request 270](https://github.com/gazebosim/gz-sim/pull/270)

1. Visualize Lidar Plugin
    * [pull request 301](https://github.com/gazebosim/gz-sim/pull/301)
    * [pull request 391](https://github.com/gazebosim/gz-sim/pull/391)

1. Replaced common::Time for std::chrono
    * [pull request 309](https://github.com/gazebosim/gz-sim/pull/309)

1. Tutorial, examples and documentation updates
    * [pull request 380](https://github.com/gazebosim/gz-sim/pull/380)
    * [pull request 386](https://github.com/gazebosim/gz-sim/pull/386)
    * [pull request 387](https://github.com/gazebosim/gz-sim/pull/387)
    * [pull request 390](https://github.com/gazebosim/gz-sim/pull/390)

1. Migration from BitBucket to GitHub
    * [pull request 73](https://github.com/gazebosim/gz-sim/pull/73)
    * [pull request 68](https://github.com/gazebosim/gz-sim/pull/68)
    * [pull request 67](https://github.com/gazebosim/gz-sim/pull/67)
    * [pull request 130](https://github.com/gazebosim/gz-sim/pull/130)

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

1. Depend on gz-rendering4, gz-gui4, gz-sensors4
    * [BitBucket pull request 540](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/540)

1. Axis-Aligned Bounding Boxes
    * [BitBucket pull request 565](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/565)

1. Add window focus upon mouse entering the render window
    * [Github pull request 97](https://github.com/gazebosim/gz-sim/pull/97)

## Gazebo Sim 3.x

### Gazebo Sim 3.15.1 (2024-01-05)

1. Update github action workflows
    * [Pull request #2237](https://github.com/gazebosim/gz-sim/pull/2237)
    * [Pull request #1988](https://github.com/gazebosim/gz-sim/pull/1988)

1. Fix macOS test failures by registering components in the core library
    * [Pull request #2220](https://github.com/gazebosim/gz-sim/pull/2220)

1. Bump Fuel model version in test
    * [Pull request #2190](https://github.com/gazebosim/gz-sim/pull/2190)

1. Fix a minor issue in the documentation of the server API
    * [Pull request #2067](https://github.com/gazebosim/gz-sim/pull/2067)

1. Use sdf::Element::FindElement instead of GetElement in ApplyLinkWrench
    * [Pull request #2052](https://github.com/gazebosim/gz-sim/pull/2052)

1. Adds a warning if the `Server` method of a `TestFixture` is called before `Finalize`
    * [Pull request #2047](https://github.com/gazebosim/gz-sim/pull/2047)

1. Protobuf: Do not require version 3 do support Protobuf 4.23.2 (23.2)
    * [Pull request #2006](https://github.com/gazebosim/gz-sim/pull/2006)

1. Print an error message when trying to load SDF files that don't contain a `<world>`
    * [Pull request #1998](https://github.com/gazebosim/gz-sim/pull/1998)

1.  Enable GzWeb visualization of markers by republishing service requests on a topic
    * [Pull request #1994](https://github.com/gazebosim/gz-sim/pull/1994)

### Gazebo Sim 3.15.0 (2023-05-08)

1. Speed up Resource Spawner load time by fetching model list asynchronously
    * [Pull request #1962](https://github.com/gazebosim/gz-sim/pull/1962)

1. ign -> gz Migrate Ignition Headers : gz-sim
    * [Pull request #1646](https://github.com/gazebosim/gz-sim/pull/1646)
    * [Pull request #1967](https://github.com/gazebosim/gz-sim/pull/1967)
    * [Pull request #1978](https://github.com/gazebosim/gz-sim/pull/1978)
    * [Pull request #1983](https://github.com/gazebosim/gz-sim/pull/1983)
    * [Pull request #1985](https://github.com/gazebosim/gz-sim/pull/1985)

1. Infrastructure
    * [Pull request #1940](https://github.com/gazebosim/gz-sim/pull/1940)
    * [Pull request #1937](https://github.com/gazebosim/gz-sim/pull/1937)

1. Backport portion of #1771 to fix command-line test
    * [Pull request #1771](https://github.com/gazebosim/gz-sim/pull/1771)

1. cmdsim.rb: fix ruby syntax
    * [Pull request #1884](https://github.com/gazebosim/gz-sim/pull/1884)

1. Fix loading wold with record topic
    * [Pull request #1855](https://github.com/gazebosim/gz-sim/pull/1855)

1. Remove duplicate Fuel server used by ResourceSpawner
    * [Pull request #1830](https://github.com/gazebosim/gz-sim/pull/1830)

1. Re-add namespace for GUI render event
    * [Pull request #1826](https://github.com/gazebosim/gz-sim/pull/1826)

1. Fix QML warnings regarding binding loops
    * [Pull request #1829](https://github.com/gazebosim/gz-sim/pull/1829)

1. Update documentation on `UpdateInfo::realTime`
    * [Pull request #1817](https://github.com/gazebosim/gz-sim/pull/1817)

1. Add jennuine as GUI codeowner
    * [Pull request #1800](https://github.com/gazebosim/gz-sim/pull/1800)

1. Remove plotIcon in Physics.qml for Component Inspector
    * [Pull request #1658](https://github.com/gazebosim/gz-sim/pull/1658)

1. Convert ignitionrobotics to gazebosim in tutorials
    * [Pull request #1757](https://github.com/gazebosim/gz-sim/pull/1757)
    * [Pull request #1758](https://github.com/gazebosim/gz-sim/pull/1758)
    * [Pull request #1759](https://github.com/gazebosim/gz-sim/pull/1759)
    * [Pull request #1760](https://github.com/gazebosim/gz-sim/pull/1760)

1. Added collection name to About Dialog
    * [Pull request #1756](https://github.com/gazebosim/gz-sim/pull/1756)

1. Remove compiler warnings
    * [Pull request #1753](https://github.com/gazebosim/gz-sim/pull/1753)

1. Update examples to use gazebosim.org
    * [Pull request #1749](https://github.com/gazebosim/gz-sim/pull/1749)

1. Remove actors from screen when they are supposed to
    * [Pull request #1699](https://github.com/gazebosim/gz-sim/pull/1699)

1. Readd namespaces for Q_ARGS
    * [Pull request #1670](https://github.com/gazebosim/gz-sim/pull/1670)

### Gazebo Sim 3.X.X (20XX-XX-XX)

### Gazebo Sim 3.13.0 (2022-06-01)

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

### Gazebo Sim 3.12.0 (2021-11-11)

1. Prevent creation of spurious `<plugin>` elements when saving worlds
    * [Pull request #1192](https://github.com/gazebosim/gz-sim/pull/1192)

1. Added support for tracked vehicles
    * [Pull request #869](https://github.com/gazebosim/gz-sim/pull/869)

1. Add components to dynamically set joint limits
    * [Pull request #847](https://github.com/gazebosim/gz-sim/pull/847)

1. Fix updating a component's data via SerializedState msg
    * [Pull request #1149](https://github.com/gazebosim/gz-sim/pull/1149)

1. Sensor systems work if loaded after sensors
    * [Pull request #1104](https://github.com/gazebosim/gz-sim/pull/1104)

1. Fix generation of systems library symlinks in build directory
    * [Pull request #1160](https://github.com/gazebosim/gz-sim/pull/1160)

1. Backport sim::Util::validTopic() from ign-gazebo4.
    * [Pull request #1153](https://github.com/gazebosim/gz-sim/pull/1153)

1. Support setting the background color for sensors
    * [Pull request #1147](https://github.com/gazebosim/gz-sim/pull/1147)

1. Use uint64_t for ComponentInspector Entity IDs
    * [Pull request #1144](https://github.com/gazebosim/gz-sim/pull/1144)

1. Fix integers and floats on component inspector
    * [Pull request #1143](https://github.com/gazebosim/gz-sim/pull/1143)

### Gazebo Sim 3.11.0 (2021-10-21)

1. Updates to camera video record from subt.
    * [Pull request #1117](https://github.com/gazebosim/gz-sim/pull/1117)
1. Fix performance level test flakiness.
    * [Pull request #1129](https://github.com/gazebosim/gz-sim/pull/1129)

### Gazebo Sim 3.10.0 (2021-10-15)

1. Performance: use std::unordered_map where possible in SceneManager
    * [Pull request #1083](https://github.com/gazebosim/gz-sim/pull/1083)

1. Enable new CMake policy to fix protobuf compilation
    * [Pull request #1059](https://github.com/gazebosim/gz-sim/pull/1059)

1. Fix setting cast_shadows for visuals without material
    * [Pull request #1015](https://github.com/gazebosim/gz-sim/pull/1015)

1. Remove duplicate XML tag in pendulum_links example world
    * [Pull request #1002](https://github.com/gazebosim/gz-sim/pull/1002)

1. Enable sensor metrics on example worlds
    * [Pull request #982](https://github.com/gazebosim/gz-sim/pull/982)

1. Improved doxygen
    * [Pull request #996](https://github.com/gazebosim/gz-sim/pull/996)

1. JointPositionController: Improve misleading error message
    * [Pull request #1098](https://github.com/gazebosim/gz-sim/pull/1098)

1. Adjust pose decimals based on element width
    * [Pull request #1089](https://github.com/gazebosim/gz-sim/pull/1089)

1. Fixed IMU system plugin
    * [Pull request #1043](https://github.com/gazebosim/gz-sim/pull/1043)

1. Use QTimer to update plugins in the Qt thread
    * [Pull request #1095](https://github.com/gazebosim/gz-sim/pull/1095)

### Gazebo Sim 3.9.0 (2021-08-16)

1. Entity tree: prevent creation of repeated entity items
    * [Pull request #974](https://github.com/gazebosim/gz-sim/pull/974)

1. Don't use $HOME on most tests (InternalFixture)
    * [Pull request #971](https://github.com/gazebosim/gz-sim/pull/971)

1. Be more specific when looking for physics plugins
    * [Pull request #965](https://github.com/gazebosim/gz-sim/pull/965)

1. Drag and drop meshes into scene
    * [Pull request #939](https://github.com/gazebosim/gz-sim/pull/939)

1. Set protobuf_MODULE_COMPATIBLE before any find_package call
    * [Pull request #957](https://github.com/gazebosim/gz-sim/pull/957)

1. [DiffDrive] add enable/disable
    * [Pull request #772](https://github.com/gazebosim/gz-sim/pull/772)

1. Fix component inspector shutdown crash
    * [Pull request #724](https://github.com/gazebosim/gz-sim/pull/724)

1. Add UserCommands Plugin.
    * [Pull request #719](https://github.com/gazebosim/gz-sim/pull/719)

1. Setting the intiial velocity for a model or joint
    * [Pull request #693](https://github.com/gazebosim/gz-sim/pull/693)

1. Examples and tutorial on using rendering API from plugins
    * [Pull request #596](https://github.com/gazebosim/gz-sim/pull/596)

1.  Add missing GZ_SIM_VISIBLE macros
    * [Pull request #563](https://github.com/gazebosim/gz-sim/pull/563)

1. Fix visibility macro names when used by a different component (Windows)
    * [Pull request #564](https://github.com/gazebosim/gz-sim/pull/564)

1. No install apt recommends and clear cache
    * [Pull request #423](https://github.com/gazebosim/gz-sim/pull/423)

1. Add 25percent darker view angle icons
    * [Pull request #426](https://github.com/gazebosim/gz-sim/pull/426)

1. Expose a test fixture helper class
    * [Pull request #926](https://github.com/gazebosim/gz-sim/pull/926)

1. Fix logic to disable server default plugins loading
    * [Pull request #953](https://github.com/gazebosim/gz-sim/pull/953)

1. removed unneeded plugin update
    * [Pull request #944](https://github.com/gazebosim/gz-sim/pull/944)

1. Functions to enable velocity and acceleration checks on Link
    * [Pull request #935](https://github.com/gazebosim/gz-sim/pull/935)

1. Support adding systems that don't come from a plugin
    * [Pull request #936](https://github.com/gazebosim/gz-sim/pull/936)

1. 3D plot GUI plugin
    * [Pull request #917](https://github.com/gazebosim/gz-sim/pull/917)

1. Add a convenience function for getting possibly non-existing components.
    * [Pull request #629](https://github.com/gazebosim/gz-sim/pull/629)

1. Fix topLevelModel method
    * [Pull request #600](https://github.com/gazebosim/gz-sim/pull/600)

1. World exporter
    * [Pull request #474](https://github.com/gazebosim/gz-sim/pull/474)

1. Fix finding PBR materials
    * [Pull request #575](https://github.com/gazebosim/gz-sim/pull/575)

1. Handle multiple logical cameras
    * [Pull request #539](https://github.com/gazebosim/gz-sim/pull/539)

1. Make some tests more robust
    * [Pull request #314](https://github.com/gazebosim/gz-sim/pull/314)

1. Fix codecheck
    * [Pull request #887](https://github.com/gazebosim/gz-sim/pull/887)

1. Hello world plugin added
    * [Pull request #699](https://github.com/gazebosim/gz-sim/pull/699)

1. Model info CLI `gz model`
    * [Pull request #893](https://github.com/gazebosim/gz-sim/pull/893)

1. Don't create components for entities that don't exist
    * [Pull request #927](https://github.com/gazebosim/gz-sim/pull/927)

1. Adds Mesh Tutorial
    * [Pull request #915](https://github.com/gazebosim/gz-sim/pull/915)

1. Fix updating GUI plugin on load
    * [Pull request #904](https://github.com/gazebosim/gz-sim/pull/904)

1. Fix documentation for the Sensor component
    * [Pull request #898](https://github.com/gazebosim/gz-sim/pull/898)

1. Use UINT64_MAX for kComponentTpyeIDInvalid instead of relying on underflow
    * [Pull request #889](https://github.com/gazebosim/gz-sim/pull/889)

1. Fix mouse view control target position
    * [Pull request #879](https://github.com/gazebosim/gz-sim/pull/879)

1. Set GUI camera pose
    * [Pull request #863](https://github.com/gazebosim/gz-sim/pull/863)

1. Enables confirmation dialog when closing Gazebo.
    * [Pull request #850](https://github.com/gazebosim/gz-sim/pull/850)

1. Depend on gz-rendering 3.5
    * [Pull request #867](https://github.com/gazebosim/gz-sim/pull/867)

1. Using math::SpeedLimiter on the diff_drive controller.
    * [Pull request #833](https://github.com/gazebosim/gz-sim/pull/833)

1. New example: get an ECM snapshot from an external program
    * [Pull request #859](https://github.com/gazebosim/gz-sim/pull/859)

1. Fix WindEffects Plugin bug, not configuring new links
    * [Pull request #844](https://github.com/gazebosim/gz-sim/pull/844)

1. Fix potentially flaky integration component test case
    * [Pull request #848](https://github.com/gazebosim/gz-sim/pull/848)

1. Cleanup and alphabetize plugin headers
    * [Pull request #838](https://github.com/gazebosim/gz-sim/pull/838)

1. Removed duplicated code with rendering::sceneFromFirstRenderEngine
    * [Pull request #819](https://github.com/gazebosim/gz-sim/pull/819)

1. Remove unused headers in video_recoder plugin
    * [Pull request #834](https://github.com/gazebosim/gz-sim/pull/834)

1. Use moveToHelper from gz-rendering
    * [Pull request #825](https://github.com/gazebosim/gz-sim/pull/825)

1. Remove tools/code_check and update codecov
    * [Pull request #814](https://github.com/gazebosim/gz-sim/pull/814)

1. Add service and GUI to configure physics parameters
    * [Pull request #536](https://github.com/gazebosim/gz-sim/pull/536)
    * [Pull request #812](https://github.com/gazebosim/gz-sim/pull/812)

1. Fix documentation for EntityComponentManager::EachNew
    * [Pull request #795](https://github.com/gazebosim/gz-sim/pull/795)

1. Fix macOS build: components::Name in benchmark
    * [Pull request #784](https://github.com/gazebosim/gz-sim/pull/784)

1. Don't store duplicate ComponentTypeId in ECM
    * [Pull request #751](https://github.com/gazebosim/gz-sim/pull/751)

1. [TPE] Support setting individual link velocity
    * [Pull request #427](https://github.com/gazebosim/gz-sim/pull/427)

1. 👩‍🌾 Enable Focal CI
    * [Pull request #646](https://github.com/gazebosim/gz-sim/pull/646)

1. Update benchmark comparison instructions
    * [Pull request #766](https://github.com/gazebosim/gz-sim/pull/766)

1. Use Protobuf_IMPORT_DIRS instead of PROTOBUF_IMPORT_DIRS for compatibility with Protobuf CMake config
    * [Pull request #715](https://github.com/gazebosim/gz-sim/pull/715)

1. Do not pass -Wno-unused-parameter to MSVC compiler
    * [Pull request #716](https://github.com/gazebosim/gz-sim/pull/716)

1. Scenebroadcaster sensors
    * [Pull request #698](https://github.com/gazebosim/gz-sim/pull/698)

1. Make it so joint state publisher is quieter
    * [Pull request #696](https://github.com/gazebosim/gz-sim/pull/696)

### Gazebo Sim 3.8.0 (2021-03-17)

1. Add joint position controller GUI, also enable tests for GUI plugins
    * [Pull request #534](https://github.com/gazebosim/gz-sim/pull/534)

1. Remove visibility from headers that are not installed
    * [Pull request #665](https://github.com/gazebosim/gz-sim/pull/665)

1. Added screenshot to toolbar
    * [Pull request #588](https://github.com/gazebosim/gz-sim/pull/588)

1. Improve gz tool support on macOS
    * [Pull request #477](https://github.com/gazebosim/gz-sim/pull/477)

1. change nullptr to a int ptr for qt 5.15.2 bug
    * [Pull request #527](https://github.com/gazebosim/gz-sim/pull/527)

1. Kinetic energy monitor plugin
    * [Pull request #492](https://github.com/gazebosim/gz-sim/pull/492)

1. Use a std::promise/std::future to avoid busy waiting the step ack messages in NetworkManagerPrimary
    * [Pull request #470](https://github.com/gazebosim/gz-sim/pull/470)

1. clarified performer example
    * [Pull request #390](https://github.com/gazebosim/gz-sim/pull/390)

1. Add tutorial tweaks
    * [Pull request #380](https://github.com/gazebosim/gz-sim/pull/380)

1. Fix Qt5 warnings for using anchors
    * [Pull request #363](https://github.com/gazebosim/gz-sim/pull/363)

1. Update codeowners
    * [Pull request #305](https://github.com/gazebosim/gz-sim/pull/305)

1. Qt auto scale factor for HiDPI displays
    * [Pull request #291](https://github.com/gazebosim/gz-sim/pull/291)

1. Fix yaw units
    * [Pull request #238](https://github.com/gazebosim/gz-sim/pull/238)

1. Fixed docblock showGrid
    * [Pull request #152](https://github.com/gazebosim/gz-sim/pull/152)

1. Fix entity tree for large worlds
    * [Pull request #673](https://github.com/gazebosim/gz-sim/pull/673)

1. Master branch updates
    * [Pull request #672](https://github.com/gazebosim/gz-sim/pull/672)

1. Backport #561: Use common::setenv
    * [Pull request #666](https://github.com/gazebosim/gz-sim/pull/666)

1. Use a custom data structure to manage entity feature maps
    * [Pull request #586](https://github.com/gazebosim/gz-sim/pull/586)

1. Limit scene broadcast publications when paused
    * [Pull request #497](https://github.com/gazebosim/gz-sim/pull/497)

1. Fix flaky SceneBoradcaster test
    * [Pull request #641](https://github.com/gazebosim/gz-sim/pull/641)

1. Add TF/Pose_V publisher in DiffDrive
    * [Pull request #548](https://github.com/gazebosim/gz-sim/pull/548)

1. 👩‍🌾 Relax performance test
    * [Pull request #640](https://github.com/gazebosim/gz-sim/pull/640)

1. 👩‍🌾 Improve velocity control test
    * [Pull request #642](https://github.com/gazebosim/gz-sim/pull/642)

1. Add `laser_retro` support
    * [Pull request #603](https://github.com/gazebosim/gz-sim/pull/603)

1. Fix pose of plane visual with non-default normal vector
    * [Pull request #574](https://github.com/gazebosim/gz-sim/pull/574)

1. Add About dialog
    * [Pull request #609](https://github.com/gazebosim/gz-sim/pull/609)

1. Make topics configurable for joint controllers
    * [Pull request #584](https://github.com/gazebosim/gz-sim/pull/584)

1. Also use Gazebo GUI render event
    * [Pull request #598](https://github.com/gazebosim/gz-sim/pull/598)

1. Tutorial on migrating SDF files from Gazebo classic
    * [Pull request #400](https://github.com/gazebosim/gz-sim/pull/400)

1. Visualize collisions
    * [Pull request #531](https://github.com/gazebosim/gz-sim/pull/531)

1. Backport state update changes from pull request #486
    * [Pull request #583](https://github.com/gazebosim/gz-sim/pull/583)

1. Publish all periodic change components in Scene Broadcaster
    * [Pull request #544](https://github.com/gazebosim/gz-sim/pull/544)

1. added size to `ground_plane` in examples
    * [Pull request #573](https://github.com/gazebosim/gz-sim/pull/573)

1. Parallelize State call in ECM
    * [Pull request #451](https://github.com/gazebosim/gz-sim/pull/451)

1. Non-blocking paths request
    * [Pull request #555](https://github.com/gazebosim/gz-sim/pull/555)

### Gazebo Sim 3.7.0 (2021-01-13)

1. Fix examples in migration plugins tutorial.
    * [Pull Request 543](https://github.com/gazebosim/gz-sim/pull/543)

1. Added missing namespace in `detail/EntityComponentManager.hh`.
    * [Pull Request 541](https://github.com/gazebosim/gz-sim/pull/541)

1. Automatically load a subset of world plugins.
    * [Pull Request 281](https://github.com/gazebosim/gz-sim/pull/281)

1. Update gtest to 1.10.0 for Windows compilation.
    * [Pull Request 506](https://github.com/gazebosim/gz-sim/pull/506)

1. Updates to ardupilot migration tutorial.
    * [Pull Request 525](https://github.com/gazebosim/gz-sim/pull/525)

1. Don't make docs on macOS.
    * [Pull Request 528](https://github.com/gazebosim/gz-sim/pull/528)

### Gazebo Sim 3.6.0 (2020-12-30)

1. Fix pose msg conversion when msg is missing orientation
    * [Pull Request 450](https://github.com/gazebosim/gz-sim/pull/450)

1. Address code checker warnings
    * [Pull Request 443](https://github.com/gazebosim/gz-sim/pull/443)
    * [Pull Request 491](https://github.com/gazebosim/gz-sim/pull/491)
    * [Pull Request 499](https://github.com/gazebosim/gz-sim/pull/499)
    * [Pull Request 502](https://github.com/gazebosim/gz-sim/pull/502)

1. Test fixes
    * [Pull Request 455](https://github.com/gazebosim/gz-sim/pull/455)
    * [Pull Request 463](https://github.com/gazebosim/gz-sim/pull/463)
    * [Pull Request 452](https://github.com/gazebosim/gz-sim/pull/452)
    * [Pull Request 480](https://github.com/gazebosim/gz-sim/pull/480)

1. Documentation updates
    * [Pull Request 472](https://github.com/gazebosim/gz-sim/pull/472)

1. Fix segfault in the Breadcrumb system when associated model is unloaded
    * [Pull Request 454](https://github.com/gazebosim/gz-sim/pull/454)

1. Added user commands to example thermal camera world
    * [Pull Request 442](https://github.com/gazebosim/gz-sim/pull/442)

1. Helper function to set component data
    * [Pull Request 436](https://github.com/gazebosim/gz-sim/pull/436)

1. Remove unneeded if statement in EntityComponentManager
    * [Pull Request 432](https://github.com/gazebosim/gz-sim/pull/432)

1. Clarify how time is represented in each phase of a System step
    * [Pull Request 467](https://github.com/gazebosim/gz-sim/pull/467)

1. Switch to async state service request
    * [Pull Request 461](https://github.com/gazebosim/gz-sim/pull/461)

1. Update key event handling
    * [Pull Request 466](https://github.com/gazebosim/gz-sim/pull/466)

1. Tape Measure Plugin
    * [Pull Request 456](https://github.com/gazebosim/gz-sim/pull/456)

1. Move deselect and preview termination to render thread
    * [Pull Request 493](https://github.com/gazebosim/gz-sim/pull/493)

1. Logical audio sensor plugin
    * [Pull Request 401](https://github.com/gazebosim/gz-sim/pull/401)

1. add frame_id and child_frame_id attribute support for DiffDrive
    * [Pull Request 361](https://github.com/gazebosim/gz-sim/pull/361)

1. Add ability to record video based on sim time
    * [Pull Request 414](https://github.com/gazebosim/gz-sim/pull/414)

1. Add lockstep mode to video recording
    * [Pull Request 419](https://github.com/gazebosim/gz-sim/pull/419)

1. Disable right click menu when using measuring tool
    * [Pull Request 458](https://github.com/gazebosim/gz-sim/pull/458)

### Gazebo Sim 3.5.0 (2020-11-03)

1. Updated source build instructions
    * [Pull Request 403](https://github.com/gazebosim/gz-sim/pull/403)

1. More world APIs, helper function ComponentData
    * [Pull Request 378](https://github.com/gazebosim/gz-sim/pull/378)

1. Improve fork experience
    * [Pull Request 411](https://github.com/gazebosim/gz-sim/pull/411)

1. Fix a crash in the grid config plugin, set grid material
    * [Pull Request 412](https://github.com/gazebosim/gz-sim/pull/412)

1. Document deprecation of log playback `<path>` SDF param
    * [Pull Request 424](https://github.com/gazebosim/gz-sim/pull/424)
    * [Pull Request 425](https://github.com/gazebosim/gz-sim/pull/425)

1. Enable mouse highlighting selection on resource spawner
    * [Pull Request 402](https://github.com/gazebosim/gz-sim/pull/402)

1. Add support for custom render engines
    * [Pull Request 373](https://github.com/gazebosim/gz-sim/pull/373)

1. Component Vector -> Map ECM Optimization
    * [Pull Request 416](https://github.com/gazebosim/gz-sim/pull/416)

### Gazebo Sim 3.4.0 (2020-10-14)

1. Fix gui sendEvent memory leaks
    * [Pull Request 365](https://github.com/gazebosim/gz-sim/pull/365)

1. Support nested models
    * [Pull Request 258](https://github.com/gazebosim/gz-sim/pull/258)

1. Generalize actor count and pose in actor population erb SDF
    * [Pull Request 336](https://github.com/gazebosim/gz-sim/pull/336)

1. Add more link APIs, with tutorial
    * [Pull Request 375](https://github.com/gazebosim/gz-sim/pull/375)

1. Add screenshots to GUI config tutorial
    * [Pull Request 406](https://github.com/gazebosim/gz-sim/pull/406)

1. Fix adding performers to entity tree
    * [Pull Request 374](https://github.com/gazebosim/gz-sim/pull/374)

1. Remove sidebar and put world control in bottom left for joint controller examples
    * [Pull Request 384](https://github.com/gazebosim/gz-sim/pull/384)

1. Allow executing a blocking single Server run in both paused and unpaused states
    * [Pull Request 297](https://github.com/gazebosim/gz-sim/pull/297)

1. Add camera video recorder system
    * [Pull Request 316](https://github.com/gazebosim/gz-sim/pull/316)

1. Decrease time step for quadcopter world
    * [Pull Request 372](https://github.com/gazebosim/gz-sim/pull/372)

1. Add support for moving the GUI camera to a pose
    * [Pull Request 352](https://github.com/gazebosim/gz-sim/pull/352)

1. Remove `lib`+`.so` from plugin's name
    * [Pull Request 279](https://github.com/gazebosim/gz-sim/pull/279)
    * [Pull Request 335](https://github.com/gazebosim/gz-sim/pull/335)

1. EntityComponentManager::EachRemoved documentation fix.
    * [Pull Request 348](https://github.com/gazebosim/gz-sim/pull/348)

1. Add more model APIs.
    * [Pull Request 349](https://github.com/gazebosim/gz-sim/pull/349)

1. Update dimensions of the grid config.
    * [Pull Request 383](https://github.com/gazebosim/gz-sim/pull/383)

1. Fix top-left toolbar layout so magnet shows.
    * [Pull Request 381](https://github.com/gazebosim/gz-sim/pull/381)

1. Add instructions to bitmask world.
    * [Pull Request 377](https://github.com/gazebosim/gz-sim/pull/377)

1. Add search and sort for resource spawner.
    * [Pull Request 359](https://github.com/gazebosim/gz-sim/pull/359)

1. Fix source build instructions for gz-sim3.
    * [Pull Request 395](https://github.com/gazebosim/gz-sim/pull/395)

1. Added playback scrubber GUI
    * [Pull Request 299](https://github.com/gazebosim/gz-sim/pull/299)
    * [Pull Request 362](https://github.com/gazebosim/gz-sim/pull/362)

1. Added wheel slip system plugin.
    * [Pull Request 134](https://github.com/gazebosim/gz-sim/pull/134)
    * [Pull Request 357](https://github.com/gazebosim/gz-sim/pull/357)
    * [Pull Request 362](https://github.com/gazebosim/gz-sim/pull/362)

1. Enhanced log playback performance.
    * [Pull Request 351](https://github.com/gazebosim/gz-sim/pull/351)
    * [Pull Request 362](https://github.com/gazebosim/gz-sim/pull/362)

1. Tests & Warnings: Qt 5.14, breadcrumbs, Gui, gz_TEST
    * [Pull Request 327](https://github.com/gazebosim/gz-sim/pull/327)

1. Added support for specifying topics to record.
    * [Pull Request 315](https://github.com/gazebosim/gz-sim/pull/315)

1. Make sure OpenGL core profile context is used by GzScene3D.
    * [Pull Request 339](https://github.com/gazebosim/gz-sim/pull/339)

1. Support relative paths for PBR materials
    * [Pull Request 328](https://github.com/gazebosim/gz-sim/pull/328)
    * [Pull Request 362](https://github.com/gazebosim/gz-sim/pull/362)

1. Add file extension automatically for record plugin.
    * [Pull Request 303](https://github.com/gazebosim/gz-sim/pull/303)
    * [Pull Request 362](https://github.com/gazebosim/gz-sim/pull/362)

1. Support spawning during log playback.
    * [Pull Request 346](https://github.com/gazebosim/gz-sim/pull/346)

1. Add Render Engine Cmd Line option
    * [Pull Request 331](https://github.com/gazebosim/gz-sim/pull/331)

### Gazebo Sim 3.3.0 (2020-08-31)

1. Added marker array service.
    * [pull request 302](https://github.com/gazebosim/gz-sim/pull/302)

1. Introduced a new parameter in the scene3D plugin to launch in fullscreen.
    * [pull request 254](https://github.com/gazebosim/gz-sim/pull/254)

1. Fix issue #285 by adding checks for a marker's parent.
    * [pull request 290](https://github.com/gazebosim/gz-sim/pull/290)

1. Fix non-specified material error.
    * [pull request 292](https://github.com/gazebosim/gz-sim/pull/292)

1. Added simulation world with large number of entities.
    * [pull request 283](https://github.com/gazebosim/gz-sim/pull/283)

1. Fixed parsing of the touch plugin' enabled flag.
    * [pull request 275](https://github.com/gazebosim/gz-sim/pull/275)

1. Added buoyancy system plugin.
    * [pull request 252](https://github.com/gazebosim/gz-sim/pull/252)

1. Implemented shift + drag = rotate in the GUI.
    * [pull request 247](https://github.com/gazebosim/gz-sim/pull/247)

1. Backport collision bitmask changes
    * [pull request 223](https://github.com/gazebosim/gz-sim/pull/223)

1. Added velocity command to TPE.
    * [pull request 169](https://github.com/gazebosim/gz-sim/pull/169)

1. This version includes all features in Gazebo Sim 2.23.0

### Gazebo Sim 3.2.0 (2020-05-20)

1. Merge gz-sim2 to gz-sim3
    * [pull request 149](https://github.com/gazebosim/gz-sim/pull/149)

### Gazebo Sim 3.1.0 (2020-05-19)

1. Port support for computing model bounding box in physics system
    * [pull request 127](https://github.com/gazebosim/gz-sim/pull/127)

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
    * [Github pull request 96](https://github.com/gazebosim/gz-sim/pull/96)

### Gazebo Sim 3.0.0 (2019-12-10)

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

1. Depend on gz-rendering3, gz-gui3, gz-sensors3
    * [BitBucket pull request 411](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/411)

1. Rendering and Animating Actors
    * [BitBucket pull request 414](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/414)


## Gazebo Sim 2.x

### Gazebo Sim 2.25.0 (2020-09-17)

1. Added wheel slip system plugin.
    * [Pull Request 134](https://github.com/gazebosim/gz-sim/pull/134)
    * [Pull Request 357](https://github.com/gazebosim/gz-sim/pull/357)

1. Enhanced log playback performance.
    * [Pull Request 351](https://github.com/gazebosim/gz-sim/pull/351)

1. Tests & Warnings: Qt 5.14, breadcrumbs, Gui, gz_TEST
    * [Pull Request 327](https://github.com/gazebosim/gz-sim/pull/327)

1. Added support for specifying topics to record.
    * [Pull Request 315](https://github.com/gazebosim/gz-sim/pull/315)

1. Make sure OpenGL core profile context is used by GzScene3D.
    * [Pull Request 339](https://github.com/gazebosim/gz-sim/pull/339)

1. Support relative paths for PBR materials
    * [Pull Request 328](https://github.com/gazebosim/gz-sim/pull/328)

1. Add file extension automatically for record plugin.
    * [Pull Request 303](https://github.com/gazebosim/gz-sim/pull/303)

1. Support spawning during log playback.
    * [Pull Request 346](https://github.com/gazebosim/gz-sim/pull/346)

### Gazebo Sim 2.24.0 (2020-09-03)

1. Resource env var, with transport interface.
    * [Pull Request 172](https://github.com/gazebosim/gz-sim/pull/172)

1. Save http URIs (fix tests)
    * [Pull Request 271](https://github.com/gazebosim/gz-sim/pull/271)

1. Insert Local Models.
    * [Pull Request 173](https://github.com/gazebosim/gz-sim/pull/173)

1. Modernize actions CI.
    * [Pull Request 269](https://github.com/gazebosim/gz-sim/pull/269)

1. Sensor topics available through components and GUI.
    * [Pull Request 266](https://github.com/gazebosim/gz-sim/pull/266)

1. Customizable layouts - fully functional.
    * [Pull Request 278](https://github.com/gazebosim/gz-sim/pull/278)

1. Add Fuel World Support.
    * [Pull Request 274](https://github.com/gazebosim/gz-sim/pull/274)

1. Insert Fuel Models.
    * [Pull Request 263](https://github.com/gazebosim/gz-sim/pull/263)

1. Disable rendering tests on macOS that are known to fail.
    * [Pull Request 209](https://github.com/gazebosim/gz-sim/pull/209)

1. Fix tests on Blueprint.
    * [Pull Request 295](https://github.com/gazebosim/gz-sim/pull/295)

1. Publish remaining breadcrumb deployments.
    * [Pull Request 308](https://github.com/gazebosim/gz-sim/pull/308)

### Gazebo Sim 2.23.0 (2020-07-28)

1. Deactivate PerformerDetector if its parent model gets removed.
    * [Pull Request 260](https://github.com/gazebosim/gz-sim/pull/260)

1. Backport support for <uri>s from Fuel #255
    * [Pull Request 255](https://github.com/gazebosim/gz-sim/pull/255)

### Gazebo Sim 2.22.0 (2020-07-22)

1. Allow zero or more key/value pairs to be added to detection header information.
    * [Pull Request 257](https://github.com/gazebosim/gz-sim/pull/257)

### Gazebo Sim 2.21.0 (2020-07-16)

1. Added support for controlling which joints are published by the
   JointStatePublisher.
    * [Pull Request 222](https://github.com/gazebosim/gz-sim/pull/222)

1. Added an additional pose offset for the performer detector plugin.
    * [Pull Request 236](https://github.com/gazebosim/gz-sim/pull/236)

1. Fixed battery issues and updated tutorial.
    * [Pull Request 230](https://github.com/gazebosim/gz-sim/pull/230)

### Gazebo Sim 2.20.1 (2020-06-18)

1. Properly add new models into the scenegraph. With this fix, when a model is spawned it will be added into the graph and resulting calls to the `scene/info` service will return a correct `msgs::Scene`.
    * [Pull Request 212](https://github.com/gazebosim/gz-sim/pull/212)

### Gazebo Sim 2.20.0 (2020-06-09)

1. Updated battery model to stop battery drain when there is no joint
   velocity/force command, and added a recharging trigger.
    * [Pull Request 183](https://github.com/gazebosim/gz-sim/pull/183)

1. Fix segfault in the Breadcrumbs system
    * [Pull Request 180](https://github.com/gazebosim/gz-sim/pull/180)

1. Added an `<odom_topic>` element to the DiffDrive system so that a custom odometry topic can be used.
    * [Pull Request 179](https://github.com/gazebosim/gz-sim/pull/179)

### Gazebo Sim 2.19.0 (2020-06-02)

1. Use updated model names for spawned models when generating SDFormat
    * [Pull Request 166](https://github.com/gazebosim/gz-sim/pull/166)

1. Allow joint force commands (JointForceCmd) to dscharge a battery.
    * [Pull Request 165](https://github.com/gazebosim/gz-sim/pull/165)

1. Allow renaming breadcrumb models if there is a name conflict
    * [Pull Request 155](https://github.com/gazebosim/gz-sim/pull/155)

1. Add TriggeredPublisher system
    * [Pull Request 139](https://github.com/gazebosim/gz-sim/pull/139)

1. Add PerformerDetector, a system for detecting when performers enter a specified region
    * [Pull Request 125](https://github.com/gazebosim/gz-sim/pull/125)

### Gazebo Sim 2.18.0 (2020-05-20)

1. Added a `/world/<world_name>/create_multiple` service that parallels the current `/world/<world_name>/create` service. The `create_multiple` service can handle an `gz::msgs::EntityFactory_V` message that may contain one or more entities to spawn.
    * [Pull Request 146](https://github.com/gazebosim/gz-sim/pull/146)

1. DetachableJoint system: Add option to suppress warning about missing child model
    * [Pull Request 132](https://github.com/gazebosim/gz-sim/pull/132)

### Gazebo Sim 2.17.0 (2020-05-13)

1. Allow battery plugin to work with joint force systems.
    * [Pull Request 120](https://github.com/gazebosim/gz-sim/pull/120)

1. Make breadcrumb static after specified time
    * [Pull Request 90](https://github.com/gazebosim/gz-sim/pull/90)

1. Disable breadcrumbs if the `max_deployments` == 0.
    * [Pull Request 88](https://github.com/gazebosim/gz-sim/pull/88)

1. Add static pose publisher and support pose\_v msg type in pose publisher system
    * [Pull Request 65](https://github.com/gazebosim/gz-sim/pull/65)

1. Refactor Gui.hh so that the Gazebo GUI can be ran from other packages
    * [Pull Request 79](https://github.com/gazebosim/gz-sim/pull/79)

1. Add ability to save worlds to SDFormat
    * [BitBucket pull request 545](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/545)

1. Add window focus upon mouse entering the render window
    * [Github pull request 95](https://github.com/gazebosim/gz-sim/pull/95)

### Gazebo Sim 2.16.0 (2020-03-24)

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

### Gazebo Sim 2.15.0 (2020-02-07)

1. Fix seeking back in time in log playback
    * [BitBucket pull request 523](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/523)

1. Fix the deprecated gz-sim command line
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

### Gazebo Sim 2.14.0 (2020-01-10)

1. Use Actuator component to communicate between MulticopterVelocityControl and MulticopterMotorModel systems
    * [BitBucket pull request 498](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/498)

1.  Backport fix to insert multiple lights with same name
    * [BitBucket pull request 502](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/502)

1.  Get all component types attached to an entity
    * [BitBucket pull request 494](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/494)

1.  Fix tooltips on entity tree
    * [BitBucket pull request 496](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/496)

### Gazebo Sim 2.13.0 (2019-12-17)

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

### Gazebo Sim 2.12.0 (2019-11-25)

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

### Gazebo Sim 2.11.0 (2019-10-23)

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

### Gazebo Sim 2.10.0 (2019-09-08)

1.  Custom odom frequency in sim time
    * [BitBucket pull request 427](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/427)

1.  Add Move To gui plugin
    * [BitBucket pull request 426](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/426)

### Gazebo Sim 2.9.0

1.  Use the JointSetVelocityCommand feature to set joint velocities
    * [BitBucket pull request 424](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/424)

### Gazebo Sim 2.8.0 (2019-08-23)

1. Add video recorder gui plugin
    * [BitBucket pull request 422](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/422)

1. Vertical rays for lidar demo
    * [BitBucket pull request 419](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/419)

1. Print world path when using cli
    * [BitBucket pull request 420](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/420)

### Gazebo Sim 2.7.1

1. Fix order of adding and removing rendering entities, and clean up mesh
   materials in the SceneManager.
    * [BitBucket pull request 415](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/415)
    * [BitBucket pull request 416](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/416)

### Gazebo Sim 2.7.0

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

### Gazebo Sim 2.6.1 (2019-07-26)

1. Clear stepMsg before populating it
    * [BitBucket pull request 398](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/398)

### Gazebo Sim 2.6.0 (2019-07-24)

1.  Improve performance of Pose Publisher
    * [BitBucket pull request 392](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/392)

1. Fix distributed sim
    * [BitBucket pull request 385](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/385)

### Gazebo Sim 2.5.0 (2019-07-19)

1. The LinearBatteryPlugin system publishes battery state
    * [BitBucket pull request 388](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/388)

### Gazebo Sim 2.4.0 (2019-07-17)

1. Bundle scene updates in sensor system
    * [BitBucket pull request 386](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/386)

### Gazebo Sim 2.3.0 (2019-07-13)

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

### Gazebo Sim 2.2.0

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

### Gazebo Sim 2.1.0

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

### Gazebo Sim 2.0.0

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

1. Port Scene3D gui plugin from gz-gui. Renamed to GzScene3D.
    * [BitBucket pull request 315](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/315)

1. Entity tree UI
    * [BitBucket pull request 285](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/285)

1. Add rendering component
    * [BitBucket pull request 306](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/306)

1. Update Camera and DepthCamera components to use sdf::Sensor object instead of an sdf::ElementPtr.
    * [BitBucket pull request 299](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/299)

1. Added system for gz::sensors::AirPressureSensor.
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

1. Gz tool
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

1. Use gz-sensors magnetometer sensor plugin
    * [BitBucket pull request 221](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/221)

1. Use gz-sensors altimeter sensor plugin
    * [BitBucket pull request 215](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/215)

1. Use gz-sensors imu sensor plugin
    * [BitBucket pull request 219](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/219)

1. Depend on gz-sensors rendering component
    * [BitBucket pull request 212](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/212)

## Gazebo Sim 1.x

### Gazebo Sim 1.X.X

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

1. Logging refactor unique path functions to gz-common
    * [BitBucket pull request 270](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/270)

1. Added test for log record and playback.
    * [BitBucket pull request 263](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/263)

1. Add ApplyJointForce system
    * [BitBucket pull request 254](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/254)

1. More gz-msgs <-> SDF conversions: Inertial, Geometry, Material
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

1. Added a docker image that uses the Gazebo meta package
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

1. Conversion from chrono to gz-msgs
    * [BitBucket pull request 223](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/223)

1. Prevent error message when using levels
    * [BitBucket pull request 229](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/229)

### Gazebo Sim 1.1.0 (2019-03-15)

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

### Gazebo Sim 1.0.2 (2019-03-12)

1. Use TARGET_SO_NAME to fix finding dartsim plugin
    * [BitBucket pull request 217](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/217)

### Gazebo Sim 1.0.1 (2019-03-01)

1. Update gazebo version number in sdf files
    * [BitBucket pull request 207](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/207)

### Gazebo Sim 1.0.0 (2019-03-01)

1. Initial release

## Gazebo Sim 0.x

### Gazebo Sim 0.1.0

1. Add support for joints
    * [BitBucket pull request 77](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/77)

1. Use SimpleWrapper for more component types
    * [BitBucket pull request 78](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/78)

1. Create EventManager and delegate System instantiation to SimulationRunner
    * [BitBucket pull request 79](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/79)

1. Integrate gz-gui
    * [BitBucket pull request 11](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/11)

1. Remove some build dependencies.
    * [BitBucket pull request 6](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/6)

1. Added basic Entity class.
    * [BitBucket pull request 3](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/3)

1. Added a basic System class.
    * [BitBucket pull request 4](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-gazebo/pull-requests/4)
