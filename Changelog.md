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
