# Gazebo Sim : A Robotic Simulator

**Maintainer:** arjoc AT intrinsic DOT ai

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-sim.svg)](https://github.com/gazebosim/gz-sim/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-sim.svg)](https://github.com/gazebosim/gz-sim/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-sim/tree/gz-sim9/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-sim/tree/gz-sim9)
Ubuntu Noble  | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sim-ci-gz-sim9-noble-amd64)](https://build.osrfoundation.org/job/gz_sim-ci-gz-sim9-noble-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=gz_sim-ci-gz-sim9-homebrew-amd64)](https://build.osrfoundation.org/job/gz_sim-ci-gz-sim9-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/gz_sim-9-clowin/badge/icon)](https://build.osrfoundation.org/job/gz_sim-9-clowin/)

Gazebo Sim is an open source robotics simulator. Through Gazebo Sim, users have access to high fidelity physics, rendering, and sensor models. Additionally, users and developers have multiple points of entry to simulation including a graphical user interface, plugins, and asynchronous message passing and services.

Gazebo Sim is derived from [Gazebo Classic](http://classic.gazebosim.org) and represents over 16 years of development and experience in robotics and simulation. This library is part of the [Gazebo](https://gazebosim.org) project.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Documentation](#documentation)

[Testing](#testing)

[Folder Structure](#folder-structure)

[Contributing](#contributing)

[Code of Conduct](#code-of-conduct)

[Versioning](#versioning)

[License](#license)

# Features

* **Dynamics simulation**: Access multiple high-performance physics engines
through
[Gazebo Physics](https://github.com/gazebosim/gz-physics).

* **Advanced 3D graphics**: Through
[Gazebo Rendering](https://github.com/gazebosim/gz-rendering),
it's possible to use rendering engines such as OGRE v2 for realistic rendering
of environments with high-quality lighting, shadows, and textures.

* **Sensors and noise models**: Generate sensor data, optionally with noise,
from laser range finders, 2D/3D cameras, Kinect style sensors, contact sensors,
force-torque, IMU, GPS, and more, all powered by
[Gazebo Sensors](https://github.com/gazebosim/gz-sensors)

* **Plugins**: Develop custom plugins for robot, sensor, and
environment control.

* **Graphical interface**: Create, introspect and interact with your simulations
through plugin-based graphical interfaces powered by
[Gazebo GUI](https://github.com/gazebosim/gz-gui).

* **Simulation models**: Access numerous robots including PR2, Pioneer2 DX,
iRobot Create, and TurtleBot, and construct environments using other physically
accurate models available through
[Gazebo Fuel](https://app.gazebosim.org/fuel). You can also build a
new model using [SDF](http://sdformat.org).

* **TCP/IP Transport**: Run simulation on remote servers and interface to
Gazebo Sim through socket-based message passing using
[Gazebo Transport](https://github.com/gazebosim/gz-transport).

* **Command line tools**: Extensive command line tools for increased simulation
introspection and control.

# Install

For installing Gazebo, see the [getting started guide](https://gazebosim.org/docs/latest/getstarted/). If you want to use `libgz-sim` as a library, see the [installation tutorial](https://gazebosim.org/api/sim/9/install.html)

# Usage

Gazebo Sim can be run from the command line, once [installed](#install), using:

```
gz sim
```

For help, and command line options use:

```
gz sim -h
```

## Known issue of command line tools

In the event that the installation is a mix of Debian and from source, command
line tools from `gz-tools` may not work correctly.

A workaround is to define the environment variable
`GZ_CONFIG_PATH` to point to the different locations of the Gazebo libraries installations,
where the YAML files for the packages are found, such as
```
export GZ_CONFIG_PATH=/usr/local/share/gz:$HOME/ws/install/share/gz
```

where `$HOME/ws` is an example colcon workspace used to build Gazebo.

On Windows, `gz sim` (i.e. running both server and GUI in one command) doesn't yet work.
To run Gazebo Sim on Windows, you need to run the server in one terminal (`gz sim -s <other args>`)
and the GUI in another terminal (`gz sim -g <other args>`). Remember this when reading through
all Gazebo Sim tutorials. Also remember that Conda and `install\setup.bat` need to be sourced
in both terminals (as well as any changes to `GZ_PARTITION` and other environment variables).

# Documentation

See the [installation tutorial](https://gazebosim.org/api/sim/9/install.html).

# Testing

See the [installation tutorial](https://gazebosim.org/api/sim/9/install.html).

See the [Writing Tests section of the contributor guide](https://gazebosim.org/docs/all/contributing/#writing-tests) for help creating or modifying tests.

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
gz-sim
├── examples                     Various examples that can be run against binary or source installs of gz-sim.
│   ├── plugin                   Example plugins.
│   ├── standalone               Example standalone programs that use gz-sim as a library.
│   └── worlds                   Example SDF world files.
├── include/gz/sim               Header files that downstream users are expected to use.
│   └── detail                   Header files that are not intended for downstream use, mainly template implementations.
├── python                       Python wrappers
├── src                          Source files and unit tests.
│   ├── gui                      Graphical interface source code.
│   └── systems                  System source code.
├── test
│   ├── integration              Integration tests.
│   ├── performance              Performance tests.
│   ├── plugins                  Plugins used in tests.
│   ├── regression               Regression tests.
├── tutorials                    Tutorials, written in markdown.
├── Changelog.md                 Changelog.
├── CMakeLists.txt               CMake build script.
├── Migration.md                 Migration guide.
└── README.md                    This readme.
```

# Contributing

Please see the
[contribution guide](https://gazebosim.org/docs/all/contributing/).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sim/blob/main/LICENSE) file.
