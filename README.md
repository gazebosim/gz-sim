# Ignition Gazebo : A Robotic Simulator

**Maintainer:** louise AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-sim.svg)](https://github.com/gazebosim/gz-sim/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-sim.svg)](https://github.com/gazebosim/gz-sim/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-sim)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-main-focal-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-main-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-main-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-main-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/ign_gazebo-ci-win/badge/icon)](https://build.osrfoundation.org/job/ign_gazebo-ci-win/)

Ignition Gazebo is an open source robotics simulator. Through Ignition Gazebo, users have access to high fidelity physics, rendering, and sensor models. Additionally, users and developers have multiple points of entry to simulation including a graphical user interface, plugins, and asynchronous message passing and services.

Ignition Gazebo is derived from [Gazebo](http://gazebosim.org) and represents over 16 years of development and experience in robotics and simulation. This library is part of the [Ignition Robotics](https://ignitionrobotics.org) project.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Documentation](#documentation)

[Testing](#testing)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#contributing)

[Versioning](#versioning)

[License](#license)

# Features

* **Dynamics simulation**: Access multiple high-performance physics engines
through
[Ignition Physics](https://github.com/gazebosim/gz-physics).

* **Advanced 3D graphics**: Through
[Ignition Rendering](https://github.com/gazebosim/gz-rendering),
it's possible to use rendering engines such as OGRE v2 for realistic rendering
of environments with high-quality lighting, shadows, and textures.

* **Sensors and noise models**: Generate sensor data, optionally with noise,
from laser range finders, 2D/3D cameras, Kinect style sensors, contact sensors,
force-torque, IMU, GPS, and more, all powered by
[Ignition Sensors](https://github.com/gazebosim/gz-sensors)

* **Plugins**: Develop custom plugins for robot, sensor, and
environment control.

* **Graphical interface**: Create, instrospect and interact with your simulations
through plugin-based graphical interfaces powered by
[Ignition GUI](https://github.com/gazebosim/gz-gui).

* **Simulation models**: Access numerous robots including PR2, Pioneer2 DX,
iRobot Create, and TurtleBot, and construct environments using other physically
accurate models available through
[Ignition Fuel](https://app.ignitionrobotics.org/fuel). You can also build a
new model using [SDF](http://sdformat.org).

* **TCP/IP Transport**: Run simulation on remote servers and interface to Ignition
Gazebo through socket-based message passing using
[Ignition Transport](https://github.com/gazebosim/gz-transport).

* **Command line tools**: Extensive command line tools for increased simulation
introspection and control.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/gazebo/6.1/install.html).

# Usage

Gazebo can be run from the command line, once [installed](#install), using:

```
ign gazebo
```

For help, and command line options use:

```
ign gazebo -h
```

## Known issue of command line tools

In the event that the installation is a mix of Debian and from source, command
line tools from `ign-tools` may not work correctly.

A workaround for a single package is to define the environment variable
`GZ_CONFIG_PATH` to point to the location of the Ignition library installation,
where the YAML file for the package is found, such as
```
export GZ_CONFIG_PATH=/usr/local/share/ignition
```

However, that environment variable only takes a single path, which means if the
installations from source are in different locations, only one can be specified.

Another workaround for working with multiple Ignition libraries on the command
line is using symbolic links to each library's YAML file.
```
mkdir ~/.ignition/tools/configs -p
cd ~/.ignition/tools/configs/
ln -s /usr/local/share/ignition/fuel8.yaml .
ln -s /usr/local/share/ignition/transport12.yaml .
ln -s /usr/local/share/ignition/transportlog12.yaml .
...
export GZ_CONFIG_PATH=$HOME/.ignition/tools/configs
```

This issue is tracked [here](https://github.com/gazebosim/gz-tools/issues/8).

# Documentation

See the [installation tutorial](https://ignitionrobotics.org/api/gazebo/6.1/install.html).

# Testing

See the [installation tutorial](https://ignitionrobotics.org/api/gazebo/6.1/install.html).

See the [Writing Tests section of the contributor guide](https://github.com/gazebosim/gz-sim/blob/main/CONTRIBUTING.md#writing-tests) for help creating or modifying tests.

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-gazebo
├── examples                     Various examples that can be run against binary or source installs of ign-gazebo.
│   ├── plugin                   Example plugins.
│   ├── standalone               Example standalone programs that use ign-gazebo as a library.
│   └── worlds                   Example SDF world files.
├── include/ignition/gazebo      Header files that downstream users are expected to use.
│   └── detail                   Header files that are not intended for downstream use, mainly template implementations.
├── src                          Source files and unit tests.
│   ├── gui                      Graphical interface source code.
│   └── systems                  System source code.
├── test
│   ├── integration              Integration tests.
│   ├── performance              Performance tests.
│   ├── plugins                  Plugins used in tests.
│   ├── regression               Regression tests.
│   └── tutorials                Tutorials, written in markdown.
├── Changelog.md                 Changelog.
├── CMakeLists.txt               CMake build script.
├── Migration.md                 Migration guide.
└── README.md                    This readme.
```

# Contributing

Please see
[CONTRIBUTING.md](https://github.com/gazebosim/gz-sim/blob/main/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sim/blob/main/LICENSE) file.
