# Ignition Gazebo : A Robotic Simulator

**Maintainer:** louise AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-gazebo.svg)](https://github.com/ignitionrobotics/ign-gazebo/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-gazebo.svg)](https://github.com/ignitionrobotics/ign-gazebo/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-gazebo/branch/ign-gazebo3/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-gazebo)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-ign-gazebo3-bionic-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-ign-gazebo3-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-ign-gazebo3-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-ign-gazebo3-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-ign-gazebo3-windows7-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-ign-gazebo3-windows7-amd64)

Ignition Gazebo is an open source robotics simulator. Through Ignition Gazebo users have access to high fidelity physics, rendering, and sensor models. Additionally, users and developers have multiple points of entry to simulation including a graphical user interface, plugins, and asynchronous message passing and services.

Ignition Gazebo is derived from [Gazebo](http://gazebosim.org) and represents over 16 years of development and experience in robotics and simulation. This library is part of the [Ignition Robotics](https://ignitionrobotics.org) project.

# Table of Contents

[Features](#features)

[Install](#install)

* [Binary Install](#binary-install)

* [Source Install](#source-install)

    * [Prerequisites](#prerequisites)

    * [Building from Source](#building-from-source)

[Usage](#usage)

[Documentation](#documentation)

[Testing](#testing)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

* **Dynamics simulation**: Access multiple high-performance physics engines
through
[Ignition Physics](https://github.com/ignitionrobotics/ign-physics).

* **Advanced 3D graphics**: Through
[Ignition Rendering](https://github.com/ignitionrobotics/ign-rendering),
it's possible to use rendering engines such as OGRE v2 for realistic rendering
of environments with high-quality lighting, shadows, and textures.

* **Sensors and noise models**: Generate sensor data, optionally with noise,
from laser range finders, 2D/3D cameras, Kinect style sensors, contact sensors,
force-torque, IMU, GPS, and more, powered by
[Ignition Sensors](https://github.com/ignitionrobotics/ign-sensors)

* **Plugins**: Develop custom plugins for robot, sensor, and
environment control.

* **Graphical interface**: Create, instrospect and interact with your simulations
through plugin-based graphical interfaces powered by
[Ignition GUI](https://github.com/ignitionrobotics/ign-gui).

* **Simulation models**: Access numerous robots including PR2, Pioneer2 DX,
iRobot Create, and TurtleBot, and construct environments using other physically
accurate models available through
[Ignition Fuel](https://app.ignitionrobotics.org/fuel). You can also build a
new model using [SDF](http://sdformat.org).

* **TCP/IP Transport**: Run simulation on remote servers and interface to Ignition
Gazebo through socket-based message passing using
[Ignition Transport](https://github.com/ignitionrobotics/ign-transport).

* **Command line tools**: Extensive command line tools for increased simulation
introspection and control.

# Install

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

The binary install method will use pre-built packages which are typically
available through a package management utility such as [Apt](https://wiki.debian.org/Apt).
This approach eliminates the need to download and compile source code, and dependencies
are handled for you. The downside of a binary install is that you won't be able to modify
the code. See [Source Install](#source-install) for information on
installing Ignition Gazebo from source.

**Ubuntu Bionic (version 2)**

1. Configure package repositories.

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
    ```

    ```
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

    ```
    sudo apt-get update
    ```

2. Install Ignition Gazebo

    ```
    sudo apt-get install libignition-gazebo2-dev
    ```

## Source Install (version 3)

Install from source if you're interested in changing the source code or need a
feature which hasn't been released yet.

### Prerequisites

Ignition Gazebo has a fairly large set of dependencies. Refer to the following sections
for dependency installation instructions for each supported operating system.

**[Ubuntu Bionic](http://releases.ubuntu.com/18.04/)**

1. Install third-party libraries:

    ```
    sudo apt-get -y install cmake build-essential curl cppcheck g++-8 libbenchmark-dev libgflags-dev doxygen ruby-ronn libtinyxml2-dev libtinyxml-dev software-properties-common libeigen3-dev qtdeclarative5-models-plugin
    ```

2. Install required Ignition libraries:

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
    ```

    ```
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

    ```
    sudo apt-get update
    ```

    ```
    sudo apt-get -y install libignition-cmake2-dev libignition-common3-dev libignition-math6-eigen3-dev libignition-plugin-dev libignition-physics2-dev libignition-rendering3-dev libignition-tools-dev libignition-transport8-dev libignition-gui3-dev libignition-msgs5-dev libsdformat9-dev
    ```

### Building from source

1. Install [prerequisites](#prerequisites)

2. Configure gcc8

    * Ubuntu

        ```
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
        ```

1. Clone the repository.

    ```
    git clone https://github.com/ignitionrobotics/ign-gazebo -b ign-gazebo3
    ```

2. Configure and  build.

    ```
    cd ign-gazebo
    mkdir build
    cd build
    cmake ../
    make
    ```

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
`IGN_CONFIG_PATH` to point to the location of the Ignition library installation,
where the YAML file for the package is found, such as
```
export IGN_CONFIG_PATH=/usr/local/share/ignition
```

However, that environment variable only takes a single path, which means if the
installations from source are in different locations, only one can be specified.

Another workaround for working with multiple Ignition libraries on the command
line is using symbolic links to each library's YAML file.
```
mkdir ~/.ignition/tools/configs -p
cd ~/.ignition/tools/configs/
ln -s /usr/local/share/ignition/fuel4.yaml .
ln -s /usr/local/share/ignition/transport7.yaml .
ln -s /usr/local/share/ignition/transportlog7.yaml .
...
export IGN_CONFIG_PATH=$HOME/.ignition/tools/configs
```

This issue is tracked [here](https://github.com/ignitionrobotics/ign-tools/issues/8).

# Documentation

API documentation and tutorials can be accessed at [https://ignitionrobotics.org/libs/gazebo](https://ignitionrobotics.org/libs/gazebo)

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need [Doxygen](http://www.doxygen.org/). On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-gazebo
    ```

3. Configure and build the documentation.

    ```
    cd ign-gazebo
    mkdir build
    cd build
    cmake ../
    make doc
    ```

4. View the documentation by running the following command from the `build` directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](#source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

See the [Writing Tests section of the contributor guide](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CONTRIBUTING.md#writing-tests) for help creating or modifying tests.

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
[CONTRIBUTING.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-gazebo/blob/master/LICENSE) file.
