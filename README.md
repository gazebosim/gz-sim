# Ignition Math : Math classes and functions for robot applications

**Maintainer:** nate AT openrobotics DOT org

[![Bitbucket open issues](https://img.shields.io/bitbucket/issues-raw/ignitionrobotics/ign-math.svg)](https://bitbucket.org/ignitionrobotics/ign-math/issues)
[![Bitbucket open pull requests](https://img.shields.io/bitbucket/pr-raw/ignitionrobotics/ign-math.svg)](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/bb/ignitionrobotics/ign-math/branch/default/graph/badge.svg)](https://codecov.io/bb/ignitionrobotics/ign-math)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-default-bionic-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-default-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-default-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-default-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-default-windows7-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-default-windows7-amd64)

Ignition Math, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides general purpose math
classes and functions designed for robotic applications.

# Table of Contents

[Features](#markdown-header-features)

[Install](#markdown-header-install)

* [Binary Install](#markdown-header-binary-install)

* [Source Install](#markdown-header-source-install)

    * [Prerequisites](#markdown-header-prerequisites)

    * [Building from Source](#markdown-header-building-from-source)

[Usage](#markdown-header-usage)

[Documentation](#markdown-header-documentation)

[Testing](#markdown-header-testing)

[Folder Structure](#markdown-header-folder-structure)

[Code of Conduct](#markdown-header-code-of-conduct)

[Contributing](#markdown-header-code-of-contributing)

[Versioning](#markdown-header-versioning)

[License](#markdown-header-license)

# Features

Ignition Math provides a wide range of functionality, including:

* Type-templated pose, matrix, vector, and quaternion classes.
* Shape representations along with operators to compute volume, density, size and other properties.
* Classes for material properties, mass, inertial, temperature, [PID](https://en.wikipedia.org/wiki/PID_controller), kmeans, spherical coordinates, and filtering.
* Optional Eigen component that converts between a few Eigen and Ignition
Math types.

# Install

We recommend following the [Binary Install](#markdown-header-binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#markdown-header-source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

On Ubuntu systems, `apt-get` can be used to install `ignition-math`:

```
sudo apt install libignition-math<#>-dev
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

The optional Eigen component of Ignition Math requires:

  * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). Refer to the [Eigen Documentation](http://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) for installation instructions. On Ubuntu systems, `apt-get` can be used to install Eigen:

    ```
    sudo apt-get install libeigen3-dev
    ```

### Building from source

1. Clone the repository

    ```
    hg clone https://bitbucket.org/ignitionrobotics/ign-math
    ```

2. Configure and build

    ```
    cd ign-math; mkdir build; cd build; cmake ..; make
    ```

3. Optionally, install Ignition Math

    ```
    sudo make install
    ```

# Usage

Please refer to the [examples directory](https://bitbucket.org/ignitionrobotics/ign-math/raw/default/examples/?at=default).

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/math](https://ignitionrobotics.org/libs/math).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    hg clone https://bitbucket.org/ignitionrobotics/ign-math
    ```

3. Configure and build the documentation.

    ```
    cd ign-math; mkdir build; cd build; cmake ../; make doc
    ```

4. View the documentation by running the following command from the build directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](#markdown-header-source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-math
├── examples                 Example programs.
├── include/ignition/math    Header files.
├── src                      Source files and unit tests.
│   └── graph                Source files for the graph classes.
├── eigen3                   Files for Eigen component.
├── test
│    ├── integration         Integration tests.
│    ├── performance         Performance tests.
│    └── regression          Regression tests.
├── tutorials                Tutorials, written in markdown.
├── Changelog.md             Changelog.
└── CMakeLists.txt           CMake build script.
```
# Contributing

Please see
[CONTRIBUTING.md](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/CONTRIBUTING.md?at=default&fileviewer=file-view-default).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/CODE_OF_CONDUCT.md?at=default&fileviewer=file-view-default).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://bitbucket.org/ignitionrobotics/ign-math/src/default/LICENSE) file.
