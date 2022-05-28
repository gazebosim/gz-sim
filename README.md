# Gazebo Math : Math classes and functions for robot applications

**Maintainer:** nate AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/gz-math.svg)](https://github.com/gazebosim/gz-math/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/gz-math.svg)](https://github.com/gazebosim/gz-math/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/gz-math/branch/ign-math7/graph/badge.svg)](https://codecov.io/gh/gazebosim/gz-math)
Ubuntu Focal  | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-ign-math7-focal-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-ign-math7-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-ign-math7-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-ign-math7-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ign_math-ign-7-win)](https://build.osrfoundation.org/job/ign_math-ign-7-win)

Gazebo Math, a component of [Ignition
Robotics](https://gazebosim.org), provides general purpose math
classes and functions designed for robotic applications.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

Gazebo Math provides a wide range of functionality, including:

* Type-templated pose, matrix, vector, and quaternion classes.
* Shape representations along with operators to compute volume, density, size and other properties.
* Classes for material properties, mass, inertial, temperature, [PID](https://en.wikipedia.org/wiki/PID_controller), kmeans, spherical coordinates, and filtering.
* Optional Eigen component that converts between a few Eigen and Ignition
Math types.

# Install

See the [installation tutorial](https://gazebosim.org/api/math/6.8/install.html).

# Usage

Please refer to the [examples directory](https://github.com/gazebosim/gz-math/raw/ign-math7/examples/).

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-math
├── examples                 Example programs.
├── include/ignition/math    Header files.
├── src                      Source files and unit tests.
│   └── graph                Source files for the graph classes.
│   └── python               SWIG Python interfaces.
│   └── ruby                 SWIG Ruby interfaces.
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
[CONTRIBUTING.md](https://github.com/gazebosim/gz-sim/blob/main/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/gazebosim/gz-sim/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Gazebo project](https://gazebosim.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Gazebo website](https://gazebosim.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-math/blob/main/LICENSE) file.
