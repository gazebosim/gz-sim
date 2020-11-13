# Ignition Math : Math classes and functions for robot applications

**Maintainer:** nate AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-math.svg)](https://github.com/ignitionrobotics/ign-math/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-math.svg)](https://github.com/ignitionrobotics/ign-math/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-math/branch/master/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-math)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-master-bionic-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-master-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-master-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-master-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_math-ci-master-windows7-amd64)](https://build.osrfoundation.org/job/ignition_math-ci-master-windows7-amd64)

Ignition Math, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides general purpose math
classes and functions designed for robotic applications.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Documentation](#documentation)

[Testing](#testing)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

Ignition Math provides a wide range of functionality, including:

* Type-templated pose, matrix, vector, and quaternion classes.
* Shape representations along with operators to compute volume, density, size and other properties.
* Classes for material properties, mass, inertial, temperature, [PID](https://en.wikipedia.org/wiki/PID_controller), kmeans, spherical coordinates, and filtering.
* Optional Eigen component that converts between a few Eigen and Ignition
Math types.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/math/6.6/install.html).

# Usage

Please refer to the [examples directory](https://github.com/ignitionrobotics/ign-math/raw/master/examples/).

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/math](https://ignitionrobotics.org/libs/math).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-math
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

1. Follow the [source install instruction](https://ignitionrobotics.org/libs/math#source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

## Ruby Tests

### Usage

The C++ classes are available in Ruby code by interface files (`.i`) used by swig to build a C++ extension module.

The interfaces and Ruby test codes are in the `src` folder. To use a C++ class in Ruby you need to:

1. Create an interface file describing the class as in Swig and Ruby reference at [The Ruby-to-C/C++ Mapping](http://www.swig.org/Doc1.3/Ruby.html#Ruby_nn11)

2. Include the interface file in `/src/ing_math.i`

3. Create the Ruby file and import the class as in Swig and Ruby reference at [C++ Classes](http://www.swig.org/Doc1.3/Ruby.html#Ruby_nn18)

### Tests

`make test` already runs all tests, including the ones made in Ruby, but you can run a Ruby test individually using

```
ctest -R Ruby_TEST.rb
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
[CONTRIBUTING.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-math/blob/master/LICENSE) file.
