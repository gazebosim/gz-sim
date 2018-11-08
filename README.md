# Ignition Gazebo

This is a prototype for the next version of [Gazebo](http://gazebosim.org).

[![codecov](https://codecov.io/bb/ignitionrobotics/ign-gazebo/branch/default/graph/badge.svg)](https://codecov.io/bb/ignitionrobotics/ign-gazebo)
[![Bitbucket open issues](https://img.shields.io/bitbucket/issues-raw/ignitionrobotics/ign-gazebo.svg)](https://bitbucket.org/ignitionrobotics/ign-gazebo/issues)
[![Bitbucket open pull requests](https://img.shields.io/bitbucket/pr-raw/ignitionrobotics/ign-gazebo.svg)](https://bitbucket.org/ignitionrobotics/ign-gazebo/pull-requests)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-default-bionic-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-default-bionic-amd64)  
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-default-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-default-homebrew-amd64)  
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_gazebo-ci-default-windows7-amd64)](https://build.osrfoundation.org/job/ignition_gazebo-ci-default-windows7-amd64)

**Table of Contents**

[Features](#markdown-header-features)

[Get Started](#markdown-header-install)

* [Prerequisites](#markdown-header-prerequisites)

* [Installing](#markdown-header-prerequisites)

  * [Debian](#markdown-header-debian)

  * [Building from source](#markdown-header-building-from-source)

* [Usage](#markdown-header-usage)

* [Testing](#markdown-header-testing)

[Documentation](#markdown-header-documentation)

[Folder Structure](#markdown-header-folder-structure)

[Code of Conduct](#markdown-header-code-of-conduct)

[Contributing](#markdown-header-code-of-contributing)

[Versioning](#markdown-header-versioning)

[License](#markdown-header-license)

# Features

TODO

# Get Started

## Prerequisites

## Building from source

1. Clone the repository.

```
hg clone https://bitbucket.org/ignitionrobotics/ign-gazebo
```

2. Configure and  build.

```
cd ign-gazebo
mkdir build
cd build
cmake ../
make
```

## Usage

TODO

## Testing

Follow these steps to run tests and static code analysis in your clone of this respository.

1. Follow the [building from source instructions](#markdown-header-building from source).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

See the [Writing Tests section of the contributor guide](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/CONTRIBUTING.md#markdown-header-writing-tests) for help creating or modifying tests.

# Documentation

The following links contain documentation for the latest release.

* [API Documentation](https://ignitionrobotics.org/libs/gazebo/latest)
* [Tutorials](https://ignitionrobotics.org/libs/gazebo/latest/tutorials)

Documentation for past releases can be accessed at [https://ignitionrobotics.org/libs/gazebo](https://ignitionrobotics.org/libs/gazebo)

You can also generate documentation from a clone of this repository by following these steps.

1. You will need [Doxygen](http://www.doxygen.org/). On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    hg clone https://bitbucket.org/ignitionrobotics/ign-gazebo
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
    
# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
+-- examples                 Example programs.  
+-- include/ignition/gazebo  Header files.  
+-- src                      Source files and unit tests.  
|    +-- components          Component source code.  
|    +-- gui                 Graphical interface source code.  
|    +-- system              System source code.
+-- test
|    +-- integration         Integration tests.
|    +-- performance         Performance tests.
|    +-- plugins             Plugin tests.
|    +-- regression          Regression tests.
|    +-- worlds              SDF world files used in tests.
+-- tutorials                Tutorials, written in markdown. 
+-- Changelog.md             Changelog.
+-- CMakeLists.txt           CMake build script.  
+-- Migration.md             Migration guide.  
+-- README.md                This readme.  
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

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/LICENSE?at=default&fileviewer=file-view-default) file.



