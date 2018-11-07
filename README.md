# Ignition Gazebo

This is a prototype for the next version of [Gazebo](http://gazebosim.org).

[![codecov](https://codecov.io/bb/ignitionrobotics/ign-gazebo/branch/default/graph/badge.svg)](https://codecov.io/bb/ignitionrobotics/ign-gazebo)

**Table of Contents**

[Features](#markdown-header-features)

[Get Started](#markdown-header-install)

* [Prerequisites](#markdown-header-prerequisites)

* [Installing](#markdown-header-prerequisites)

  * [Debian](#markdown-header-debian)

  * [Building from source](#markdown-header-building-from-source)

* [Usage](#markdown-header-usage)

* [Testing](#markdown-header-testing)

[Direc
[Documentation](#markdown-header-documentation)

[Code of Conduct](#markdown-header-code-of-conduct)

[Contributing](#markdown-header-code-of-contributing)

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

The following describes how to run the tests and static code checker.

1. Run tests.

```
make test
```

2. Static code checker.

```
make codecheck
```

# Folder Structure

  .
  ├── doc                     # Contains files for building documentation.
  ├── examples                # Example programs.
  ├── include                 # Header files.
  ├── src                     # Source files and unit tests.
  ├── test                    # Integration, performance, and regression tests.
  ├── tools                   # Integration, performance, and regression tests.
  ├── appveyor.yml            # [Appveyor](https://www.appveyor.com/) configuration.
  ├── bitbucket-pipelines.yml # [Bitbucket
  pipelines](https://bitbucket.org/ignitionrobotics/ign-gazebo/addon/pipelines/home#!/) configuration.
  ├── CMakeLists.txt          # CMake build script.
  ├── configure.bat           # Windows build script.
  ├── LICENSE                 # License information.
  └── README.md               # This readme.

# Documentation

TODO

* API
* Contributor Guide
* Tutorials

Documentation is generated at compile time. To view the documentation:

```
firefox build/doxygen/html/index.html
```

# Contributing

Please see
[CONTRIBUTING.md](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/CONTRIBUTING.md?at=default&fileviewer=file-view-default).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/406665896aa40bb42f14cf61d48b3d94f2fc5dd8/CODE_OF_CONDUCT.md?at=default&fileviewer=file-view-default).

# Versioning

TODO: Add information about ignition versioning.

We use [Semantic Versioning](https://semver.org/).

# License

[Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0)
