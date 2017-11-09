# Ignition Math


**Math classes and functions for robot applications.**

Ignition Math is a component in the Ignition framework, a set of libraries
designed to rapidly develop robot applications. The library defines math
classes and functions used in other Ignition libraries and programs.

  [http://ignitionrobotics.org](http://ignitionrobotics.org)

## Continuous integration

This project uses [appveyor](https://ci.appveyor.com/project/scpeters/ign-math/history)
for testing on Windows.
It has the following build status:
![appveyor badge](https://ci.appveyor.com/api/projects/status/bitbucket/ignitionrobotics/ign-math?svg=true)

This project also uses [bitbucket pipelines](https://bitbucket.org/ignitionrobotics/ign-math/addon/pipelines/home#!/)
for testing with Linux.

Test coverage reports are available at Codecov:

[![codecov](https://codecov.io/bb/ignitionrobotics/ign-math/branch/master/graph/badge.svg)](https://codecov.io/bb/ignitionrobotics/ign-math)

## Required Dependencies

    brew install ignition-cmake

## Optional Dependencies

    sudo apt-get install doxygen
    brew install doxygen

## Installation

Standard installation can be performed in UNIX systems using the following
steps:

 - mkdir build/
 - cd build/
 - cmake ..
 - sudo make install

## Uninstallation

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall

## Create Documentation & Release

1. Build documentation

```
cd build
make doc
```

1. Upload documentation to ignitionrobotics.org.

```
cd build
sh upload.sh
```

1. If you're creating a new release, then tell ignitionrobotics.org about
   the new version. For example:

```
curl -k -X POST -d '{"libName":"common", "version":"1.0.0", "releaseDate":"2017-10-09T12:10:13+02:00","password":"secret"}' https://api.ignitionrobotics.org/1.0/versions
```

