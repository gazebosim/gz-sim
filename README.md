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
[![Build Status](https://drone.io/bitbucket.org/ignitionrobotics/ign-math/status.png)](https://drone.io/bitbucket.org/ignitionrobotics/ign-math/latest)

This project also uses [bitbucket pipelines](https://bitbucket.org/ignitionrobotics/ign-math/addon/pipelines/home#!/)
for testing with Linux.

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
