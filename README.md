# Ignition Gazebo

This is a prototype for the next version of [Gazebo](http://gazebosim.org).

# Building from source

1. Get the source code

```
hg clone https://bitbucket.org/ignitionrobotics/ign-gazebo
```

2. Configure and Build

```
cd ign-gazebo
mkdir build
cd build
cmake ../
make
```

# Tests

Testing is done using Google Test. Tests are built by default. To run all tests:

```
make test
```

# Style and Static Code Check

In the root of the source tree run:

```
sh tools/code_check.sh
```

# Documentation

Documentation is generated at compile time. To view the documentation:

```
firefox build/doxygen/html/index.html
```
