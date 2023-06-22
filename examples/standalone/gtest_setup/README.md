# GTest setup

Example of how to setup simulation-based tests using GTest.

The example contains 2 tests:

* `gravity_TEST` is a minimal example to explain the basics
* `command_TEST` has a slightly more realistic test

See the comments on the source code for more explanations.

## Build

From the root of the repository:

    cd examples/standalone/gtest_setup
    mkdir build
    cd build
    cmake ..
    make

## Run tests

    cd examples/standalone/gtest_setup/build
    ./gravity_TEST
    ./command_TEST
