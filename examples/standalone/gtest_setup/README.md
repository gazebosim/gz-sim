# GTest setup

Example of how to setup simulation-based tests using GTest.

The example contains 2 tests:

* `gravity_TEST` is a minimal example to explain the basics
* `command_TEST` has a slightly more realistic test

See the comments on the source code for more explanations.

## Build Instructions

From this directory, run the following to compile:

```bash
cmake -B build
cmake --build build
```

## Run tests

From this directory, run the following to run tests:

### CMake 3.20 or newer

```bash
ctest --test-dir build
```

By default, ctest hides stdout from tests.
To enable test output, add `-V`.
CTest also hides colors, which can be re-enabled.
```bash
GTEST_COLOR=1 ctest -V --test-dir build 
```


### CMake 3.19 or earlier

```bash
cd build
ctest
```

