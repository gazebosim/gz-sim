# Ignition Seed

This repository contains boilerplate code to start new C++ ignition projects.

## Quickstart

1. Choose a name for your project. The name should be one word.
   On the following instructions, substitute:

    * `<project-name>` with your project's lower case name, i.e. `math`
    * `<PROJECT-NAME>` with your project's lower case name, i.e. `MATH`

1. Clone this repository:

        hg clone http://bitbucket.org/ignitionrobotics/ign-seed ign-<name>

        Example with <projhect-name> == "test":


1. Move to the project folder:

        cd ign-<project-name>

1. Substitute `<project-name>` in the code with <name>:

        find -regex '.*\.\(cc\|txt\|hh\|md\)$' -exec sed -i 's/<project-name>/<name>/g' {} \;

1. Substitute `<PROJECT-NAME>` in the code with <NAME>:

        find -regex '.*\.\(cc\|txt\|hh\|md\)$' -exec sed -i 's/<PROJECT-NAME>/<NAME>/g' {} \;


1. Rename files and directories:

        mv include/ignition/project-name include/ignition/<name>

### Example

This example will create an ign-test instance of ign-seed.

```
hg clone http://bitbucket.org/ignitionrobotics/ign-seed ign-test
```

```
cd ign-test
```

```
find -regex '.*\.\(cc\|txt\|hh\|md\)$' -exec sed -i 's/<project-name>/test/g' {} \;
```

```
find -regex '.*\.\(cc\|txt\|hh\|md\)$' -exec sed -i 's/<PROJECT-NAME>/TEST/g' {} \;
```

```
mv include/ignition/project-name include/ignition/test
```

Now delete these instructions down to the line below, and follow the
   remaining instructions.

-----------------

# Ignition <project-name>

** Igntion <project-name> classes and functions for robot applications.**

Ignition <project-name> is a component in the ignition framework, a set
of libraries designed to rapidly develop robot applications.

  [http://ignitionrobotics.org](http://ignitionrobotics.org)

## Installation

Standard installation can be performed in UNIX systems using the following
steps:

    mkdir build/
    cd build/
    cmake ..
    sudo make install

## Uninstallation

To uninstall the software installed with the previous steps:

    cd build/
    sudo make uninstall

## Testing

Tests are built by default. After building, to run all tests:

    make test

To run one specific test:

    ./src/UNIT_Example_TEST

### Disable tests building

To build without tests, on the cmake step, do this instead:

    cmake .. -DENABLE_TESTS_COMPILATION=False

### Test coverage

To build test coverage, first install lcov:

    sudo apt-get install lcov

Configure coverage:

    cmake -DCMAKE_BUILD_TYPE=coverage ../; make

Run tests:

    make test # or individual test

Make coverage:

    make coverage # FIXME: currently failing

See coverage report:

    firefox coverage/index.html

## Code checker

To run the code checker:

    sh tools/code_check.sh

## Documentation generation

    # TODO


