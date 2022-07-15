\page test_fixture Test fixture

Gazebo can be used to run scripted simulations so that it's convenient
to run automated tests for robot applications. This tutorial will go over the
process of writing automated tests using Gazebo so that they can be run
using Continuous Integration (CI).

## Example setup

Gazebo can be used for testing using any testing library. We provide
an example of how to setup some test cases using
[Google Test](https://github.com/google/googletest) in
[gz-sim/examples/standalone/gtest_setup](https://github.com/gazebosim/gz-sim/tree/main/examples/standalone/gtest_setup).

The instructions on that example's `README` can be followed to compile and run
those tests. Also be sure to go through the source code for comments with
helpful pointers on how to setup tests.

## Test Fixture

The example above uses the `gz::sim::TestFixture` class, which provides
a convenient API to start a simulation and step through it while checking that
entities in simulation are acting as expected. See that class' documentation
for more information.
