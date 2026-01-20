# Joint control components

This example shows how to use three different
[components](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html)
to create joint motion:

* `JointForceCmd`: this component applies an effort to each degree of freedom
  of the specified joint (force for translational joint axes and torque for
  rotational joint axes).
* `JointVelocityCmd`: this component specifies a desired velocity that a
  controller attempts to match subject to joint effort limits.
* `JointVelocityReset`: this component rewrites the joint velocity state,
  in a similar way to setting a new initial condition.

These components are demonstrated with the following plugins:

* `ResetJointVelocityNearPosition`: this plugin sets the `JointVelocityReset`
  component to a fixed value when the measured joint position is near a trigger
  position. This can create a bouncing behavior of a pendulum if the trigger
  position is the bottom stable equilibrium.
* `PulseJointVelocityCommand`: this plugin applies pulses of joint velocity
  commands as a square wave with a specified time period between pulses.

## Build

~~~
cd examples/plugin/joint_control_components
mkdir build
cd build
cmake ..
make
~~~

This will build the `ResetJointVelocityNearPosition` and
`PulseJointVelocityCommand` libraries under `build`.

## Run

Add the library to the path:

~~~
cd examples/plugin/joint_control_components
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run a world that loads examples of these plugins.

    gz sim -r -v 4 joint_control_components.sdf

There should be 3 pendulums loaded.

* On the far left, the pendulum has only the `ResetJointVelocityNearPosition`
  with a trigger position at the downward stable eqilibrium point. This exhibits
  a bouncing behavior.
* On the far right, the pendulum has only the `PulseJointVelocityCommand`.
  It starts out stationary, then alternates between moving at constant velocity
  and swinging passively.
* In the center, the pendulum has both plugins enabled and experiences pulses
  of constant velocity alternating with passive swinging and bouncing at the
  bottom.
