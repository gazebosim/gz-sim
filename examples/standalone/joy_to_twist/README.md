# Joy to Twist

Standalone program that subscribes to
[ignition::msgs::Joy](https://ignitionrobotics.org/api/msgs/5.6/classignition_1_1msgs_1_1Joy.html)
messages and converts publishes
[ignition::msgs::Twist](https://ignitionrobotics.org/api/msgs/5.6/classignition_1_1msgs_1_1Twist.html)
messages according to user-defined configuration.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd examples/standalone/joy_to_twist
mkdir build
cd build
cmake ..
make
~~~

This will generate the `joy_to_twist` executable under `build`.

## Run

The executable expects an SDF file with configurations.
An example file, `joy_to_twist.sdf`, is provided.

You can run the example as follows:

    ./joy_to_twist ../joy_to_twist.sdf

This program by itself won't do much unless there is another source of joy
messages that it can consume. See the demo below for a full integrated example.

## Demo example

Ignition Gazebo ships with an example file which has a differential drive vehicle
that can be controlled using a joystick. You can run it as follows:

1. In a terminal, run the joystick executable which will publish joystick
   messages. See that standalone program's instructions to details on how
   to build it. Once it's built, you can run it as follows:

        cd examples/standalone/joystick
        ./joystick ../joystick.sdf

1. On another terminal, run the `joy_to_twist` executable as described above,
   which will convert joy messages to twist messages:

        cd examples/standalone/joy_to_twist
        ./joy_to_twist ../joy_to_twist.sdf

1. Finally, on a 3rd terminal, run `ign gazebo` with the vehicle that will
   consume the twist messages:

        cd examples/worlds
        ign gazebo -v 4 diff_drive.sdf

1. Now hold your joystick's A button (or equivalent) and move the directional
   stick to control the vehicle.

