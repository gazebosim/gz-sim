# Joystick

Standalone program that publishes
[gz::msgs::Joy](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Joy.html)
messages from a joystick device using Gazebo Transport.

The mapping of joystick buttons to fields in the message is the same as [this](http://wiki.ros.org/joy).

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/standalone/joystick
mkdir build
cd build
cmake ..
make
~~~

This will generate the `joystick` executable under `build`.

## Run

The executable expects an SDF file with configurations.
An example file, `joystick.sdf`, is provided.

You can run the example as follows:

    ./joystick ../joystick.sdf

You may get error messages if a joystick is not found or the SDF file is invalid.
Make sure you have a joystick connected and that the correct device is passed in
the SDF file's `<dev>` tag, which defaults to `/dev/input/js0`.

> **Tip**: If running inside docker, be sure to run the container with the
  `--device=/dev/input/js0` option.

If no errors are printed, you can check that the messages are being published by
echoing the `/joy` topic:

    gz topic echo -t /joy

## Demo example

Gazebo ships with an example file which has a differential drive vehicle
that can be controlled using a joystick. You can run it as follows:

1. In a terminal, run the joystick executable as described above to publish
   joystick messages:

        cd examples/standalone/joystick
        ./joystick ../joystick.sdf

1. On another terminal, run the `joy_to_twist` executable to convert joy
   messages to twist messages. See that standalone program's instructions for
   details on how to build it. Once it's built, you can run it as follows:

        cd examples/standalone/joy_to_twist
        ./joy_to_twist ../joy_to_twist.sdf

1. Finally, on a 3rd terminal, run `gz sim` with the vehicle that will
   consume the twist messages:

        cd examples/worlds
        gz sim -v 4 diff_drive.sdf

1. Now hold your joystick's A button (or equivalent) and move the directional
   stick to control the vehicle.
