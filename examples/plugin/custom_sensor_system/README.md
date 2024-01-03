# Custom sensor system

This example shows how to use a custom sensor with Gazebo.

It uses the odometer created on this example:
[gz-sensors/examples/custom_sensor](https://github.com/gazebosim/gz-sensors/tree/main/examples/custom_sensor).

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/plugins/custom_sensor_system
mkdir build
cd build
cmake ..
make
~~~

This will generate the `OdometerSystem` library under `build`.

## Run

The plugin must be attached to an entity to be loaded. This is demonstrated in
the `odometer.sdf` file that's going to be loaded.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd examples/plugins/custom_sensor_system
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world:

    gz sim -r odometer.sdf

You should see a box slowly moving in a straight line.

Listen to the odometer data with:

```
gz topic -e -t /world/odometer_world/model/model_with_sensor/link/link/sensor/an_odometer/odometer
```
