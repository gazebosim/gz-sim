\page light_config Light config

This tutorial gives an introduction to the Ignition Gazebo's service `/world/<world name>/light_config`.
This service will allow to modify lights in the scene.

# Modifying lights

To modify lights inside the scene we need to use the service `/world/<world name>/light_config` and
fill the message [`ignition::msgs::Light`](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Light.html).
In particular this example modify the point light that we introduced with the function `createLight()`.

\snippet examples/standalone/light_control/light_control.cc create light

**NOTE:**: You can check the [model creation](model_creation.html) tutorial to learn how to include models and lights in the scene.

As you can see in the snippet we modify the specular and diffuse of the light that corresponds to type of light in the scene.

\snippet examples/standalone/light_control/light_control.cc modify light

In this case we are creating random numbers to fill the diffuse and specular.

\snippet examples/standalone/light_control/light_control.cc random numbers

# Run the example

To run this example you should `cd` into `examples/standalone/light_control` and build the code:

```bash
mkdir build
cd build
cmake ..
make
```

Then you should open two terminals and run:

 - Terminal one:
 ```bash
 ign gazebo -r -v 4 empty.world
 ```

 - Terminal two:
 ```bash
 ./light_control
 ```
