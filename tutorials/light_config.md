\page light_config Light config

This tutorial gives an introduction to Gazebo Sim's service `/world/<world name>/light_config`.
This service will allow to modify lights in the scene.

## Modifying lights

To modify lights inside the scene we need to use the service `/world/<world name>/light_config` and
fill the message [`gz::msgs::Light`](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Light.html).

This tutorial describes the code in the `examples/standalone/light_control` example.

First we create a point light with the function `createLight()`:

\snippet examples/standalone/light_control/light_control.cc create light

**NOTE:**: You can check the [entity creation](entity_creation.html) tutorial to learn how to include models and lights in the scene.

As you can see in the snippet below, we then modify the specular and diffuse colors of the point light in the scene.

\snippet examples/standalone/light_control/light_control.cc modify light

The `r`, `g`, `b` components of the light diffuse and specular colors are randomly generated and constantly changing:

\snippet examples/standalone/light_control/light_control.cc random numbers

## Run the example

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
 gz sim -r -v 4 empty.sdf
 ```

 - Terminal two:
 ```bash
 ./light_control
 ```
