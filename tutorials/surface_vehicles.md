\page surface_vehicles

# Overview

This tutorial explains how to create and load a maritime surface vehicle in
Gazebo. This type of vehicle usually has multiple thrusters and navigate
with the presence of waves and wind.

## Related tutorials

https://gazebosim.org/api/sim/8/create_vehicle.html

https://gazebosim.org/api/sim/8/adding_visuals.html

https://gazebosim.org/api/sim/8/frame_reference.html

https://gazebosim.org/api/sim/8/adding_system_plugins.html

https://gazebosim.org/api/sim/8/theory_hydrodynamics.html

# Adding an environment

We'll start this tutorial creating a workspace with some custom models, worlds,
and plugins that are not available yet on Gazebo. You could use this workspace
as a template when in need of customizing Gazebo for your needs.

To compile all the custom libraries in the right order `colcon` is recommended.
The `colcon` tool is available on all platforms using `pip3`.

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/surface_vehicles/gz_maritime_ws.zip -O ~/gz_maritime_ws.zip
unzip ~/gz_maritime_ws.zip
```

## Generic tools

Install tools needed by this tutorial:

```bash
sudo apt install python3-pip wget
pip3 install -U colcon-common-extensions
```

## Setup the workspace

Let's now build our workspace:

```bash
cd ~/gazebo_maritime_ws
colcon build --merge-install
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/gazebo_maritime_ws/install/share/gazebo_maritime/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/gazebo_maritime_ws/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/gazebo_maritime_ws/install/lib
```

And run the simulation:

```bash
gz sim -r src/gazebo_maritime/worlds/sydney_regatta.sdf
```

@image html files/surface_vehicles/sydney_regatta.png

Gazebo is loaded with the Sydney Regatta Center!

# Create your vehicle

It's time to create our maritime surface vehicle. We're going to use the
Wave Adaptive Modular Vehicle (WAM-V) as the surface platform. This is the
robot that we use in the [VRX project](https://github.com/osrf/vrx) as well.

## Basic SDF

Let's start with the basic SDF of the WAM-V platform. Open with your favorite
editor the file
`~/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf`.

Notice how the vehicle is composed by multiple links:

```bash
wam-v
└────────────────────────────── base_link
  left_chasis_engine_joint└─────── left_engine_link
     left_engine_propeller_joint└─────── left_propeller_link

  right_chasis_engine_joint└─────── right_engine_link
    right_engine_propeller_joint└─────── right_propeller_link
```

The main chassis with the two rigid pontoons are captured by `base_link`.
A revolute joint (`left_chasis_engine_joint`) connects the chasis and the
`left_engine_link`. The purpose of this connection is to allow the engine to
rotate. This is an additional degree of freedom that we expose to make the
thruster control more flexible. Then, the `left_engine_link` is connected to the
`left_propeller_link` via the revolute joint `left_engine_propeller_joint`.
The purpose of this connection is to allow the propeller to spin. This
kinematic structure is repeated on the right side.

Uncomment now the following block in the `sydney_regatta.sdf` file:

```xml
<!-- Uncomment this block to load the WAM-V model-->
<include>
  <name>wam-V</name>
  <pose>-532 162 0 0 0 1</pose>
  <uri>wam-v</uri>
</include>
```

And launch Gazebo:

```bash
gz sim src/gazebo_maritime/worlds/sydney_regatta.sdf
```

@image html files/surface_vehicles/wamv.png

If you hit play, your WAM-V will sink (sad but expected). There's no buoyancy
enabled yet.

## Checking frame of reference

Launch your world in Gazebo as we did before:

```bash
gz sim src/gazebo_maritime/worlds/sydney_regatta.sdf
```

You can visualize the frame of reference of your vehicle by loading the plugin
`Transform Control` from the GUI.

@image html files/lander/gui_transform_control.png

Then, click on the vehicle, and then, click on the `Translate mode` icon. You'll
now visualize your WAM-V frame of reference. As this is the expected frame of
reference, we won't need to apply changes in the COLLADA mesh or add any
`<pose>` offset to our `model.sdf`.

@image html files/surface_vehicles/wamv_reference_frame.png

## Adding plugins

### The Surface plugin

Normally, there's a section here where we explain how to add buoyancy to our
model. It's totally possible to do it that way but for stability reasons, we
have created a more specific plugin to simulate buoyancy in the presence of
waves, the `Surface` plugin.

Open the file `~/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf`
with your favorite editor and look at the `Surface` plugin:

```xml
<!-- Uncomment to produce buoyancy on the left hull -->
<plugin
  filename="libSurface.so"
  name="maritime::Surface">
  <link_name>base_link</link_name>
  <hull_length>4.9</hull_length>
  <hull_radius>0.213</hull_radius>
  <fluid_level>0</fluid_level>
  <points>
    <point>0.6 1.03 0</point>
    <point>-1.4 1.03 0</point>
  </points>
  <wavefield>
    <topic>/gazebo/wavefield/parameters</topic>
  </wavefield>
</plugin>
```

Before we go into the weeds of the plugin, you can guess from the comment that
this plugin simulates the buoyancy generated by the left pontoon. A very similar
plugin is attached to the right pontoon.

The `Surface` plugin simulates the buoyancy of an object at the surface of
a fluid. This system/plugin must be attached to a model and the system applies
buoyancy to a collection of points around a given link.

This plugin models the vehicle's buoyancy assuming a single hull with a
cylindrical shape. You can instantiate multiple plugins when your vehicle has
multiple pontoons.

This plugin also supports waves. If you provide a wavefield topic via SDF, the
plugin will receive the wavefield parameters and account for the delta Z that
the waves generate at each point.

Here are the detailed parameters:

* `<link_name>` is the name of the link used to apply forces.
* [Optional] `<hull_length>` is the length of the vessel [m].
* [Optional] `<hull_radius>` is the radius of the vessel's hull [m].
* [Optional] `<fluid_level>` is the depth at which the fluid should be in the
             vehicle
* [Optional] `<fluid_density>` is the density of the fluid.
* [Optional] `<points>` contains a collection of points where the forces
             generated by this plugin will be applied. See the format of each
             point next:
* [Optional] `<points><point>` Relative position of the point relative to
             `link_name`.
* [Optional] `<wavefield>`: The wavefield parameters.
             See `~/gazebo_maritime_ws/src/gazebo_maritime/src/Wavefield.hh`.

Go ahead and uncomment the two blocks in
`~/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf` to
instantiate the `Surface` plugins and see the effect on your WAM-V:

```bash
colcon build --merge-install
gz sim -r src/gazebo_maritime/worlds/sydney_regatta.sdf
```

Now your vehicle is floating in the water!

### Hydrodynamics

I know, I know, it looks like a dancing catamaran. The reason for that behavior
is because we don't have any drag yet to oppose the forces of the `Surface`
plugins. We need hydrodynamics!

We will be using Fossen's equations which describe the motion of a craft through
the water for this. For better understanding of the parameters here, I would
refer you to [his book](https://www.wiley.com/en-sg/Guidance+and+Control+of+Ocean+Vehicles-p-9780471941132)
(check your library).
Usually these parameters can be found via fluid simulation programs or
experimental tests in a water tub.

Uncomment the following block from
`~/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf` to enable
hydrodynamics.

```xml
<!-- Hydrodynamics -->
<plugin
  filename="gz-sim-hydrodynamics-system"
  name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <xDotU>0.0</xDotU>
  <yDotV>0.0</yDotV>
  <nDotR>0.0</nDotR>
  <xU>-51.3</xU>
  <xAbsU>-72.4</xAbsU>
  <yV>-40.0</yV>
  <yAbsV>0.0</yAbsV>
  <zW>-500.0</zW>
  <kP>-50.0</kP>
  <mQ>-50.0</mQ>
  <nR>-400.0</nR>
  <nAbsR>0.0</nAbsR>
</plugin>
```

And run Gazebo:

```bash
colcon build --merge-install
gz sim -r src/gazebo_maritime/worlds/sydney_regatta.sdf
```

The WAM-V is much more stable now and see how it moves following the movement
of the waves.

You could experiment with the wavefield parameters to see different behaviors
of the WAM-V floating on the waves. For example, update the following block
in the `~/gazebo_maritime_ws/src/gazebo_maritime/worlds/sydney_regatta.sdf`
world file:

```xml
<!-- The wave field -->
<plugin filename="libPublisherPlugin.so" name="maritime::PublisherPlugin">
  <message type="gz.msgs.Param" topic="/gazebo/wavefield/parameters"
           every="2.0">
    params {
      key: "direction"
      value {
        type: DOUBLE
        double_value: 0.0
      }
    }
    params {
      key: "gain"
      value {
        type: DOUBLE
        double_value: 0.8
      }
    }
    params {
      key: "period"
      value {
        type: DOUBLE
        double_value: 5
      }
    }
    params {
      key: "steepness"
      value {
        type: DOUBLE
        double_value: 0
      }
    }
  </message>
</plugin>
```

The gain parameter has been increased to `0.8` producing bigger waves. You
should expect your WAM-V to move more agresively up and down.

Both the `WaveVisual` plugin (used for rendering the waves) and the `Surface`
plugin (used to simulate the physics of the waves) are updated via the same
topic. The `PublisherPlugin` periodically publishes the wavefield parameters
to make sure that all the wave-related plugins work with the same wavefield
parameters.

### Thruster

We need the vehicle to move, so we will be adding the `Thruster` plugin. In the
case of the WAM-V, we need two thrusters. Each thruster plugin takes in a force
and applies it along with calculating the desired rpm. Uncomment the following
block from your
`~/gazebo_maritime_ws/src/gazebo_maritime/models/wam-v/model.sdf` model:

```xml
<plugin
  filename="gz-sim-thruster-system"
  name="gz::sim::systems::Thruster">
  <joint_name>left_engine_propeller_joint</joint_name>
  <thrust_coefficient>0.004422</thrust_coefficient>
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
  <velocity_control>true</velocity_control>
</plugin>

<plugin
  filename="gz-sim-joint-position-controller-system"
  name="gz::sim::systems::JointPositionController">
  <joint_name>left_chasis_engine_joint</joint_name>
  <use_velocity_commands>true</use_velocity_commands>
  <topic>/wamv/left/thruster/joint/cmd_pos</topic>
</plugin>

<plugin
  filename="gz-sim-thruster-system"
  name="gz::sim::systems::Thruster">
  <joint_name>right_engine_propeller_joint</joint_name>
  <thrust_coefficient>0.004422</thrust_coefficient>
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
  <velocity_control>true</velocity_control>
</plugin>

<plugin
  filename="gz-sim-joint-position-controller-system"
  name="gz::sim::systems::JointPositionController">
  <joint_name>right_chasis_engine_joint</joint_name>
  <use_velocity_commands>true</use_velocity_commands>
  <topic>/wamv/right/thruster/joint/cmd_pos</topic>
</plugin>
```

And start Gazebo again:

```bash
colcon build --merge-install
gz sim -r src/gazebo_maritime/worlds/sydney_regatta.sdf
```

Send a command to rotate the left thruster:

```bash
gz topic -t /wamv/left/thruster/joint/cmd_pos -m gz.msgs.Double -p 'data: -0.15'
```

Send a command to spin the right propeler:

```bash
gz topic -t /model/wam-V/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10.00'
```

And see how your WAM-V starts moving, bon voyage!
