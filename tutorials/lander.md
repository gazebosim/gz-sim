\page lander

# Overview

This tutorial explains how to create and load an underwater lander vehicle in
Gazebo. This type of vehicles don't usually have control surfaces or thrusters.
They typically sink until they reach the seafloor or certain depth and go back
to the surface while they collect sensor measurements.

## Related tutorials

https://gazebosim.org/api/sim/8/create_vehicle.html

https://gazebosim.org/api/sim/8/adding_visuals.html

https://gazebosim.org/api/sim/8/frame_reference.html

https://gazebosim.org/api/sim/8/adding_system_plugins.html

https://gazebosim.org/api/sim/8/theory_buoyancy.html

https://gazebosim.org/api/sim/8/theory_hydrodynamics.html

# Create your vehicle

As an example, we'll go through the process of creating an Inkfish underwater
lander vehicle.

## Basic SDF

Create a workspace to store your brand new model named `my_lander`.

```bash
mkdir -p ~/gazebo_maritime/models/my_lander && cd ~/gazebo_maritime/models/my_lander
```

Download the `model.config` file and copy it within that directory:

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/lander/model.config -O ~/gazebo_maritime/models/my_lander/model.config
```

In its simple version, the lander does not have any moving pieces, so the SDF
model only has one single link, `base_link` in our case.

Create a `model.sdf` and paste the following content:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="lander">
    <link name="base_link">
      <inertial>
        <!-- Center of mass -->
        <pose>0 0 -0.982494 0 0 0</pose>

        <mass>1209.175</mass>
        <inertia>
          <ixx>205.227945746</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>412.254844006</iyy>
          <iyz>0</iyz>
          <izz>383.659583213</izz>
        </inertia>
      </inertial>

      <visual name="chassis_visual">
        <geometry>
          <box>
            <size>1.325 1.1503 1.5</size>
          </box>
        </geometry>
      </visual>
    </link>

  </model>
</sdf>
```

We can now test it in Gazebo. Launch it with:

```bash
gz sim ~/gazebo_maritime/models/my_lander/model.sdf
```

You should see your model visualized as a box.

@image html files/lander/simple_lander.png

## Adding visuals

Let's now add nicer-looking visuals to our lander. Download the following
`COLLADA` mesh file from here.

```bash
mkdir ~/gazebo_maritime/models/my_lander/meshes
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/lander/inkfish-lander.dae -O ~/gazebo_maritime/models/my_lander/meshes/inkfish-lander.dae
```

Replace the `chassis_visual` element in your `model.sdf` with the following
block:

```xml
<visual name="chassis_visual">
  <geometry>
    <mesh>
      <uri>meshes/inkfish-lander.dae</uri>
    </mesh>
  </geometry>
</visual>
```

And launch Gazebo to see the results:

```bash
gz sim ~/my_models/my_lander/model.sdf
```

Your box should now be replaced with a better looking mesh.

@image html files/lander/lander_visuals.png

## Checking frame of reference

Launch your lander in Gazebo as we did before:

```bash
gz sim ~/gazebo_maritime/models/my_lander/model.sdf
```

You can visualize the frame of reference of your vehicle by loading the plugin
`Transform Control` from the GUI.

@image html files/lander/gui_transform_control.png

Then, click on the vehicle, and then, click on the `Translate mode` icon. You'll
now visualize your lander frame of reference. As this is the expected frame of
reference, we won't need to apply changes in the COLLADA mesh or add any
`<pose>` offset to our `model.sdf`.

## Adding plugins

### Buoyancy

We are going to start extending our model adding buoyancy properties. For that,
we need to do two things:

1. Create a Gazebo world SDF file and load the buoyancy world plugin.
2. Add `<collision>` elements to our lander model SDF.

Let's start with the world. Download the following world:

```bash
mkdir -p ~/gazebo_maritime/worlds
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/lander/buoyant_lander.sdf -O ~/gazebo_maritime/worlds/buoyant_lander.sdf
export GZ_SIM_RESOURCE_PATH=:$HOME/gazebo_maritime/models
```

Now, open your `~/gazebo_maritime/models/my_lander/model.sdf` and add the low
volume outer collision elements to the `base_link`:

```xml
<collision name="collision_top">
  <pose>0 0 1.5 0 0 0</pose>
  <geometry>
    <box>
      <size>1.325 1.1503 0.000001</size>
    </box>
  </geometry>
</collision>

<collision name="collision_rear">
  <pose>0.6625 0 0.75 0 0 0</pose>
  <geometry>
    <box>
      <size>0.000001 1.1503 1.5</size>
    </box>
  </geometry>
</collision>

<collision name="collision_front">
  <pose>-0.6625 0 0.75 0 0 0</pose>
  <geometry>
    <box>
      <size>0.000001 1.1503 1.5</size>
    </box>
  </geometry>
</collision>

<collision name="collision_starboard">
  <pose>0 0.57515 0.75 0 0 0</pose>
  <geometry>
    <box>
      <size>1.325 0.000001 1.5</size>
    </box>
  </geometry>
</collision>

<collision name="collision_port">
  <pose>0 -0.57515 0.75 0 0 0</pose>
  <geometry>
    <box>
      <size>1.325 0.000001 1.5</size>
    </box>
  </geometry>
</collision>

<collision name="collision_bottom">
  <pose>0 0 0 0 0 0</pose>
  <geometry>
    <box>
      <size>1.325 1.1503 0.000001</size>
    </box>
  </geometry>
</collision>
```

You can now launch Gazebo:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lander.sdf
```

And visualize collisions by right-clicking on the lander, and the, `View` ->
`Collisions`.

@image html files/lander/lander_collisions.png

If you hit play, your lander will sink as expected. Although the buoyancy plugin
is loaded, notice that all the collision elements that we added are low-volume.
This is on purpose to make sure that our model can collide with obstacles.
However, it's generating very little buoyancy.

Let's now compute the reference volume that will make the lander neutrally
buoyant.

$$volume\\_neutral = \frac{mass}{waterDensity} = \frac{1209.175}{1025} = 1.1796829268292683$$

Now, let's account for the volume that our lander already has thanks to the
low-volume outer collision elements:

$$outer\\_volume = 2*1.1325*1.1503*0.000001 + 2*0.000001*1.1503*1.5 + 2*1.325* 0.000001*1.5 = 1.00313295e-05$$

Let's now compute the volume of the main buoyancy `<collision>` element:

$$main\\_buoyancy\\_volume = 1.1796829268292683 - 1.00313295e-05 = 1.1796728954997684$$

Let's verify that our lander is neutrally buoyant now. Add the following
collision element to your `base_link`:

```xml
<!-- Center of buoyancy -->
<collision name="buoyancy">
  <pose>0 0 0 0 0 0</pose>
  <geometry>
    <box>
      <size>1.1796728954997684 1 1</size>
    </box>
  </geometry>
</collision>
```

And launch Gazebo:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lander.sdf
```

Hit play, and your lander should now maintain depth!

This an example to illustrate how to calculate this reference value, however a
lander is normally designed to sink.

Change the buoyancy plugin to the following value to be more realistic:

```xml
<!-- Center of buoyancy -->
<collision name="buoyancy">
  <pose>0 0 0 0 0 0</pose>
  <geometry>
    <box>
      <size>1.14 1 1</size>
    </box>
  </geometry>
</collision>
```

And launch Gazebo:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lander.sdf
```

Hit play and the lander will slowly sink and hit the seafloor.

### Hydrodynamics

As an underwater vehicle, the lander should be influenced by the water as it
moves. We can do that by attaching the `hydrodynamics` plugin to our lander.
Add the next SDF block to your `model.sdf`. Note that we have empirically
adjusted its values. Follow the
[hydrodynamics tutorial](https://gazebosim.org/api/sim/8/theory_hydrodynamics.html)
for recommendations about how to tune its values.

```xml
<!-- Hydrodynamics -->
<plugin
  filename="gz-sim-hydrodynamics-system"
  name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <xDotU>-4.876161</xDotU>
  <yDotV>-126.324739</yDotV>
  <zDotW>-126.324739</zDotW>
  <kDotP>0</kDotP>
  <mDotQ>-33.46</mDotQ>
  <nDotR>-33.46</nDotR>
  <xUabsU>-6.2282</xUabsU>
  <xU>0</xU>
  <yVabsV>-601.27</yVabsV>
  <yV>0</yV>
  <zWabsW>-601.27</zWabsW>
  <zW>0</zW>
  <kPabsP>-0.1916</kPabsP>
  <kP>0</kP>
  <mQabsQ>-632.698957</mQabsQ>
  <mQ>0</mQ>
  <nRabsR>-632.698957</nRabsR>
  <nR>0</nR>
</plugin>
```

### Drop weight

A common feature in maritime landers is to include a drop weight device. When
this device is enabled, some part of the vehicle is detached from the main
chassis. At this point, the vehicle becomes positive buoyant, and starts moving
up towards the water surface.

In order to simulate this behavior, we'll need to make two changes to our lander
model:

1. Add a new separate link to act as the drop weight.
2. Attach a plugin to disconnect the previous link on demand.

Let's add the following SDF block to your `model.sdf`:

```xml
<!-- Drop weight -->
<link name="drop_weight">
  <pose>0 0 0.25 0 0 0</pose>
  <visual name="drop_weight_visual">
    <geometry>
      <box>
        <size>0.5 0.5 0.5</size>
      </box>
    </geometry>
  </visual>
  <collision name="drop_weight_collision">
    <geometry>
      <box>
        <size>0.5 0.5 0.000001</size>
      </box>
    </geometry>
  </collision>
  <inertial>
    <mass>120</mass>
  </inertial>
</link>

<plugin
  filename="gz-sim-detachable-joint-system"
  name="gz::sim::systems::DetachableJoint">
  <parent_link>base_link</parent_link>
  <child_model>__model__</child_model>
  <child_link>drop_weight</child_link>
  <topic>/model/lander/drop_weight</topic>
</plugin>
```

The mass of the drop weight changes the buoyancy properties of the lander. Let's
calculate the reference values:

$$volume\\_neutral\\_with\\_dropweight = \frac{mass}{waterDensity} = \frac{1209.175 + 120}{1025} = 1.2967560975609755$$

$$volume\\_neutral\\_without\\_dropweight = \frac{mass}{waterDensity} = \frac{1209.175}{1025} = 1.1796829268292683$$

We're looking for a volume that's lower than `volume_neutral_with_dropweight` to
cause the lander to sink from its initial configuration, and bigger than
`volume_neutral_without_dropweight` to cause the lander to go back to the
surface when the drop weight is enabled. Let's choose `1.24` as the volume for
our main buoyancy collision element. Replace the `buoyancy` collision element
with:

```xml
<!-- Center of buoyancy -->
<collision name="buoyancy">
  <pose>0 0 0 0 0 0</pose>
  <geometry>
    <box>
      <size>1.24 1 1</size>
    </box>
  </geometry>
</collision>
```

And launch Gazebo:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lander.sdf
```

Hit play and observe how the lander sinks until it hits the seafloor. Then,
enable the drop weight to detach the extra link:

```bash
gz topic -t "/model/lander/drop_weight" -m gz.msgs.Empty -p "unused: true"
```

Observe how your lander comes to the surface.
