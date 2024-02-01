\page create_vehicle

# Overview

This tutorial goes through the process of designing a maritime model to be used
in Gazebo. Our basic model will be a turtle. First, we'll start simple and will
add more features as we progress in the tutorials.

## Related tutorials

https://gazebosim.org/docs/harmonic/building_robot

# Design a basic SDF model

Create a workspace to store your brand new model named `my_turtle`.

```bash
mkdir -p ~/gazebo_maritime/models/my_turtle && cd ~/gazebo_maritime/models/my_turtle
```

Each model must have a `model.config` file in the model's root directory that
contains meta information about the model. Create a `model.config` file:

```xml
<?xml version="1.0"?>
<model>
  <name>my_turtle</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Carlos Agüero</name>
    <email>caguero@openrobotics.org</email>
  </author>

  <description>
    My 3D turtle.
  </description>
</model>
```

You can find a description of the allowed elements in `model.config` in
[this Gazebo Classic tutorial](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot).

Create a `model.sdf` file that contains the Simulator Description Format of the
model. You can find more information on the [SDF website](http://sdformat.org/).

```xml
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name="my_turtle">
    <static>true</static>
    <link name='base_link'>

      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>10</mass>
        <inertia>
          <ixx>0.35032999999999995</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.35032999999999995</iyy>
          <iyz>0</iyz>
          <izz>0.61250000000000006</izz>
        </inertia>
      </inertial>

      <collision name='collision'>
        <geometry>
          <box>
            <size>1 1 0.009948450858321252</size>
          </box>
        </geometry>
      </collision>

      <visual name='visual'>
        <pose>0.08 0 0.05 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.35</radius>
            <length>0.23</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

The `model.sdf` file contains the necessary tags to instantiate a very minimal
model named `my_turtle` using SDF version 1.6.

# Load your model in Gazebo

Launch Gazebo and load our model:

```bash
gz sim ~/gazebo_maritime/models/my_turtle/model.sdf
```

You should see your model visualized as a cylinder and the Gazebo `Entity Tree`
should capture its structure.

@image html files/create_vehicle/basic_model.png
