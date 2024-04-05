\page adding_visuals

# Overview

This tutorial describes how to import 3D meshes into Gazebo to increase the
visual fidelity of your model. Continuing with our example, we'll attach a 3D
mesh to our turtle, making it look much better.

## Related tutorials

https://gazebosim.org/api/sim/8/meshtofuel.html

The next tutorials, although still relevant, are from an older version of Gazebo
and some details might be different than the current versions:

https://classic.gazebosim.org/tutorials?tut=import_mesh&cat=build_robot
https://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i2

## What is a visual?

The visual element specifies the shape used by the rendering engine. For most
use cases the collision and visual elements are the same. The most common use
for different collision and visual elements is to have a simplified collision
element paired with a visual element that uses a complex mesh. This will help
improve performance.

SDF supports the notion of visual as described
[here](http://sdformat.org/spec?ver=1.10&elem=visual).

From our
[previous tutorial](https://gazebosim.org/api/sim/8/create_vehicle.html),
the turtle visual is a cylinder. Let's use a COLLADA mesh instead.

## Model directory structure

Gazebo has defined a model directory structure that supports stand-alone models,
and the ability to share models via an online model database. Review
[this tutorial](https://gazebosim.org/api/sim/8/meshtofuel.html) for more
information.

Another benefit of Gazebo's model structure is that it conveniently organizes
resources, such as mesh files, required by the model.

```bash
my_turtle
├── materials               Directory for textures
    └── textures
├── meshes                  Directory for mesh files such as COLLADA, STL, and Wavefront OBJ files
├── thumbnails              Directory for preview images on Fuel
├── model.config            Meta data about the model
└── model.sdf               SDF description of the model
```

Create the directories to add the mesh and its texture:

```bash
mkdir -p ~/gazebo_maritime/models/my_turtle/meshes
mkdir -p ~/gazebo_maritime/models/my_turtle/materials/textures
```

Next, download the COLLADA mesh and its texture.

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/adding_visuals/turtle.dae -O ~/gazebo_maritime/models/my_turtle/meshes/turtle.dae
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/adding_visuals/Turtle_BaseColor.png -O ~/gazebo_maritime/models/my_turtle/materials/textures/Turtle_BaseColor.png
```

Now, let's edit our `model.sdf` to use the new mesh as our visual.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="turtle">
    <static>true</static>
    <link name="base_link">

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
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 0.009948450858321252</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>meshes/turtle.dae</uri>
          </mesh>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
```

# Load your model in Gazebo

Launch Gazebo and load our model:

```bash
gz sim ~/gazebo_maritime/models/my_turtle/model.sdf
```

You should see your model visualized as a mesh now!

@image html files/adding_visuals/basic_visual_model.png
