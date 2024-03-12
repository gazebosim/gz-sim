\page adding_system_plugins

# Overview

This tutorial explains how to add plugins to your model to provide extra
capabilities to it.

## Prerequisites

Make sure to go through the following tutorial first, where you'll learn how
to create the vehicle used in this tutorial.

https://gazebosim.org/api/sim/8/create_vehicle.html

## Related tutorials

https://gazebosim.org/api/sim/8/createsystemplugins.html

# Adding a system plugin

[This Gazebo tutorial](https://gazebosim.org/api/sim/8/createsystemplugins.html)
describes what is a system plugin in depth. Intuitively, you can envision a
system plugin as a piece of code that modifies the behavior of the simulation
when the general physics engine does not exactly capture your needs.
In our example, our turtle does not move because it's configured as a static
model. Let's see what happens if you remove that tag.

Modify your `~/gazebo_maritime/models/my_turtle/model.sdf` and remove the line
 `<static>true</static>`. Then launch the simulation:

```bash
gz sim ~/gazebo_maritime/models/my_turtle/model.sdf
```

Hit the play button and you'll see how your turtle falls into the void. Perhaps
not what you expected but this is completely normal. Gazebo thinks that your
turtle is in the air without any support underneath. Then, gravity makes your
turtle to free fall forever.

If we want to simulate that our turtle floats like if it was in the water,
we'll need to attach a custom buoyancy plugin to our world. This buoyancy plugin
already exists in Gazebo, we only need to load it.

Now, run Gazebo with the provided `buoyant_turtle.sdf` world and you'll see how
your turtle does not sink anymore.

```bash
mkdir -p ~/gazebo_maritime/worlds
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/adding_system_plugins/buoyant_turtle.sdf -O ~/gazebo_maritime/worlds/buoyant_turtle.sdf
export GZ_SIM_RESOURCE_PATH=:$HOME/gazebo_maritime/models
gz sim -r ~/gazebo_maritime/worlds/buoyant_turtle.sdf
```

The turtle now stays floating with an oscillating movement up and down.

You just added buoyancy to your model! As a general rule, a maritime model will
need at least two system plugins:

1. Buoyancy
2. Hydrodynamics

As you have experienced, the buoyancy plugin generates an upthrust opposing some
weight of the model. Your model could have positive, neutral or negative
buoyancy. You'll be able to tune that aspect of your model later.

The hydrodynamics plugin models the force and torque that the vehicle
experiences when moving within a fluid. Intuitively, the hydrodynamics plugin
generates drag opposing the movement of the vehicle. If your model does not have
hydrodynamics, it will behave as if there's no resistance.

For the sake of illustrating the effect of hydrodynamics, let's attach a simple
controller to our turtle and move it without hydrodynamics. Keep in mind that
the goal is to move the model one meter and stop it.

Uncomment the following block from `buoyant_turtle.sdf`:

```xml
<plugin filename="gz-sim-trajectory-follower-system"
        name="gz::sim::systems::TrajectoryFollower">
  <link_name>base_link</link_name>
  <force>0.2</force>
  <torque>0.01</torque>
  <range_tolerance>0.1</range_tolerance>
  <waypoints>
    <waypoint>1 0</waypoint>
  </waypoints>
</plugin>
```

And run Gazebo:

```bash
gz sim -r ~/gazebo_maritime/worlds/buoyant_turtle.sdf
```

As you just observed, we failed in our goal and the turtle behaved as if it was
moving on ice. Now, let's add hydrodynamics.

Uncomment the following block from `buoyant_turtle.sdf`:

```xml
<plugin
  filename="gz-sim-hydrodynamics-system"
  name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <xDotU>-0.04876161</xDotU>
  <yDotV>-1.26324739</yDotV>
  <zDotW>-1.26324739</zDotW>
  <kDotP>0</kDotP>
  <mDotQ>-0.3346</mDotQ>
  <nDotR>-0.3346</nDotR>
  <xUabsU>-0.62282</xUabsU>
  <xU>-5</xU>
  <yVabsV>-60.127</yVabsV>
  <yV>-5</yV>
  <zWabsW>-6.0127</zWabsW>
  <zW>-100</zW>
  <kPabsP>-0.001916</kPabsP>
  <kP>-1</kP>
  <mQabsQ>-6.32698957</mQabsQ>
  <mQ>-1</mQ>
  <nRabsR>-6.32698957</nRabsR>
  <nR>-1</nR>
</plugin>
```

And run Gazebo:

```bash
gz sim -r ~/gazebo_maritime/worlds/buoyant_turtle.sdf
```

Now, when our simple trajectory controller reaches its target and stops appling
force, the turtle stops moving acting like the fluid decelerates its motion.
Additionally you can notice how the up and down oscillations are also damped by
the effect of the hydrodynamics.

The hydrodynamics are also configurable with its SDF parameters but we'll talk
about configuration in a separate tutorial.
