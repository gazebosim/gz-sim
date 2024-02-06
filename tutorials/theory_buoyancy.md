\page theory_buoyancy

# Overview

This tutorial describes the theory of operation of the buoyancy plugin, an
essential piece needed in most of the maritime simulations.

# The buoyancy plugin

This plugin is attached to the world to provide certain buoyancy to the
vehicles. The buoyancy force opposes gravity exerted on the robot. The
parameters of the plugin are configured via SDF in the world file. Check the
[Buoyancy.hh](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/buoyancy/Buoyancy.hh)
Gazebo header file for a complete description of all the accepted parameters.

The buoyancy plugin uses the `<collision>` elements of each SDF model to compute
the volume of fluid displaced.

A common pattern for the maritime domain is to create very thin collision
elements surrounding the vehicle (with low volume) and a separate collision
element within the vehicle itself. The collection of low volume collision
elements work as a shell to generate contacts when necessary. The extra
collision element is only used to generate the aggregated desired air volume.

The outer set of collision elements in combination with the buoyancy collision
element induce forces at the center of each `<collision>` element. If you
followed the recommendation for creating low-volume outer collisions, the
predominant force will be applied at the center of the buoyancy collision
element. You can tune the location of that collision with the `<pose>` tag,
to match your expected center of buoyancy.

The buoyancy force is proportional to the volume of air in the vehicle according
to this equation:

$$volume\\_neutral = \frac{mass}{waterDensity}$$

Pure water's density is `1000 kgm^-3` and seawater's density is `1025 kgm^-3`
approximately. The mass of your vehicle is something that you'll decide or know
by adding the mass of all your links. With that in mind, you'll be able to
compute the volume of air that will make your vehicle neutrally buoyant.

If your vehicle is neutrally buoyant, `volume_neutral` should be the total
amount of volume that you should have by adjusting the size of your
`<collision>` elements. If your vehicle is not neutrally buoyant, this value
will serve as a reference. If the total volume is smaller than `volume_neutral`,
the vehicle will sink. Otherwise, it will move up.

The `<GAZEBO_INSTALL_PATH>/examples/worlds/buoyancy.sdf` SDF file (included with
Gazebo) contains an example with three submarines. The first submarine is
neutrally buoyant, the second sinks, and the third floats. Run the following
command to see an example of a buoyancy plugin configured with a uniform fluid
density:

```bash
gz sim buoyancy.sdf
```

# Graded buoyancy

Often when simulating a maritime environment one may need to simulate both
surface and underwater vessels. This means the buoyancy plugin needs to
take into account two different fluids. One being water with a density of
`~1000 kgm^-3` and another being air with a very light density of say `1 kgm^-3`.
An example for such a configuration may be found in the
`<GAZEBO_INSTALL_PATH>/examples/worlds/graded_buoyancy.sdf` world (included
with Gazebo).

```bash
gz sim graded_buoyancy.sdf
```

You should be able to see a sphere bobbing up and down undergoing simple
harmonic motion on the surface of the fluid (this is expected behavior
as the harmonic motion is usually damped by the hydrodynamic forces. See the
hydrodynamics tutorial for an example of how to use it). The key part of this is

```xml
<graded_buoyancy>
  <default_density>1000</default_density>
  <density_change>
    <above_depth>0</above_depth>
    <density>1</density>
  </density_change>
</graded_buoyancy>
```

The default density tag says that by default the world has a fluid density
of `1000 kgm^-3`. This essentially states that by default the world is filled
with dihydrogen monoxide (aka water). The `<density_change>` tag essentially
establishes the fact that there is a another fluid. The `<above_depth>` tag says
that above z=0 there is another fluid with a different density. The density of
that fluid is defined by the `<density>` tag. We will be simulating air with a
fluid density of `1 kgm^-3`.

# Known limitations

When the buoyancy plugin is configured in `graded buoyancy mode`, only `<box>`
and `<sphere>` collision geometries are supported.
