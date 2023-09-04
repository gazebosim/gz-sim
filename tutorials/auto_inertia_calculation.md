\page auto_inertia_calculation Automatic Inertia Calculation for Links

## Automatic Inertia Calculation for SDFormat Links

This feature enables automatic calculation for the Moments of Inertia, Mass, and Inertial Pose (Center of Mass pose) of a link described using SDFormat. The following geometry types are currently supported
 for this feature:
 * Box
 * Capsule
 * Cylinder
 * Ellipsoid
 * Sphere
 * Mesh
   
Using this feature, a user can easily set up an accurate simulation with physically plausible inertial values for a link. This also removes the dependency on manual calculations or 3rd-party mesh processing software
which can help lower the barrier of entry for beginners.

This tutorial will focus on how this feature can be enabled and how the inertia values for a link can be configured. Some limitations and recommendations would also be discussed along the way that would allow users to more mindfully utilize this feature.

## Basic Overview

This feature introduced a new `auto` attribute for the `<inertial>` tag which can be set to true or false (The value is false by default) to enable or disable the automatic calculations respectively. In 
case, `auto` is set to true, the constituent **collision geometries** of the link are considered for the calculations. A newly introduced `<density>` tag can be used to specify the mass density value
of the collision in kg/m^3. The density of water (1000 kg/m^3) is utilized as the default value.
In case of multiple collision geometries in a link, a user is free to provide different density values for each and the inertia values from each would be aggregated to calculate the final inertia of the link.
Now let's see some examples below!

## Automatic Inertia Calculations for Basic Shapes
