\page wheel_slip_systems Wheel Slip systems

# Overview

This tutorial shows how to configure slip parameters for wheeled vehicles using
two different wheel slip plugins.

Wheel slip configuration is useful for simulating scenarios where a vehicle
drives over different types of surfaces. For example, a vehicle climbing up
steep gravel road may start slipping and the Gazebo allows users to configure
the slip compliance and friction parameters of each individual wheel to
approximate this behavior instead of simulating each individual rock on the
road and its contact with the wheel.

# Wheel Slip systems

There are two systems in Gazebo related to wheel slip configuration:

1. Wheel slip system
1. Lookup wheel slip system

Both of the these systems are model plugins, meaning they should be added to a
model in the world. The wheel slip system allows users to configure slip
parameters for each wheel in the model. The lookup wheel slip system on the
other hand has to be used together with the wheel slip system. It provides extra
functionality that enables the slip parameters to be dynamically updated as the
vehicle moves around in the environment.


## Wheel Slip system

The wheel slip system updates wheel slip parameters based on linear wheel spin
velocity. Here are the parameters that can be configured per wheel link:

* `<slip_compliance_lateral>`: Unitless wheel slip compliance in lateral
  direction. The parameter should be non-negative, with a value of zero allowing
  no slip and larger values allowing increasing slip.
* `<slip_compliance_longitudinal>`: Unitless wheel slip compliance in longitudinal
  direction. The parameter should also be non-negative and larger values mean
  increasing slip.
* `<wheel_normal_force>`: Wheel normal force estimate used to compute slip
  compliance, which takes units of 1/N.
* `<wheel_radius>`: Radius of the wheel in meters.

You can find more detailed information on the wheel slip system and its
parameters in the
[WheelSlip.hh](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/wheel_slip/WheelSlip.hh)
header file.

Gazebo has an example world that demonstrates the use of the wheel slip system,
namely [trisphere_cycle_wheel_slip.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/trisphere_cycle_wheel_slip.sdf).

In this example, the world consists of two trisphere cycles and uses an inclined
gravity vector to simulate a sloped surface. The blue trisphere cycle on the
left has zero slip compliance values (no slip) while the red trisphere cycle
on the right has both the lateral and longitudinal slip compliance values
set to 1.

Launch Gazebo with the example world:

```bash
gz sim -v 4 trisphere_cycle_wheel_slip.sdf
```

Hit the Play button and you should see that the two trisphere cycles start
moving down the slope under the effect of the inclined gravity vector.
After some time, the red trisphere cycle picks up speed and wheel slippage
occurs, causing it to tumble. The blue trisphere cycle continues to roll down the
slope normally.

@image html files/wheel_slip_systems/wheel_slip.gif

## Lookup Wheel Slip system

The look up wheel slip system dynamically adjusts the wheel slip and friction
parameters based on the wheel's position within the region covered by a 'slip map'.
This system needs to be used together with the wheel slip system otherwise it
will not have any impact on the model it is attached to.

As the wheeled vehicle moves around in the environment, the lookup wheel slip
system looks up the corresponding lateral and longitudinal slip and friction
delta values encoded in the slip map based on the wheel position. These delta
values are applied against the 'nominal' slip parameters of the wheel link,
i.e. original slip compliance values specified in the wheel slip system and
the wheel link's origin friction (mu1 and mu2) values. This yields new
updated slip and friction values which are then sent to the wheel slip system
at run time.

The slip map is a lookup image map that represents the traversable region
of the world. The slip and friction values are encoded in the image's RGB channels.

Here are the lookup wheel slip system's parameters:
* `<slip_map>`: Path to the slip map image.
  * The red channel affects lateral slip, green affects longitudinal slip, and
    blue affects friction (mu1 and mu2).
* `<size_x>`: x size of lookup slip map in meters.
* `<size_y>`: y size of lookup slip map in meters.
* `<wheel_link_name>`: The wheel link name. Specify one `<wheel_link_name>`
  per wheel link.
* `<slip_compliance_lateral_delta>`: The delta amount to apply to the lateral slip.
* `<slip_compliance_longitudinal_delta>`: The delta amount to apply to
  longitudinal slip.
* `<friction_delta>`: The delta amount to apply to the friction coefficients
  (mu1, mu2).

You can find more detailed information on the lookup wheel slip system and its
parameters in the
[LookupWheelSlip.hh](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/lookup_wheel_slip/LookupWheelSlip.hh)
header file.

The [lookup_wheel_slip.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim10/examples/worlds/lookup_wheel_slip.sdf).
example world showcases the use of the lookup wheel slip system together with
the wheel slip system.

In this example, the world consists of two vehicles, blue and green,
on a flat heightmap. The blue vehicle has a lookup wheel slip system
and a regular wheel slip system while the green only has a regular
wheel slip plugin with fixed slip compliance values.

The slip map used in this demo contains several regions with increased
longitudinal slip compliance values (encoded in the green channel of the
slip map). The same slip map is overlaid on top of the heightmap to help
visualize where the slippage regions are.

Launch Gazebo with the example world:

```bash
gz sim -v 4 lookup_wheel_slip.sdf
```

Hit the Play button to see the two vehicles move forward. When the blue vehicle
drives over the green regions, the longitudinal slip compliance value of the
wheel that is in contact with the slip region will be dynamically updated
(increased) causing the wheel to slip and the whole vehicle to turn. The green
vehicle should move forward in a straight line without being affected by the slip
regions.

@image html files/wheel_slip_systems/lookup_wheel_slip.gif
