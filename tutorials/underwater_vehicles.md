\page underwater_vehicles

# Overview

Gazebo now supports basic simulation of underwater vehicles.
This capability is based on the equations described in Fossen's ["Guidance and
Control of Ocean Vehicles"](https://www.wiley.com/en-sg/Guidance+and+Control+of+Ocean+Vehicles-p-9780471941132).
This tutorial will guide you through the steps you
need to setup simulation of an underwater vehicle. In this tutorial, we will
guide you through the setup of the [MBARI LRAUV](https://app.gazebosim.org/accurrent/fuel/models/MBARI%20Tethys%20LRAUV).

## Related tutorials

https://gazebosim.org/api/sim/8/create_vehicle.html

https://gazebosim.org/api/sim/8/adding_visuals.html

https://gazebosim.org/api/sim/8/frame_reference.html

https://gazebosim.org/api/sim/8/adding_system_plugins.html

https://gazebosim.org/api/sim/8/theory_buoyancy.html

https://gazebosim.org/api/sim/8/theory_hydrodynamics.html

# Create your vehicle

As an example, we'll go through the process of creating an MBARI LRAUV
underwater vehicle.

Create a workspace to store your brand new model named `my_lrauv`.

```bash
mkdir -p ~/gazebo_maritime/models/my_lrauv/materials/textures
mkdir -p ~/gazebo_maritime/models/my_lrauv/meshes
```

Download all the files from here and copy them within that directory:

```bash
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/model.config -O ~/gazebo_maritime/models/my_lrauv/model.config
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/model.sdf -O ~/gazebo_maritime/models/my_lrauv/model.sdf
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/materials/textures/Tethys_Albedo.png -O ~/gazebo_maritime/models/my_lrauv/materials/textures/Tethys_Albedo.png
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/materials/textures/Tethys_Metalness.png -O ~/gazebo_maritime/models/my_lrauv/materials/textures/Tethys_Metalness.png
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/materials/textures/Tethys_Normal.png -O ~/gazebo_maritime/models/my_lrauv/materials/textures/Tethys_Normal.png
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/materials/textures/Tethys_Roughness.png -O ~/gazebo_maritime/models/my_lrauv/materials/textures/Tethys_Roughness.png
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/meshes/base.dae -O ~/gazebo_maritime/models/my_lrauv/meshes/base.dae
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/meshes/propeller.dae -O ~/gazebo_maritime/models/my_lrauv/meshes/propeller.dae
wget -r https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv/meshes/tethys.dae -O ~/gazebo_maritime/models/my_lrauv/meshes/tethys.dae
```

Open with your favorite editor `~/gazebo_maritime/models/my_lrauv/model.sdf`.

Notice how the vehicle has been divided into multiple links: chassis, horizontal
fins, vertical fins, and a propeller. This is a very common division in vehicles
that contain one or more thrusters and some control surfaces. Both the propeller
and the control surfaces are movable, thus we need to create separate links for
them.

At the bottom of the `model.sdf` file, you'll find the three joints that connect
the horizontal fins, vertical fins, and propeller to the main chassis
respectively. All of them are `revolute` joints, but with different upper and
lower limits. While the propeller does not have any limits (to allow infinite
rotation), the fins have lower values to constraint the amount of movement of
the fin. Again, this is a very common pattern that you'll need to configure
based on the specs of your vehicle.

Let's run the simulation:

```bash
gz sim ~/gazebo_maritime/models/my_lrauv/model.sdf
```

You should see Gazebo with your LRAUV loaded. If you hit play, your robot will
sadly sink. The good news is that this is expected as we did not attach any
plugins to the LRAUV model. Let's do that!

# Adding buoyancy

The first thing we need to decide is how much buoyancy we want to provide to our
vehicle. In the LRAUV case, we want a robot with neutral buoyancy. In other
words, our robot should maintain depth when stationary.

Buoyancy works by opposing the weight of the vehicle. In order to compute the
amount of buoyancy force, first we need to sum the total weight of our LRAUV.
There's a total of `148.3571` kg (`147.8671` + `0.2` + `0.2` + `0.09` kgs) when
we add the mass of the chassis, fins and propeller. The buoyancy force is
proportional the volume of air in the vehicle according to this equation:

$$volume\\_neutral = \frac{mass}{waterDensity}$$

In our case:

$$volume\\_neutral = \frac{148.3571}{1000} = 0.1483571$$

That's the total volume of air that our vehicle should contain to keep the robot
neutral. If the volume is smaller, the vehicle will sink. If the vehicle's
volume is bigger, it will move up. The buoyancy plugin uses all the
`<collision>` elements of the model to compute volume.

Let's verify the total amount of air volume in our vehicle adding up the volumes
of all the collision elements:

$$volume = (2 * 0.3 * 0.2464451666666667) + (0.1 * 0.1 * 0.02) + (0.1 * 0.1 * 0.02) + (0.03 * 0.1 * 0.03) = 0.14835710000000002$$

This is the volume that we were looking for to achieve neutral buoyancy. Let's
test it! Uncomment the lines in `model.sdf:18--24` to restore the buoyancy and
launch the simulation:

Let's now download the following world that includes the buoyancy plugin:

```bash
mkdir -p ~/gazebo_maritime/worlds
wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/buoyant_lrauv.sdf -O ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
export GZ_SIM_RESOURCE_PATH=:$HOME/gazebo_maritime/models
```

And launch Gazebo:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
```

Now your LRAUV should stay in place! Let's experiment with the buoyancy values.
Edit the `main_body_buoyancy` geometry in your `model.sdf` to be:

```xml
<box>
  <size>2 0.3 0.24</size>
</box>
```

And now relaunch the simulation:

```bash
gz sim ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
```

Your LRAUV should slowly sink. Feel free to experiment with other buoyancy
values but remember to restore the original value:

```xml
<box>
  <size>2 0.3 0.2464451666666667</size>
</box>
```

# Adding thrusters

We need the vehicle to move, so we will be adding the `Thruster` plugin. The
thruster plugin takes in a force and applies it along with calculating the
desired rpm. Uncomment the following block from your `buoyant_lrauv.sdf` world:

```xml
<plugin
  filename="gz-sim-thruster-system"
  name="gz::sim::systems::Thruster">
  <namespace>my_lrauv</namespace>
  <use_angvel_cmd>0</use_angvel_cmd>
  <joint_name>propeller_joint</joint_name>
  <thrust_coefficient>0.004422</thrust_coefficient>
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
</plugin>
```

And start the simulation:

```bash
gz sim -r ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
```

Now if we were to publish to `/model/my_lrauv/joint/propeller_joint/cmd_thrust`

```bash
gz topic -t /model/my_lrauv/joint/propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: -15'
```

We should see the model move. The thrusters are governed by the equation on
page 246 of Fossen's book. In particular it relates force to rpm as follows:

$$thrust = fluid\\_density * RPM^2 * thrust\\_constant * propeller\\_blade\\_size^4$$

The plugin takes in commands in newtons. So if you have a different thrust
curve you can still use the plugin with some type of adapter script. The thrust
constant is normally determined by individual manufacturers. In this case we are
using the LRAUV's thrust coefficient. you may also build a test rig to measure
your thruster's thrust coefficient.

# Adding hydrodynamics

You may notice that the robot now keeps getting faster and faster. This is
because there is no drag to oppose the thruster's force. We will be using
Fossen's equations which describe the motion of a craft through the water for
this. For better understanding of the parameters here, I would refer you to
his book. Usually these parameters can be found via fluid simulation programs or
experimental tests in a water tub.

Uncomment the following block from `buoyant_lrauv.sdf` to enable hydrodynamics.

```xml
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

# Adding control surfaces

Just like aeroplanes, an underwater vehicle may also use fins for stability and
control. Fortunately, Gazebo already has a version of the LiftDrag plugin. In
this tutorial, we will simply add two liftdrag plugins to the rudder and
elevator of MBARI's LRAUV. For more info about the liftdrag plugin including
what the parameters mean, you may look
at [this gazebo classic tutorial](http://gazebosim.org/tutorials?tut=aerodynamics&cat=physics).
Essentially when we tilt the fins, we should experience a lift force which
will cause the vehicle to experience a torque and the vehicle should start
turning when we move.

These are the relevant blocks to enable the LiftDrag plugins. They're already
attached to the model for you.

```xml
<!-- Vertical fin -->
<plugin
filename="gz-sim-lift-drag-system"
name="gz::sim::systems::LiftDrag">
  <air_density>1000</air_density>
  <cla>4.13</cla>
  <cla_stall>-1.1</cla_stall>
  <cda>0.2</cda>
  <cda_stall>0.03</cda_stall>
  <alpha_stall>0.17</alpha_stall>
  <a0>0</a0>
  <area>0.0244</area>
  <upward>0 1 0</upward>
  <forward>1 0 0</forward>
  <link_name>vertical_fins</link_name>
  <cp>0 0 0</cp>
</plugin>

<!-- Horizontal fin -->
<plugin
filename="gz-sim-lift-drag-system"
name="gz::sim::systems::LiftDrag">
  <air_density>1000</air_density>
  <cla>4.13</cla>
  <cla_stall>-1.1</cla_stall>
  <cda>0.2</cda>
  <cda_stall>0.03</cda_stall>
  <alpha_stall>0.17</alpha_stall>
  <a0>0</a0>
  <area>0.0244</area>
  <upward>0 0 1</upward>
  <forward>1 0 0</forward>
  <link_name>horizontal_fins</link_name>
  <cp>0 0 0</cp>
</plugin>
```

The numbers in this case were kindly provided by MBARI for the LRAUV.
We also need to be able to control the position of the thruster fins so we will
use the joint controller plugin. These are the relevant blocks to enable the
control surfaces. They're already attached to the model for you.

```xml
<plugin
filename="gz-sim-joint-position-controller-system"
name="gz::sim::systems::JointPositionController">
  <joint_name>horizontal_fins_joint</joint_name>
  <p_gain>0.1</p_gain>
</plugin>

<plugin
filename="gz-sim-joint-position-controller-system"
name="gz::sim::systems::JointPositionController">
  <joint_name>vertical_fins_joint</joint_name>
  <p_gain>0.1</p_gain>
</plugin>
```

Launch Gazebo:

```bash
gz sim -r ~/gazebo_maritime/worlds/buoyant_lrauv.sdf
```

We should now be able to wiggle the fins using the following command:

```bash
gz topic -t /model/my_lrauv/joint/vertical_fins_joint/0/cmd_pos -m gz.msgs.Double -p 'data: -0.17'
```

Now apply some thrust:

```bash
gz topic -t /model/my_lrauv/joint/propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: -15'
```

@image html files/underwater_vehicles/lrauv_turn_left.png

And observe how the LRAUV turns due to the rudder's position. You can stop the
propeller with:

```bash
gz topic -t /model/my_lrauv/joint/propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0'
```

And observe how eventually the LRAUV stops due to the water drag.
