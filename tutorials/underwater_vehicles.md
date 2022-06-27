\page underwater_vehicles Underwater vehicles
# Simulating Autnomous Underwater Vehicles

Gazebo now supports basic simulation of underwater vehicles.
This capability is based on the equations described in Fossen's ["Guidance and
Control of Ocean Vehicles"](https://www.wiley.com/en-sg/Guidance+and+Control+of+Ocean+Vehicles-p-9780471941132).
This tutorial will guide you through the steps you
need to setup simulation of an underwater vehicle. In this tutorial, we will
guide you through the setup of the [MBARI Tethys](https://app.gazebosim.org/accurrent/fuel/models/MBARI%20Tethys%20LRAUV).
One can find the final sdf file for this tutorial in the
`examples/worlds/auv_controls.sdf` file.

# Understanding Hydrodynamic Forces
The behaviour of a moving body through water is different from the behaviour of
a ground based vehicle. In particular bodies moving underwater experience much
more forces derived from drag, buoyancy and lift. The way these forces act on
a body can be seen in the following diagram:
![force diagram](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo5/tutorials/files/underwater/MBARI%20forces.png)

# Setting up the buoyancy plugin
The buoyancy plugin in Gazebo uses the collision mesh to calculate the volume
of the vehicle. Additionally, it needs to know the density of the fluid in which
it is moving. By default this is set to 1000kgm^-3. However, in real life this
may vary depending on many factors like depth, salinity of water etc. To add
the buoyancy plugin all one needs to do is add the following under the `<world>`
tag:
```xml
<plugin
      filename="gz-sim-buoyancy-system"
      name="gz::sim::systems::Buoyancy">
    <uniform_fluid_density>1000</uniform_fluid_density>
</plugin>
```
# Setting up the thrusters
We need the vehicle to move, so we will be adding the `Thruster` plugin. The
thruster plugin takes in a force and applies it along with calculating the desired
rpm. Under the `<include>` or `<model>` tag add the following:
```xml
<plugin
    filename="gz-sim-thruster-system"
    name="gz::sim::systems::Thruster">
    <namespace>tethys</namespace>
    <joint_name>propeller_joint</joint_name>
    <thrust_coefficient>0.004422</thrust_coefficient>
    <fluid_density>1000</fluid_density>
    <propeller_diameter>0.2</propeller_diameter>
</plugin>
```
Now if we were to publish to `/model/tethys/joint/propeller_joint/cmd_pos`
```
gz topic -t /model/tethys/joint/propeller_joint/cmd_pos \
   -m gz.msgs.Double -p 'data: -31'
```
we should see the model move. The thrusters are governed by the equation on
page 246 of Fossen's book. In particular it relates force to rpm as follows:
`Thrust = fluid_density * RPM^2 * thrust_constant * propeller blade size^4`.
The plugin takes in commands in newtons. So if you have a different thrust
curve you can still use the plugin with some type of adapter script. The thrust
constant is normally determined by individual manufacturers. In this case we are
using the Tethys's thrust coefficient. you may also build a test rig to measure
your thruster's thrust coefficient.

# Adding Hydrodynamic behaviour
You may notice that the robot now keeps getting faster and faster. This is
because there is no drag to oppose the thruster's force. We will be using
Fossen's equations which describe the motion of a craft through the water for
this. For better understanding of the parameters here, I would refer you to
his book. Usually these parameters can be found via fluid simulation programs or
experimental tests in a water tub.
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
    <xUU>-6.2282</xUU>
    <xU>0</xU>
    <yVV>-601.27</yVV>
    <yV>0</yV>
    <zWW>-601.27</zWW>
    <zW>0</zW>
    <kPP>-0.1916</kPP>
    <kP>0</kP>
    <mQQ>-632.698957</mQQ>
    <mQ>0</mQ>
    <nRR>-632.698957</nRR>
    <nR>0</nR>
</plugin>
```

# Control surfaces
Just like aeroplanes, an underwater vehicle may also use fins for stability and
control. Fortunately, Gazebo already has a version of the LiftDrag plugin. In
this tutorial, we will simply add two liftdrag plugins to the rudder and
elevator of MBARI's Tethys. For more info about the liftdrag plugin inluding
what the parameters mean you may look
at [this gazebo classic tutorial](http://gazebosim.org/tutorials?tut=aerodynamics&cat=physics).
Essentially when we tilt the fins, we should experience a lift force which
will cause the vehicle to experience a torque and the vehicle should start
turning when we move.

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
The number in this case were kindly provided by MBARI for the Tethys.
We also need to be able to control the position of the thruster fins so we will
use the joint controller plugin.
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
We should now be able to wiggle the fins using the following command:
```
gz topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos \
  -m gz.msgs.Double -p 'data: -0.17'
```

# Testing the system out

To control the rudder of the craft run the following
```
gz topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos \
   -m gz.msgs.Double -p 'data: -0.17'
```
To apply a thrust you may run the following command
```
gz topic -t /model/tethys/joint/propeller_joint/cmd_pos \
-m gz.msgs.Double -p 'data: -31'
```
The vehicle should move in a circle.

# Ocean Currents

When underwater, vehicles are often subject to ocean currents. The hydrodynamics
plugin allows simulation of such currents. We can add a current simply by
publishing the following:
```
gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 1, y:0, z:0'
```
You should observe your vehicle slowly drift to the side.
