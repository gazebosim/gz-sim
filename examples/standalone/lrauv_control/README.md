## Introduction
This example shows a simple controller for an LRAUV. The vehicle has 3 actuators : the thruster, the fins to control yaw angle, and the fins to control
pitch angle. Here, the target state of the vehicle is the speed, yaw and pitch angle, and the controller must adjust the fin angles and thruster to achieve
that state. The vehicle is a MIMO system in reality, where all the three actuators affect each other, but we can approximate this to be a SISO system when
the pich and yaw angles to be changed are not too drastic.

Therefore, we apply a PD controller for speed, and a P controller for yaw and pitch angles. The odometry publisher plugin supplies the required feedback
on the states.

## Usage
This example needs to be run in two steps.

Step 1 : Assuming ``~/gazebo_ws/`` is your Gazebo workspace, find the world file ``~/gazebo_ws/src/gz-sim/examples/worlds/lrauv_control_demo.sdf``.

Open a new terminal window, source your gazebo workspace and run :
```
cd ~/gazebo_ws/src/gz-sim/examples/worlds
gz sim -r lrauv_control_demo.sdf
```

This should open up a new gazebo window with the LRAUV at rest at the origin.

Step 2 : Open a new terminal, source your gazebo workspace, and navigate to this example.

```
cd ~/gazebo_ws/src/gz-sim/examples/standalone/lrauv_control
mkdir build; cd build
cmake ..; make

# Your controller executable should be built successfully
# Usage :
# ./lrauv_control speed_in_metres_per_sec yaw_angle_in_rad pitch_angle_in_rad
./lrauv_control 0.5 0.174 0.174
```

The vehicle should now move, tracing its path along the way. The console should show the errors for each state.

```
.
.
.
-----------------------
States            ( target, current, error) :
Speed (m/s)       : 0.50000 0.54543 -0.04543
Yaw angle (deg)   : 9.96947 11.31291 -1.34344
Pitch angle (deg) : 9.96947 6.39717 3.57229
-----------------------
States            ( target, current, error) :
Speed (m/s)       : 0.50000 0.54543 -0.04543
Yaw angle (deg)   : 9.96947 11.31291 -1.34344
Pitch angle (deg) : 9.96947 6.39717 3.57229
```

## Note
This is only meant to be an example, and the controller might not behave correctly at high speeds or sudden change in desired angles.
