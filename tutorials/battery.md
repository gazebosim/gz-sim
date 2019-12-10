\page battery Battery

The battery system keeps track of the battery charge on a robot model.

Currently, one battery per model is supported. When the battery drains completely, all joints of the corresponding model are turned off, meaning joint forces are set to 0.

All logic for battery consumption are encapsulated in a plugin.


## Try it out

A linear consumption battery plugin has been implemented. The battery can be added to a Model with:

```{.xml}
<model>
  ...
  <plugin filename="libignition-gazebo-linearbatteryplugin-system.so"
        name="ignition::gazebo::systems::LinearBatteryPlugin">
        <!--Li-ion battery spec from LIR18650 datasheet-->
        <battery_name>linear_battery</battery_name>
        <voltage>4.2</voltage>
        <open_circuit_voltage_constant_coef>4.2</open_circuit_voltage_constant_coef>
        <open_circuit_voltage_linear_coef>-2.0</open_circuit_voltage_linear_coef>
        <initial_charge>2.5</initial_charge>
        <capacity>2.5 </capacity>
        <resistance>0.07</resistance>
        <smooth_current_tau>2.0</smooth_current_tau>
        <enable_recharge>false</enable_recharge>
        <charging_time>3.0</charging_time>
        <soc_threshold>0.51</soc_threshold>
        <!-- Consumer-specific -->
        <power_load>2.1</power_load>
        <start_on_motion>true</start_on_motion>
      </plugin>
  ...
</model>
```
`<power_load>` is a consumer-specific parameter. You can set this to a high value to see what happens when the battery drains. All others are properties of the battery.

It's recommended to run this demo at a high RTF, so it's faster to observe the battery drain:

```
ign gazebo -v 4 linear_battery_demo.sdf -z 1000000
```

Move both vehicles:

```
ign topic -t "/model/vehicle_blue/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 6.0} angular: {z: 0.4}"
ign topic -t "/model/vehicle_green/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 6.0} angular: {z: 0.4}"
```

Listen to battery state of charge:

```
ign topic -e -t /model/vehicle_blue/battery/linear_battery/state
```

With `<start_on_motion>` set to `true`, the battery only starts draining when the vehicle starts moving; if set to `false`, the battery starts draining when the sdf file is loaded.
The blue vehicle on the left has a battery without recharge, while the green one on the right has a battery with recharge.
The blue vehicle on the left with <enable_recharge> set to false will eventually stop after running out of power.
The green vehicle on the right with <enable_recharge> set to true will recharge when the battery state of charge drains below the `<soc_threshold>`.

To control the vehicles with keyboard, run

```
cd ign-gazebo/examples/standalone/keyboard
mkdir build && cd build
cmake .. && make
./keyboard ../keyboard.sdf
```

One vehicle can be moved using arrow keys and the other using WASD keys. See more about the usage of the keyboard plugin in `examples/standalone/keyboard/README.md`.


## Known Issues

* The rate of consumption should be affected by torque. For example, going uphill should consume more power than going downhill.
