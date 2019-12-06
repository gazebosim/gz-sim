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
    <battery_name>linear_battery</battery_name>
    <voltage>12.592</voltage>
    <open_circuit_voltage_constant_coef>12.694</open_circuit_voltage_constant_coef>
    <open_circuit_voltage_linear_coef>-3.1424</open_circuit_voltage_linear_coef>
    <initial_charge>1.1665</initial_charge>
    <capacity>1.2009</capacity>
    <resistance>0.061523</resistance>
    <smooth_current_tau>1.9499</smooth_current_tau>
    <power_load>6.6</power_load>
  </plugin>
  ...
</model>
```
`<power_load>` is a consumer-specific parameter. You can set this to a high value to see what happens when the battery drains. All others are properties of the battery.

This has been added to a demo world, which can be run using:

```
ign gazebo -v 4 -r linear_battery_demo.sdf
```

The blue vehicle on the left has a battery, while the one on the right does not. When the battery drains, the corresponding vehicle stops moving.

To control the vehicles with keyboard, run

```
cd ign-gazebo/examples/standalone/keyboard
mkdir build && cd build
cmake .. && make
./keyboard ../keyboard.sdf
```

See more about the usage of the keyboard plugin in `examples/standalone/keyboard/README.md`.


## Known Issues

* The rate of consumption should be affected by torque. For example, going uphill should consume more power than going downhill.
