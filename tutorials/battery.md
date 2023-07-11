\page battery Battery

The battery system keeps track of the battery charge on a robot model.

Currently, one battery per model is supported. When the battery drains
completely, all joints of the corresponding model are turned off, meaning joint
forces are set to 0.

All logic for battery consumption are encapsulated in a plugin.


## A perfect battery

An ideal battery has a constant voltage while discharging and no internal
resistance. Here's a minimum example of a perfect battery that can be added to
a model:

```{.xml}
<model>
  ...
  <plugin filename="gz-sim-linearbatteryplugin-system"
        name="gz::sim::systems::LinearBatteryPlugin">
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
`<power_load>` is a consumer-specific parameter. You can set this to a
high value to see what happens when the battery drains. All others are
properties of the battery.

Next, you can find a description of the SDF parameters used:

* `<battery_name>`: The name of the battery.

* `<voltage>`: Initial voltage of the battery (V).

* `<open_circuit_voltage_constant_coef>`: Voltage at full charge (V).

* `<capacity>`: Total charge that the battery can hold (Ah).

* `<power_load>`: Power load on battery (W).

* `<fix_issue_225>`: As reported [here](https://github.com/gazebosim/gz-sim/issues/225),
there are some issues affecting batteries in Gazebo Blueprint and Citadel.
This parameter fixes the issues. Feel free to omit the parameter if you have
legacy code and want to preserve the old behavior.

* `<start_draining>`: Start draining battery from the begining of the
simulation. If this is not set the battery draining can only be started
through the topics set through .

* `<start_power_draining_topic>`: Topic(s) that can be used to start
power draining.

* `<stop_power_draining_topic>`: Topic(s) that can be used to stop power
draining.

* `<fix_issue_225>`: As reported
[here](https://github.com/gazebosim/gz-sim/issues/225), there are some issues
affecting batteries in Ignition Blueprint and Citadel. This parameter fixes the
issues. Feel free to omit the parameter if you have legacy code and want to
preserve the old behavior.

When setting the `<capacity>`, `<voltage>` of the battery and its
`<power_load>`, keep in mind the following formula:

`battery_runtime` (hours) = `<capacity>` * `<voltage>` / `<power_load>`

### Known limitations

If `<fix_issue_225>` is not set, the battery drains at a faster (100x) rate.
In this case, the battery runtime should be calculated as follows:

`battery_runtime` (hours) = `<capacity>` *
`<voltage>` / (`<power_load>` * 100)


## Try a more realistic battery

If you need to model a more realistic battery, you can use the following
advanced SDF parameters:

* `<open_circuit_voltage_linear_coef>`: Amount of voltage decrease when
no charge (V).

* `<initial_charge>`: Initial charge of the battery (Ah).

* `<resistance>`: Internal resistance (Ohm)

* `<smooth_current_tau>`: Coefficient for smoothing current.

Please, refer to the battery specification to set the advanced values.


## Charging

A battery can be charged if the SDF parameter `<enable_recharge>` is set to
true. Here are the relevant SDF parameters related with charging:

* `<enable_recharge>`: As mentioned, it should be `true` to enable recharging.

* `<charging_time>`: Hours taken to fully charge the battery. Keep in mind that
this value assumes no battery load while charging. If the battery is under
load, it will take a longer time to recharge.

* `<recharge_by_topic>`: If true, the start/stop signals for recharging the
battery will also be available via topics. The regular Gazebo services will
still be available.

By default, two Gazebo Transport services are available for managing charging:

* `/model/<model_name>/battery/<battery_name>/recharge/start`:
Enable recharging.
* `/model/<model_name>/battery/<battery_name>/recharge/stop`:
Disable recharging.

Both services accept an `gz::msgs::Boolean` parameter.

## Try out an example

A battery has been added to a demo world, which can be run using:

```
gz sim -v 4 linear_battery_demo.sdf -z 1000000
```

The blue vehicle on the left has a battery, while the one on the right does
not.  When the battery drains, the corresponding vehicle stops moving. Please,
see `gz-sim/examples/worlds/linear_battery_demo.sdf`, where you can
find the commands to visualize the state of the battery, as well as commands to
start and stop the recharging.


To control the vehicles with keyboard, run

```
cd gz-sim/examples/standalone/keyboard
mkdir build && cd build
cmake .. && make
./keyboard ../keyboard.sdf
```
See more about the usage of the keyboard plugin in
`examples/standalone/keyboard/README.md`.

## Creating battery consumers from other systems

You can easily create battery consumers from other systems by creating a
consumer entity that holds a `BatteryPowerLoad` component.

### Example of the Thruster system as a battery consumer

It is entirely up to the developer to set the means and ways to calculate
the power load of the system and the options for future users of the system.
Basically all the system needs to do is to set up an entity and add the
`BatteryPowerLoad` component with the corresponding information, the rest is
up to the developer. However the `Thruster` system is a good example of how to
make a system consume power from a certain battery.

The way the `Thruster` system allows users to set it as a battery consumer is
through the options `<power_load>` and `<battery_name>`. Through these options
the system user can set how much power load should the system consume and which
battery should it use. Take note that, with this approach of identifying
batteries the user would have to ensure that the battery names are unique
within the system.

#### Configure step

At the configure step the parameters are read and saved:

```
  if (_sdf->HasElement("power_load"))
  {
    if (!_sdf->HasElement("battery_name"))
    {
      gzerr << "Specified a <power_load> but missing <battery_name>."
          "Specify a battery name so the power load can be assigned to it."
          << std::endl;
    }
    else
    {
      this->dataPtr->powerLoad = _sdf->Get<double>("power_load");
      this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
    }
  }
```

Note that both are required, if no `power_load` present then `battery_name` is
ignored and if no `battery_name` is there when a `power_load` is set an error
will be thrown asking for this parameter.

#### PreUpdate step

During this step and only for one time the battery consumer will be set. This
is done in this step instead of doing it in the configure step to avoid race
conditions and make sure all the batteries are ready. The `batteryInitialized`
makes sure the initialization only happens once.

```
  // Intit battery consumption if it was set
  if (!this->dataPtr->batteryName.empty() &&
      !this->dataPtr->batteryInitialized)
  {
    this->dataPtr->batteryInitialized = true;
    ...
```

The battery entity is searched using the `ecm` and saved
to later add the info to the `BatteryPowerLoad` component.

```
    // Check that a battery exists with the specified name
    Entity batteryEntity;
    int numBatteriesWithName = 0;
    _ecm.Each<components::BatterySoC, components::Name>(
      [&](const Entity &_entity,
        const components::BatterySoC */*_BatterySoC*/,
        const components::Name *_name)->bool
      {
        if (this->dataPtr->batteryName == _name->Data())
        {
          ++numBatteriesWithName;
          batteryEntity = _entity;
        }
        return true;
      });
```

Some errors are thrown if no batteries or more than one batteries
are found with the specified name.

```
   if (numBatteriesWithName == 0)
    {
      gzerr << "Can't assign battery consumption to battery: ["
             << this->dataPtr->batteryName << "]. No batteries"
             "were found with the given name." << std::endl;
      return;
    }
    if (numBatteriesWithName > 1)
    {
      gzerr << "More than one battery found with name: ["
             << this->dataPtr->batteryName << "]. Please make"
             "sure battery names are unique within the system."
             << std::endl;
      return;
    }
```

Note that in this example the name is used to uniquely identify
a battery within a system but it is up to the developer to use any
other means they consider adequate. I.e. some sort of hierarchy can
be enforced in the system to help identify batteries using this hierarchy.

Finally, the consumer entity is created and the power load and battery
info added to its `BatteryPowerLoad` component:

```
    // Create the battery consumer entity and its component
    this->dataPtr->consumerEntity = _ecm.CreateEntity();
    components::BatteryPowerLoadInfo batteryPowerLoadInfo{
        batteryEntity, this->dataPtr->powerLoad};
    _ecm.CreateComponent(this->dataPtr->consumerEntity,
        components::BatteryPowerLoad(batteryPowerLoadInfo));
    _ecm.SetParentEntity(this->dataPtr->consumerEntity, batteryEntity);
  }
```

The `consumerEntity` is saved in case it has to be modified or even
deleted in the future. If a developer wants to change the power load they
just need to modify the component, setting it to 0 will stop the consumer
effect on the battery. Another option is to entirely delete the consumer
entity.

The battery plugin basically checks for entities with a `BatteryPowerLoad`
component and uses that information to calculate the total battery consumption
before the draining step happens. By modifying the values and quantities of
entities with `BatteryPowerLoad` components developers can add, modify and
remove battery consumers from the different batteries available in a system.

## Known Issues

* The rate of consumption should be affected by torque. For example, going
uphill should consume more power than going downhill.
