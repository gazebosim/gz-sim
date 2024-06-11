\page reset_simulation Reset simulation

The Reset Gazebo transport API is exposed to allow resetting simulation to time zero.
It's possible to call this API using the command line or through the GUI.
In addition to the API, we have also expanded the simulation system API with a Reset interface.

To repeat this demo, run the `rolling_shapes.sdf` file:
```bash
gz sim rolling_shapes.sdf
```

## Reset interface

System authors may now choose to implement the Reset interface to have a more intelligent
reset process (avoiding reloading assets or regenerating scene graphs being the motivating examples).
Since this interface is opt-in, systems that don't implement the API will still be reset via destruction and reconstruction.
The [physics](https://github.com/gazebosim/gz-sim/blob/23881936d93d335a2ad1086008416f1f36c3fdcc/src/systems/physics/Physics.cc#L919-L928) and [scene_broadcaster](https://github.com/gazebosim/gz-sim/blob/23881936d93d335a2ad1086008416f1f36c3fdcc/src/systems/scene_broadcaster/SceneBroadcaster.cc#L489-L495) systems are the first two to implement this optimized reset functionality, with more to come as it makes sense.

Follow the tutorial \subpage createsystemplugins to see how to support Reset by implementng the `ISystemReset` interface.

## Transport API

To invoke reset over transport API, we should call the service `/world/<world_name>/control` and fill the request message type
`gz.msgs.WorldControl`. This service returns a `gz.msgs.Boolean` with the status of the reset (true means everything was fine, false otherwise).

The `WorldControl` message now contains a `reset` field for resetting the world:

```bash
# the world name is `default` in this example
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
```

## GUI

We included a new button in the `World Control` plugin allowing to reset the simulation from the GUI

@image html files/reset_simulation/reset_button.gif
