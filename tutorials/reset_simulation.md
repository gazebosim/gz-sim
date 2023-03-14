\page reset_simulation Reset simulation

The Reset Gazebo transport API is exposed to allow resetting simulation to time zero.
It's possible to call this API using the command line or through the GUI.
In addition to the API, we have also expanded the simulation system API with a Reset interface.

# Reset interface

System authors may now choose to implement the Reset interface to have a more intelligent
reset process (avoiding reloading assets or regenerating scene graphs being the motivating examples).
Since this interface is opt-in, systems that don't implement the API will still be reset via destruction and reconstruction.
The [physics](https://github.com/gazebosim/gz-sim/blob/gz-sim7/src/systems/physics/Physics.cc#L928-L937) and [rendering systems](https://github.com/gazebosim/gz-sim/blob/gz-sim7/src/systems/scene_broadcaster/SceneBroadcaster.cc#L452-L458) are the first two to implement this optimized reset functionality, with more to come as it makes sense to.

Following the tutorial \subpage createsystemplugins we should implement `ISystemReset` interface.

# Transport API

To call the reset transport API we should call the service `/world/default/control` and fill the request message type
`gz.msgs.WorldControl`, this service return a `gz.msgs.Boolean` with the status of the reset (true is everything was fine, false otherwise)

The `WorldControl` message now contains a reset field that we should filled if we want to reset the world:

```bash
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
```

# GUI

We included a new button in the `World Control` plugin allowing to reset the simulation from the GUI

@image html files/reset_simulation/reset_button.gif
