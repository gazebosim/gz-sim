\page pause_run_simulation Pause and Run simulation

A Gazebo transport API is exposed to allow starting and stopping the simulation.
It's possible to call this API using the command line or through the GUI.

To repeat this demo, run the `default` world:
```bash
gz sim default.sdf
```

## Transport API

When Gazebo is run headless, this is an easy way to start the simulation.

To pause and play over the transport API, we should call the service `/world/<world_name>/control` and fill the request message type
`gz.msgs.WorldControl`. This service returns a `gz.msgs.Boolean` with the status of the request (true means everything was fine, false otherwise).

The `WorldControl` message contains a `pause` field for starting and stopping the simulation.

To start the simulation:

```bash
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

To pause the simulation:

```bash
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'
```

When paused, time will stop in Gazebo, and the physics will not be running.

To check the current state of the simulator, check the `paused` field in `/stats` like so:
```bash
gz topic --echo --topic /stats -n 1
```
If the simulator is currently paused but was running before, we would see something similar to this:
```text
sim_time {
  sec: 8
  nsec: 707000000
}
real_time {
  sec: 8
  nsec: 824323281
}
iterations: 8707
real_time_factor: 0.998022916602211
```


## GUI

We included a button in the `World Control` plugin allowing to start and stop the simulation from the GUI.

@image html files/pause_run_simulation/gui_pause_run.png
