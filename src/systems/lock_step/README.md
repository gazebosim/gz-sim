## LockStep Framework Prototype

The LockStep framework allows a System plugin to be run in a remote process, and
stepped synchronously with the sim server.

The motivation for the LockStep framework is to isolate the System runtime from
the sim server.
This allows sandboxing untrusted 3p System plugins for instance
in production.
At the same time, running the System synchronously with the sim server allows
the simulation to be run deterministically.

There are two parts in the LockStep framework.
- `LockStep` System: Added as a placeholder in the model/ world sdformat file
  instead of the actual plugin.
- `LockStepRuntime`: Instantiated and run from `runtime_main.cc`, this class
  loads the actual plugin and steps the target System.
  It hosts several gz transport service end-points that are called synchronously
  by the `LockStep` System to relay `EntityComponentManager` updates from the
  sim server.

### Example
An example world is available in `examples/worlds/lock_step.sdf` where the
`LockStep` System is used to run the `Sensors` System remotely.
A camera is present in the world, which is rendered from the remote `Sensors`
System.
A runtime xml config file at `examples/plugin/lock_step/runtime.config`
specifies that the `Sensors` System to be loaded and run in a `LockStepRuntime`.

After building gz-sim from source, on one terminal tab, start the runtime.
The runtime must be started first for now to avoid the `LockStem` System timing
out on a `/Configure` service call and skipping the System completely.
```
$ install/libexec/gz/sim10/lockstep-runtime \
    src/gz-sim/examples/plugin/lock_step/runtime.config
```

On another terminal tab, run gz sim:
```
$ GZ_SIM_RESOURCE_PATH=src/gz-sim/examples/worlds gz sim \
    src/gz-sim/examples/worlds/lock_step.sdf
```

(If developing headless over ssh, add `-s -r --headless-rendering` to enable
headless rendering with egl.)

In the gui, you should see a scene with a hundred bouncing balls.
Open the `Image Display` gui plugin and verify that images are being published
periodically on the `/test/camera1` topic.

You can change the remote System update rate by setting `<update_rate>` in the
`LockStep` System plugin xml in `examples/worlds/lock_step.sdf`.
A value of zero will step the remote System at the same rate as the server.
The actual update rate is capped at the step rate of the server itself (200Hz in
the above example).

You can also experiment with different transport implementations to see the
effect on performance (RTF).
Prepend `GZ_TRANSPORT_IMPLEMENTATION=zeromq` or `zenoh` to both the commands
above to test with either ZeroMQ or Zenoh.

### TODOs
- Send ECM updates from remote System back to server and apply them to the
  server ECM
- Parse service topics from xml instead of hardcoding
- Aggregate multiple `LockStep` systems into a `LockStepManager` world plugin
  that serializes the ECM updates only once to minimize overhead
- Parallelize lock step with server step to improve performance
- Explore shared memory communication to eliminate ECM serialization/
  deserialization/ transport overhead
- Track state in `LockStepRuntime` to prevent accidental out-of-order service
  calls (e.g. `PreUpdate`, `PostUpdate`, `PostUpdate`).
