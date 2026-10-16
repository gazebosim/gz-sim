# Performance with SDF Runner

The `sdf_runner` script is used to run given SDF configurations in an instrumented environment.  The script will execute the sdf simulation for a given number of iterations, and then save statistics about the simulation to a `csv` file.

This runs a headless simulation environment, and is additionally useful for running in `valgrind`, `perf`, or other performance tools.

## Using SDF Runner

From the build directory, run `make PERFORMANCE_sdf_runner` to build the executor.

### Parameters

Arguments are parsed in order:

1. SDF File to execute.
1. Number of iterations to run the simulation
1. Update rate in Hz (Default is 1000)

Example: `./PERFORMANCE_sdf_runner cubes.sdf 5000 10000`

Pass `--headless-rendering` anywhere on the command line to render with EGL, as
`gz sim --headless-rendering` does. Worlds with rendering sensors need it on a
machine without a display.

## Generating benchmark worlds

`gen_perf_world.py` writes reproducible SDF worlds with N copies of a small
differential-drive robot (`amr`) on a grid. Every model is inline, so a run
needs no Fuel access. The output depends only on the arguments.

The world loads only the systems named in `--plugins`, and sets
`include_server_config_plugins` to false. SceneBroadcaster and UserCommands
are absent unless requested. The step size is 1 ms and
`<real_time_factor>` is 0, so the server steps as fast as it can.

All robots receive the same constant velocity command (0.5 m/s forward, 0.2
rad/s yaw by default). The DiffDrive systems share one topic, and a
TriggeredPublisher system publishes the command whenever the world statistics
message arrives. Each robot drives a circle of 2.5 m radius, so it stays near
its grid cell. The default grid spacing of 8 m keeps robots apart.

```
# 100 robots, physics and DiffDrive only
./gen_perf_world.py --robots 100 --plugins physics -o amr_100.sdf

# 50 robots with the default server systems and one gpu_lidar each
./gen_perf_world.py --robots 50 --plugins physics,scene_broadcaster,user_commands,sensors \
    --lidar gpu_lidar --lidar-samples 720 --lidar-rate 10 --lidar-fov 270 \
    --warehouse -o amr_50_lidar.sdf

./PERFORMANCE_sdf_runner amr_50_lidar.sdf 5000 --headless-rendering
```

Run `gen_perf_world.py --help` for the robot count, spacing, plugin list,
physics engine, lidar options, step size, and commanded speeds. The optional
`--warehouse` adds static shelves between the robot columns so lidars see
geometry.

## Analyzing the output

The runner will generate a `data.csv` file that can then be used with the `gz_perf.py` tool to generate statistics and plots of the real time factor information.

Examples:

* `gz_perf.py data.csv --summarize` Summarize RTF statistics

```
Iterations: 10000
Mean RTF:   0.93054
Median RTF: 0.93015
Min RTF:    0.00244
  Iteration: 0
  Sim Time:  0.001
  Real Time: 0.82727
Max RTF:    1.01867
  Iteration: 634
  Sim Time:  0.63500
  Real Time: 1.96814
```

* `gz_perf.py data.csv --plot` Time series plot of RTF vs simualation time

* `gz_perf.py data.csv --hist` Histogram of real time factors
