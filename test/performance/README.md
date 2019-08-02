# Performance with SDF Runner

The `sdf_runner` script is used to run given SDF configurations in an instrumented environment.  The script will execute the sdf simulation for a given number of iterations, and then save statistics about the simulation to a `csv` file.

This runs a headless simulation environment, and is additionally useful for running in `valgrind`, `perf`, or other performance tools.

## Using SDF Runner

From the build directory, run `make PERFORMANCE_sdf_runner` to build the executor.

### Parameters

* `-v <level>` Control the verbosity of output 0-4
* `-z <rate>` Update rate in Hz (Default is 1000)
* `--iterations` Number of iterations to run the simulation
* `-f` SDF File to execute.

Example: `./PERFORMANCE_sdf_runner -z 100000 -f cubes.sdf`

## Analyzing the output

The runner will generate a `data.csv` file that can then be used with the `ign_perf.py` tool to generate statistics and plots of the real time factor information.

Examples:

* `ign_perf.py data.csv --summarize` Summarize RTF statistics

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

* `ign_perf.py data.csv --plot` Time series plot of RTF vs simualation time

* `ign_perf.py data.csv --hist` Histogram of real time factors

