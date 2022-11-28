## Running benchmarks.

1. Change directory to the `build` folder.

2. Executed a benchmark, such as `./bin/BENCHMARK_ecm_serialize`.

3. If you need JSON output use `--benchmark_out_format=json`. For example:

    ```
    ./bin/BENCHMARK_ecm_serialize --benchmark_out_format=json --benchmark_out=results.json
    ```

### Comparing benchmark results

Given a set of changes to the codebase, it is often useful to see the difference in performance.

Once two (or more) benchmarks have been executed, compare the results by downloading the [benchmark tools](https://github.com/google/benchmark/tree/master/tools), and then run the following command:

```
# From the downloaded "tools" folder
./compare.py benchmarks baseline.json contender.json
```

### CPU Scaling Warnings

Note: If you receive warnings about CPU scaling, you can change the CPU governor with:

```
sudo cpupower frequency-set --governor performance
```

The previous setting can be restored with:

```
sudo cpupower frequency-set --governor powersave
```
