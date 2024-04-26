Example program to generate performance data

## Build Instructions

From this directory, run the following to compile:

```bash
cmake -B build
cmake --build build
```

### Run

```bash
cd build
./each
```

This will generate `./build/each.data` which is the performance data.

### Generate and view plots

One the above program is complete, generate the plots.

```bash
cd build
# Generate plots
gnuplot -e "filename='each.data'" ../each.gp
eog *.png
```

You can now scroll through two plots to view performance data.

