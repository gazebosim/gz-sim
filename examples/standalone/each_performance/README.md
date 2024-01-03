Example program to generate performance data

### Build

    cd examples/standalong/each_performance
    mkdir build
    cd build
    cmake ..
    make

### Run

    ./each

### Generate and view plot

    gnuplot -e "filename='each.data'" ../each.gp
    eog *.png
