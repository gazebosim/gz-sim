# Ignition Plotting

### Build
```bash
mkdir build
cd build
cmake ..
make
export IGN_GUI_PLUGIN_PATH=`pwd`
```

### Try the Plotting

* Run the Keyboard to control the blue car
```bash
cd /PATH_TO_IGN_GAZEBO/examples/standalone/keyboard/build
./keyboard ../keyboard.sdf
```

* Run ignition gazebo and open the levels sdf
```bash
ign gazebo levels.sdf
```

* Open GazeboPlotting


