# How to build the demo

# Sourcing ignition installation
source ~/ignf_workspace/install/setup.zsh

# Build
mkdir build
cd build
cmake .. ; make
cd ..

# Export the system so that ignition can find it 
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build/custom_system

# Test run
ign gazebo -r test.sdf 

