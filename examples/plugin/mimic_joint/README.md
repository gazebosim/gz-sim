# MimicJoint

## Generate the Plugin 
It is neccesary to have the repository
ign-plugin

The folder contains 3 files:

* Header file: MimicJoint.hh
* Source file: MimicJoint.cc
* CMakeList.txt

``` bash 
 cd mimic_joint
mkdir build/
cd build/
cmake ..
make
```

It should generate a .so file 
we can import it to ignition gazebo:

``` bash 
cd mimic_joint
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
```
## Note
Model file is yet to be worked on.