# Hello world plugin

A simple system plugin to print hello world.

## Generate the Plugin 

It is neccesary to have the repo [ign-plugin](https://github.com/ignitionrobotics/ign-plugin)

## Folder structure

* Source file: HelloWorld.cc
* CMakeLists.txt

``` bash 
 cd hello_world
mkdir build/
cd build/
cmake ..
make
```

It should generate a _libHelloWorld.so_ file 
we can import it to ignition gazebo:
``` bash 
cd hello_world
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
```
