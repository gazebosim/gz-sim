# Hello world plugin

A simple system plugin to print hello world.

## Folder structure

* Source file: HelloWorld.hh
* Main file: HelloWorld.cc
* CMakeLists.txt
* Package.xml
* libHelloWorld.so  

## Generate the Plugin 

``` bash 
cd Hello_world
mkdir build/
cd build/
cmake ..
make
```

It should generate a libHelloWorld.so file 
we can import it to ignition gazebo:
``` bash 
cd Hello_world
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
```


