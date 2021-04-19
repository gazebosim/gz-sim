# random velocity plugin


## generate libRandomVelocity.so file

``` bash 
cd random_vel
mkdir build/
cd build/
cmake ..
make
```

It should generate a libRandomVelocity.so file 

## to load the file
``` bash 
cd random_vel
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
```

## if the path is not exported properly, the libRandomVelocity.so file will not be loaded in the plugin
``` bash 
cd random_vel/build
sudo cp libRandomVelocity.so /usr/lib/x86_64-linux-gnu/ign-gazebo-5/plugins
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
``` 

