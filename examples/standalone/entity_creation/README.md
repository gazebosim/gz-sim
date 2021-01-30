# Examples using the create service

## Build

```
mkdir build
cd build
cmake ..
make
```

## Run

This example only works if the world is called `empty`. Start an empty world with:

```
ign gazebo empty.sdf
```

Then run the create program to spawn entities into the world:

```
cd build
./entity_creation
```
