# Examples using the create service

## Build

```bash
cmake -B build
cmake --build build
```

## Run

This example only works if the world is called `empty`. Start an empty world with:

```bash
gz sim empty.sdf
```

Then run the create program to spawn entities into the world:

```bash
./build/entity_creation
```
