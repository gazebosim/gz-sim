\page collada_world_exporter Collada World Exporter

The Collada world exporter is a world plugin that automatically generates
a Collada mesh consisting of the models within a world SDF file. This plugin
can be useful if you'd like to share an SDF world without requiring an SDF
loader.

## Using the Collada World Exporter

1. Add the following lines as a child to the `<world>` tag in an SDF file.
```
<plugin
  filename="gz-sim-collada-world-exporter-system"
  name="gz::sim::systems::ColladaWorldExporter">
</plugin>
```

2. Run the world using
```
gz sim -v 4 -s -r --iterations 1 WORLD_FILE_NAME
```

3. A subdirectory, named after the world, has been created in the current working directory. Within this subdirectory is the mesh and materials for the world.

Refer to the [collada_world_exporter.sdf](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/collada_world_exporter.sdf) example.
