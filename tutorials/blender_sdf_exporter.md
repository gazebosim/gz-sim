\page blender_sdf_exporter Blender SDF Exporter

Blender is a Digital Content Creation (DCC) tool for working with 3d models.
In some cases you may be using it to bake lighting and environment maps.

The Blender SDF exporter is a blender script in which you can run within Blender to
export your meshes, their associated textures and lights to a dae file, its
corresponding SDF file and config file.

Please note that the SDF format does not have 1 to 1 parity of features with Blender's
mesh/materials/lights feature set. As such feel free to customize the script as needed.

## Using the Blender SDF Exporter

1. Download the blender script in [sdf_exporter.py](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/scripts/blender/sdf_exporter.py).

2. Open the script under Blender's Scripting tab and run it.

3. You will see a file dialog requesting a location to save the files to. Hit 'Save' when you are done.

4. The files `meshes/model.dae`, `model.config` and `model.sdf` will be created at the location you specified.
