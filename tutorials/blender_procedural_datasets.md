\page blender_procedural_datasets Generation of Procedural Datasets with Blender

[url_blender_org]: https://blender.org
[Blender][url_blender_org] is a free and open source 3D software suite that
includes several tools and features for creation of models that can be utilized
also inside robotics simulators like Ignition Gazebo. In addition to modeling,
sculpting and texturing models manually, Blender similarly enables procedural
generation of datasets that can facilitate diversity in simulation environments.

[procedural_dataset_generator_script]: ../examples/scripts/blender/procedural_dataset_generator.py
Script [`procedural_dataset_generator.py`][procedural_dataset_generator_script]
supports generation of procedural SDF models for use inside Ignition Gazebo. Its
exporting capabilities can be used with a number of existing `.blend` files.
However, the Blender project must use node-based modifiers to enable generation
of procedural datasets (examples shown below). This script has currently the
following features:

1. Generation of SDF description for models compatible with Ignition Fuel

2. Exporting of visual and collision mesh geometry (at different levels of
detail)

3. Linking to existing PBR textures via symlink or hard copy

4. Estimation of inertial properties assuming uniform density.

5. Randomization of parameters, e.g. mass and surface friction

This script is still not yet fully featured, e.g. baking of procedural textures
is not yet implemented, hence it contains a number of TODO notes (contributions
are welcome).

## Procedural Mesh Geometry

[url_blender_docs_geometry_nodes]: https://docs.blender.org/manual/en/latest/modeling/geometry_nodes/introduction.html
[rock_blend_file]: ./files/blender_procedural_generation/rock.blend
[Geometry Nodes][url_blender_docs_geometry_nodes] is a feature of Blender that
enables creation and modification of mesh geometry through node-based pipelines.
By combining procedural operations with randomness, a seemingly infinite number
of model variations can be achieved by changing a seed of the pseudo-random
generator. For example, below is a showcase of randomly generated geometry for
rocks. The corresponding `.blend` file can be accessed here (tested with Blender
3.0/3.1): [rock.blend][rock_blend_file]. This file can be used as a template for
creation of other custom datasets.

@image html files/blender_procedural_generation/example_rock.gif "Procedural Generation Example" width=100%

[url_blender_tutorial_geometry_nodes_0]: https://youtube.com/watch?v=4WAxMI1QJMQ&list=PLjEaoINr3zgFX8ZsChQVQsuDSjEqdWMAD&index=9
[url_blender_tutorial_geometry_nodes_1]: https://youtube.com/watch?v=XSkaM-8Vgz8
Although Geometry Nodes enable relatively simple and fast creation of datasets,
it can be tricky to get started. Therefore, consider following one of the online
tutorials if you are interested in creating your own datasets. As an example,
[Geometry Nodes Tutorial (Donut part 9)][url_blender_tutorial_geometry_nodes_0]
provides an introduction to Geometry Nodes (consider watching the entire series
to get familiar with Blender). Similarly, more complex tutorials are available
such as [Plant Growth with Fields][url_blender_tutorial_geometry_nodes_1].

<!-- ## Procedural Textures and Materials -->
<!-- TODO[feature]: Document procedural materials once implemented -->

## Estimation of Inertial Properties

[url_trimesh_org]: https://trimsh.org
Script [`procedural_dataset_generator.py`][procedural_dataset_generator_script]
also supports the automatic estimation of inertial properties by the use of
[trimesh][url_trimesh_org] Python module. During the first execution of the
script, this module is automatically installed inside the Python environment of
Blender. Note, that this requires permissions to write into Blender installation
directory. Mass, inertia and centre of mass are estimated for each exported
object while assuming uniform density and configured density (or target mass).

## Known Issues

[url_snap_blender]: https://snapcraft.io/blender
If Blender is installed from [snap][url_snap_blender], it will not be possible
to setup the trimesh module required for estimation of inertial properties.
Therefore, please run Blender from console with the `--python-use-system-env`
flag.
