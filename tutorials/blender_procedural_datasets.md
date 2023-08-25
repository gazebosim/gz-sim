\page blender_procedural_datasets Generation of Procedural Datasets with Blender

Procedurally-generated datasets of 3D models allow us to create diverse
simulation environments with nearly unlimited variety. Such environments enable
developers to evaluate their algorithms under various conditions that their
robots might experience. Furthermore, it is a simple way of producing a large
quantity of synthetic training data for applications within machine learning.

## Overview

[Blender](https://blender.org) is a free and open-source 3D software suite with
several tools and features for creating models that can be utilized inside
robotics simulators like Gazebo. In addition to manually modelling, sculpting
and texturing models, Blender also enables procedural generation of datasets
that can facilitate diversity in simulation environments. Furthermore, its
Python API allows us to generate such datasets programmatically.

### Procedural Geometry

[Geometry Nodes](https://docs.blender.org/manual/en/latest/modeling/geometry_nodes/introduction.html)
is a system of Blender for creating and modifying geometry through node-based
operations. It enables the synthesis of many model variations by combining
procedural operations with randomization. Although it enables a relatively
simple and fast method for creating datasets, it can be tricky to get started.
Fortunately, there are several helpful resources that can help you with the
creation of your own custom 3D models, e.g. an [overview video](https://www.youtube.com/watch?v=kMDB7c0ZiKA),
[demo files](https://blender.org/download/demo-files/#geometry-nodes),
[training from Blender Studio](https://studio.blender.org/training/geometry-nodes-from-scratch),
[tutorial by Blender Guru](https://www.youtube.com/watch?v=aO0eUnu0hO0) and many
others.

<!-- ### Procedural Materials and Textures -->

### Python Script for Generating Procedural Datasets of SDF models

Python script [`procedural_dataset_generator.py`](https://github.com/gazebosim/gz-sim/tree/main/examples/scripts/blender/procedural_dataset_generator.py)
contains a Blender exporter of SDF models and a generator of procedural SDF
dataset. The exporter outputs models that are compatible with the SDF format
and the directory structure of [Fuel](https://app.gazebosim.org), which can be
used with any existing `.blend` project. The dataset generator leverages
Blender's Geometry Nodes modifiers that are configured to generate countless
model variations via the Random Seed input attribute (seed for the pseudorandom
generation).

This script processes all selected objects in the opened Blender scene (or all
existing objects if none is selected). The exporter is configurable via CLI
arguments (or by adjusting default model-specific parameters). Currently, the
following features are supported:

- Export of mesh geometry as COLLADA (`.dae`), Wavefront (`.obj`) or STL
  (`.stl`) file formats.
- Option to exclude specific Blender objects from being exported to visual or
  collision geometry.
- Export separate visual and collision geometry at different levels of
  resolution to improve collision-checking performance with low-poly geometry.
- Estimation of inertial properties while assuming a uniform density.
- Optional randomization of model properties via uniform distribution, e.g. mass
  and surface friction.
- Linking of PBR textures sets via symbolic links or copies.
- Generation of thumbnails for the exported models.

## Instructions

### Dependencies

Blender is required to open `.blend` projects and run the script that employs
the Python API of Blender (`bpy`). Versions `[>=3.0, <3.3]` were tested with the
script, but newer versions are also expected to work.

- [Blender `[>=3.0]`](https://blender.org/download)

### Run the Python Script

#### Option A — Run Script Inside Blender

The first option is to run the Python script directly in Blender's *Text Editor*
tab for scripting while using the default parameters that are configurable via
constants at the beginning of the script. For new projects, you can follow these
steps:

1. Open the desired Blender `.blend` project. It is recommended to open Blender
   using a console to get full access to the standard output and error.

```bash
blender [blender options] file.blend
```

2. Copy the [`procedural_dataset_generator.py`](https://github.com/gazebosim/gz-sim/tree/main/examples/scripts/blender/procedural_dataset_generator.py)
   Python script into a new text data block in your `.blend` under the
   *Text Editor* tab.
3. Configure the default parameters of the script for your specific models via
   constants at the beginning of the file.
4. Run the script using the *Run script* button in the panel of the
   *Text Editor* tab at the top of the screen.

![Instructions in Blender](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/blender_instructions.png)

Once you follow these steps and configure the script for your `.blend` project,
you can save it and use Option B in the future.

#### Option B — Run an Internal Script

The second option is to run a script that is already saved inside a `.blend`
project, i.e. saved and configured through the procedure above. Therefore, this
approach is applicable only for `.blend` projects previously configured with the
[`procedural_dataset_generator.py`](https://github.com/gazebosim/gz-sim/tree/main/examples/scripts/blender/procedural_dataset_generator.py)
Python script. This option enables configuring the script via CLI arguments
instead of modifying its contents, which is unnecessary if the project is
already configured.

```bash
# Note: `procedural_dataset_generator.py` is the name of the Python script inside the `file.blend`
blender [blender options] file.blend --python-text procedural_dataset_generator.py -- [script options]
```

#### Option C — Run an External Script

It is also possible to run the Python script externally without configuring it
first inside a `.blend` project. This option also enables configuring the script
via CLI arguments, which is often required for new models.

```bash
# Note: `/path/to/procedural_dataset_generator.py` can be an absolute or relative system path
blender [blender options] file.blend --python /path/to/procedural_dataset_generator.py -- [script options]
```

## Examples

In order to demonstrate the generation of procedural datasets, the following two
`.blend` projects are provided:

- [rock.blend](https://github.com/gazebosim/gz-sim/tree/main/tutorials/files/blender_procedural_datasets/rock.blend)
  — Models of randomized rocks for Gazebo that robots can interact with
- [woodland.blend](https://github.com/gazebosim/gz-sim/tree/main/tutorials/files/blender_procedural_datasets/woodland.blend)
  — Static environments of natural scenery with randomly scattered assets of
  low-poly trees, rocks, grass and flowers (these assets were adapted from
  [Blender Studio](https://studio.blender.org))

You need to download these `.blend` projects to follow the examples. You can do
that either manually on GitHub or via `wget`/`curl`.

```bash
wget https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/rock.blend
wget https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/woodland.blend
```

### Try Demo Files

Now you can open these projects with Blender and run the scripts. Alternatively,
you could run the pre-configured Python scripts for each `.blend` project
directly from the console and configure CLI arguments as desired.

```bash
blender rock.blend --python-text procedural_dataset_generator.py -- -o sdf_models/rock
blender woodland.blend --python-text procedural_dataset_generator.py -- -o sdf_models/woodland
```

![Example of generating a dataset of rock SDF models](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/demo_blender_rock.gif)

![Example of generating a dataset of woodland SDF models](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/demo_blender_woodland.gif)

You can configure the script in several ways (see
`blender rock.blend --python-text procedural_dataset_generator.py -- -h`). For
example, you can set a combination of `--texture-source-mode` and
`--texture-source-value` arguments to link models to (random) PBR textures.

```bash
## Configure `--texture-source-value` with a path to one of the following directories
# ├── ./                                    # <-- Here to randomly sample a texture set
#   ├── texture0/                           # <-- Here to use texture0
#     ├── *albedo*.png || *color*.png
#     ├── *normal*.png
#     ├── *roughness*.png
#     └── *specular*.png || *metalness*.png
#   ├── texture1/
#   └── ...
blender rock.blend --python-text procedural_dataset_generator.py -- --texture-source-mode path --texture-source-value /path/to/textures
```

### Expected Output

Once the models are generated, you can see the generated directory structure for
each model.

```bash
./sdf_models/
├── rock/            # Parent directory of the generated dataset for rock models
  ├── rock0/         # SDF Model directory of the 1st model
  │ ├── meshes/      # Directory for mesh geometry
  │ │ ├── collision/ # Collision geometry, defaults to `.stl`
  │ │ └── visual/    # Visual geometry, defaults to `.obj`+`.mtl`
  │ ├── thumbnails/  # Directory for the generated thumbnail
  │ ├── model.config # Meta data about the model
  │ └── model.sdf    # SDF description of the model
  ├── rock1/         # SDF Model directory of the 2nd model
  └── ...
└── woodland/          # Identical structure for the dataset of woodland models ...
  ├── woodland0/
  ├── woodland1/
  └── ...
```

### `GZ_SIM_RESOURCE_PATH` Environment Variable

In order to make the models discoverable by Gazebo, you need to set the
`GZ_SIM_RESOURCE_PATH` environment variable such that it includes the path to
the parent directory of the models.

Note that the Python script will always print the adequate command with the
exact path after the export of models is finished.

```bash
export GZ_SIM_RESOURCE_PATH="${PWD}/sdf_models/rock${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_RESOURCE_PATH="${PWD}/sdf_models/woodland${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
```

### Spawn Models in Gazebo

Hereafter, you can spawn the generated models inside Gazebo with your preferred
approach, e.g. via the Resource Spawner GUI plugin. Below are some examples of
Gazebo environments using the rock and woodland SDF models.

![Example of the generated rock SDF models in Gazebo](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/demo_gazebo_rock.png)

![Example of the generated woodland SDF models in Gazebo](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/blender_procedural_datasets/demo_gazebo_woodland.png)

Every object that uses Geometry Nodes in these projects has several input
attributes that can be configured. You can open the `.blend` projects again,
navigate to the `Modifier Properties` tab for each object, and try tweaking them
yourself.

## Known Issues

### Installation of Modules in Blender's Python Environment (`trimesh`)

The [trimesh](https://trimsh.org) Python module is used to estimate inertial
properties. This module is not included by default with the Python environment
of Blender and thus needs to be installed. It is installed automatically during
the first run of the Python script. However, installing a new Python module
within Blender will not be possible if you use it from
[Snap Store](https://snapcraft.io/blender). If this is the case, you must use
the `--python-use-system-env` Blender option when running the Python script.
Furthermore, you might need to run the script as `sudo` for the first time if
the installation fails due to a lack of permission.

## Future Work

The Python script is not fully featured yet. Below is a list of features that
could be eventually added to improve its functionality:

- Add support for exporting processed objects as separate `<visual>` and
  `<collision>` entities of a single SDF model.
- Add support for baking textures for all processed models. Separate `<visual>`
  entities would be required to prevent overlapping of UV maps.
- Add support for pulling PBR textures from an online source, e.g. repository.
- Add support for exporting lights as SDF entities.
- Add support for exporting cameras as SDF entities.
