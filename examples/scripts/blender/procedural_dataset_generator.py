#!/usr/bin/env -S blender --python
"""
This script contains a Blender exporter of SDF models `sdf_model_exporter` for Gazebo
and a generator of procedural SDF datasets `procedural_dataset_generator`.

The SDF exporter outputs models that are compatible with Fuel. It processes all selected
objects in the Blender scene (or all existing objects if none is selected). The exporter
is fully configurable via CLI arguments. It can export separate visual and collision
geometry at different resolution levels. It supports automatic estimation of inertial
properties for non-static models by `trimesh` Python module. Several attributes such as
model mass and surface friction can be randomized via uniform distribution by specifying
a range of minimum and maximum values.

The dataset generator is based on Blender's `Geometry Nodes` modifiers with support
for variations via `random_seed` input attribute (seed for the pseudorandom generation).
This module-based class inherits all functionalities of the SDF model exporter.

You can run this script in multiple ways:
a) Directly in Blender's [Text Editor] tab | with the default parameters (configurable)
  1. Copy this script into a [New Text] data block in your 'file.blend'
  2. Configure the default script parameters for your model via constants below
  3. Run the script using the [Run script] button at the top of the [Text Editor] tab
b) Running an internal script (saved within 'file.blend') | with CLI args (configurable)
  $ blender [blender options] file.blend --python-text script.py -- [script options]
c) Running an external script | with CLI args (configurable)
  $ blender [blender options] file.blend --python external_script.py -- [script options]

Show the help message of this script ([script options]):
  $ blender -b file.blend --python-text script.py -- -h
  $ blender -b file.blend --python external_script.py -- -h

Show the help message of Blender ([blender options]):
  $ blender -h
"""
from __future__ import annotations

# Raise an informative error in case the script is run outside Python env of Blender
try:
    import bpy
except ImportError as err:
    raise ImportError(
        "Python module of Blender 'bpy' not found! Please, execute this script inside "
        "a Blender environment (e.g. run inside the Scripting tab of Blender)."
    ) from err

import argparse
import enum
import os
import random
import shutil
import sys
from os import path
from types import ModuleType
from typing import Any, Dict, Iterable, List, Optional, TextIO, Tuple, Union
from xml.dom import minidom
from xml.etree import ElementTree

# Last tested and working version of Blender for this script (MAJOR, MINOR)
LAST_WORKING_VERSION: Tuple(int, int) = (3, 2)

### Default script parameters (adjustable via CLI arguments)
## Parameters for SDF model exporter `sdf_model_exporter`
OUTPUT_DIR: str = path.join(os.getcwd(), "sdf_models")
MODEL_VERSION: Optional[int] = None
## Parameters for procedural dataset generator `procedural_dataset_generator`
FIRST_SEED: int = 0
NUMBER_OF_VARIANTS: int = 8

### Default model-specific parameters (adjustable via CLI arguments)
## Default level of detail for exported geometry, e.g. subdivision level
DETAIL_LEVEL_VISUAL: int = 1
DETAIL_LEVEL_COLLISION: int = 0
## Default objects to ignore while exporting visual/collision geometry, even if selected
IGNORE_OBJECTS_VISUAL: List[str] = []
IGNORE_OBJECTS_COLLISION: List[str] = []
## Default source of textures for the model
# Options: "collada" == "dae" | "wavefront" == "obj" | "stl"
FILETYPE_VISUAL: str = "wavefront"
FILETYPE_COLLISION: str = "stl"
## Default source of textures for the model
# If true, symbolic links will be created for all textures instead of copies
SYMLINK_EXTERNAL_TEXTURES: bool = True
# Options: "none" | "path"
TEXTURE_SOURCE_MODE: str = "none"
TEXTURE_SOURCE_VALUE: Optional[str] = None
MATERIAL_TEXTURE_DIFFUSE: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
MATERIAL_TEXTURE_SPECULAR: Optional[Tuple[float, float, float]] = (0.2, 0.2, 0.2)
## Default inertial and dynamic properties of exported models
# If true, the model is immovable and it won't be updated by physics engine
STATIC: bool = False
# Options: "none" | "density" | "random_density" | "mass" | "random_mass"
INERTIAL_ESTIMATION_MODE: str = "none"
INERTIAL_ESTIMATION_VALUE: Optional[List[float]] = None
# (Random) coefficient of the surface friction (equal in both directions)
FRICTION_COEFFICIENT: List[float] = [1.0]

### Default keyword arguments for additional parameters (overwritten by CLI arguments)
DEFAULT_KWARGS: Any = {}


class sdf_model_exporter(ModuleType):
    """
    Blender exporter of objects as SDF models.

    Note: This class is designed as a "module class" to enable simple copy-pasting
    among Blender projects and environments, i.e. methods do not require an instance
    during their call while still supporting inheritance.
    """

    ## Directory structure compatible with Gazebo Fuel
    MODEL_ROOT: str = ""
    BASENAME_SDF: str = path.join(MODEL_ROOT, "model.sdf")
    BASENAME_SDF_MODEL_CONFIG: str = path.join(MODEL_ROOT, "model.config")
    DIRNAME_MESHES: str = path.join(MODEL_ROOT, "meshes")
    DIRNAME_MESHES_VISUAL: str = path.join(DIRNAME_MESHES, "visual")
    DIRNAME_MESHES_COLLISION: str = path.join(DIRNAME_MESHES, "collision")
    DIRNAME_MATERIALS: str = path.join(MODEL_ROOT, "materials")
    DIRNAME_TEXTURES: str = path.join(DIRNAME_MATERIALS, "textures")
    DIRNAME_THUMBNAILS: str = path.join(MODEL_ROOT, "thumbnails")

    # Targeted version of SDF
    SDF_VERSION: str = "1.9"

    @enum.unique
    class ExportFormat(enum.Enum):
        """
        Helper enum of supported mesh file formats coupled with their configured
        exporters.
        """

        COLLADA = enum.auto()
        STL = enum.auto()
        WAVEFRONT = enum.auto()

        def export(self, filepath: str) -> str:
            """
            Use Blender exporter of the corresponding mesh format. The desired
            `filepath` might be adjusted to include the appropriate file extension. Upon
            success, the absolute filepath of the exported mesh is returned. Note, that
            only the selected object(s) will be exported.
            """

            # Make sure the file has the correct extension
            if not filepath.endswith(self.extension):
                # Truncate incorrect extension(s) if there are any
                split_path = path.basename(filepath).split(".")
                if len(split_path) > 1:
                    filepath = path.join(path.dirname(filepath), split_path[0])
                # Append the correct extension
                filepath += self.extension
            filepath = path.abspath(filepath)
            os.makedirs(name=path.dirname(filepath), exist_ok=True)

            # Export with Blender based on the file format
            if self.COLLADA == self:
                bpy.ops.wm.collada_export(
                    filepath=filepath,
                    check_existing=False,
                    filter_blender=False,
                    filter_backup=False,
                    filter_image=False,
                    filter_movie=False,
                    filter_python=False,
                    filter_font=False,
                    filter_sound=False,
                    filter_text=False,
                    filter_archive=False,
                    filter_btx=False,
                    filter_collada=True,
                    filter_alembic=False,
                    filter_usd=False,
                    filter_obj=False,
                    filter_volume=False,
                    filter_folder=True,
                    filter_blenlib=False,
                    filemode=8,
                    display_type="DEFAULT",
                    sort_method="DEFAULT",
                    prop_bc_export_ui_section="main",
                    apply_modifiers=True,
                    export_mesh_type=0,
                    export_mesh_type_selection="view",
                    export_global_forward_selection="Y",
                    export_global_up_selection="Z",
                    apply_global_orientation=False,
                    selected=True,
                    include_children=False,
                    include_armatures=False,
                    include_shapekeys=False,
                    deform_bones_only=False,
                    include_animations=False,
                    include_all_actions=False,
                    export_animation_type_selection="sample",
                    sampling_rate=1,
                    keep_smooth_curves=False,
                    keep_keyframes=False,
                    keep_flat_curves=False,
                    active_uv_only=False,
                    use_texture_copies=True,
                    triangulate=True,
                    use_object_instantiation=True,
                    use_blender_profile=True,
                    sort_by_name=False,
                    export_object_transformation_type=0,
                    export_object_transformation_type_selection="matrix",
                    export_animation_transformation_type=0,
                    export_animation_transformation_type_selection="matrix",
                    open_sim=False,
                    limit_precision=False,
                    keep_bind_info=False,
                )
            elif self.STL == self:
                bpy.ops.export_mesh.stl(
                    filepath=filepath,
                    check_existing=False,
                    use_selection=True,
                    axis_forward="Y",
                    axis_up="Z",
                    global_scale=1,
                    use_scene_unit=False,
                    ascii=False,
                    use_mesh_modifiers=True,
                )
            elif self.WAVEFRONT == self:
                bpy.ops.export_scene.obj(
                    filepath=filepath,
                    check_existing=False,
                    axis_forward="Y",
                    axis_up="Z",
                    use_selection=True,
                    use_animation=False,
                    use_mesh_modifiers=True,
                    use_edges=True,
                    use_smooth_groups=False,
                    use_smooth_groups_bitflags=False,
                    use_normals=True,
                    use_uvs=True,
                    use_materials=True,
                    use_triangles=True,
                    use_nurbs=False,
                    use_vertex_groups=False,
                    use_blen_objects=True,
                    group_by_object=False,
                    group_by_material=False,
                    keep_vertex_order=False,
                    global_scale=1,
                    path_mode="AUTO",
                )
            else:
                raise ValueError(f"Filetype '{self}' is not supported for export.")

            return filepath

        @classmethod
        def from_str(cls, filetype_str: str) -> sdf_model_exporter.ExportFormat:
            """
            Return ExportFormat that corresponds with a matched `filetype_str` string.
            """

            filetype_str = filetype_str.strip().upper()
            if "COLLADA" in filetype_str or "DAE" in filetype_str:
                return cls.COLLADA
            elif "STL" in filetype_str:
                return cls.STL
            elif "WAVEFRONT" in filetype_str or "OBJ" in filetype_str:
                return cls.WAVEFRONT
            else:
                raise ValueError(f"Unknown '{filetype_str}' filetype.")

        @classmethod
        def default_visual(cls) -> sdf_model_exporter.ExportFormat:
            """
            Return the default filetype for visual mesh geometry.
            """

            return cls.from_str(FILETYPE_VISUAL)

        @classmethod
        def default_collision(cls) -> sdf_model_exporter.ExportFormat:
            """
            Return the default filetype for collision mesh geometry.
            """

            return cls.from_str(FILETYPE_COLLISION)

        @property
        def extension(self) -> str:
            """
            Return file extension of the corresponding filetype.
            """

            if self.COLLADA == self:
                return ".dae"
            elif self.STL == self:
                return ".stl"
            elif self.WAVEFRONT == self:
                return ".obj"
            else:
                raise ValueError(f"Unknown extension for '{self}' filetype.")

    @enum.unique
    class TextureSource(enum.Enum):
        """
        Helper enum of possible sources for textures. Each source should provide a path
        to searchable directory with the following structure (each texture type is
        optional and image format must be supported by Gazebo):
        ├── ./
            ├── texture_0/
                ├── *albedo*.png || *color*.png
                ├── *normal*.png
                ├── *roughness*.png
                └── *specular*.png || *metalness*.png
            ├── ...
            └── texture_n/
        Alternatively, it can point to a directory that directly contains textures and
        no other subdirectories.
        """

        NONE = enum.auto()
        BLENDER_MODEL = enum.auto()
        FILEPATH = enum.auto()
        ONLINE = enum.auto()

        def get_path(self, value: Optional[str] = None) -> Optional[str]:
            """
            Return a path to a directory with PBR textures.
            """

            if self.NONE == self:
                return None
            elif self.BLENDER_MODEL == self:
                return path.abspath(self._generate_baked_textures())
            elif self.FILEPATH == self:
                if not value or not path.isdir(value):
                    raise ValueError(
                        f"Value '{value}' is an invalid path to a directory with "
                        "textures."
                    )
                return path.abspath(value)
            elif self.ONLINE == self:
                return path.abspath(self._pull_online_textures(value))
            else:
                raise ValueError(f"Texture source '{self}' is not supported.")

        def is_enabled(self) -> bool:
            """
            Returns true if textures are enabled.
            """

            return not self.NONE == self

        @classmethod
        def from_str(cls, source_str: str) -> sdf_model_exporter.TextureSource:
            """
            Return TextureSource that corresponds with a matched `source_str` string.
            """

            source_str = source_str.strip().upper()
            if not source_str or "NONE" in source_str:
                return cls.NONE
            elif "BLENDER" in source_str:
                return cls.BLENDER_MODEL
            elif "PATH" in source_str:
                return cls.FILEPATH
            elif "ONLINE" in source_str:
                return cls.ONLINE
            else:
                raise ValueError(f"Unknown '{source_str}' texture source.")

        @classmethod
        def default(cls) -> sdf_model_exporter.TextureSource:
            """
            Return the default texture source.
            """

            return cls.from_str(TEXTURE_SOURCE_MODE)

        def _generate_baked_textures(
            cls,
        ) -> str:
            """
            Bake PBR textures for the model and return a path to a directory that
            contains them.
            """

            # TODO[feature]: Implement baking of textures from Blender materials
            return NotImplementedError("Baking of textures is not yet implemented!")

        def _pull_online_textures(
            cls,
            value: Optional[Any] = None,
        ) -> str:
            """
            Get PBR textures from an online source.
            """

            # TODO[feature]: Add option for pulling PBR textures from an online source
            return NotImplementedError(
                "Getting textures from an online source is not yet implemented!"
            )

    @enum.unique
    class InertialEstimator(enum.Enum):
        """
        Helper enum for estimating inertial properties of a model.
        """

        ## Modes of estimation
        NONE = enum.auto()
        DENSITY = enum.auto()
        RANDOM_DENSITY = enum.auto()
        MASS = enum.auto()
        RANDOM_MASS = enum.auto()

        def estimate_inertial_properties(
            self,
            analysed_mesh_filepath: str,
            value: List[float],
        ) -> Dict[
            str,
            Union[
                float,
                Tuple[float, float, float],
                Tuple[
                    Tuple[float, float, float],
                    Tuple[float, float, float],
                    Tuple[float, float, float],
                ],
            ],
        ]:
            """
            Estimate inertial properties of the mesh assuming uniform density.
            """

            # Try to import trimesh or try to install it within the Python environment
            # Note: This step might throw an exception if it is not possible
            # TODO[enhancement]: Add other methods for estimating inertial properties
            self._try_install_trimesh()
            import trimesh

            # Load the mesh
            mesh = trimesh.load(
                analysed_mesh_filepath, force="mesh", ignore_materials=True
            )

            # Extract or sample a floating point value from the input
            value = self.sample_value(value)

            # Set the density (either via density or mass)
            if self.DENSITY == self or self.RANDOM_DENSITY == self:
                mesh.density = value
            elif self.MASS == self or self.RANDOM_MASS == self:
                mesh.density = value / mesh.volume
            else:
                raise ValueError(
                    f"Mode for estimation of inertial properties '{self}'  is not "
                    "supported."
                )

            return {
                "mass": mesh.mass,
                "inertia": mesh.moment_inertia,
                "centre_of_mass": mesh.center_mass,
            }

        def sample_value(self, value: Optional[List[float]] = None) -> Optional[float]:
            """
            Extract or sample a value that should be used when estimating inertial
            properties.
            """

            if self.NONE == self:
                return None
            elif self.DENSITY == self or self.MASS == self:
                if not value or (isinstance(value, Iterable) and len(value) != 1):
                    raise ValueError(
                        "Exactly a single value must be provided when estimating "
                        "inertial properties with either target density of mass."
                    )
                return value[0] if isinstance(value, Iterable) else value
            elif self.RANDOM_DENSITY == self or self.RANDOM_MASS == self:
                if not value or (isinstance(value, Iterable) and len(value) != 2):
                    raise ValueError(
                        "A range with two values (MIN and MAX) must be provided when "
                        "sampling a random density of mass during estimation of "
                        "inertial properties."
                    )
                return random.uniform(value[0], value[1])
            else:
                raise ValueError(
                    f"Mode for estimation of inertial properties '{self}'  is not "
                    "supported."
                )

        def is_enabled(self) -> bool:
            """
            Returns true if estimation of inertial properties is enabled.
            """

            return not self.NONE == self

        @classmethod
        def from_str(cls, source_str: str) -> sdf_model_exporter.InertialEstimator:
            """
            Return InertialEstimator that corresponds with a matched `source_str`
            string.
            """

            source_str = source_str.strip().upper()
            if not source_str or "NONE" in source_str:
                return cls.NONE
            elif "RANDOM" in source_str:
                if "DENSITY" in source_str:
                    return cls.RANDOM_DENSITY
                elif "MASS" in source_str:
                    return cls.RANDOM_MASS
            elif "DENSITY" in source_str:
                return cls.DENSITY
            elif "MASS" in source_str:
                return cls.MASS
            else:
                raise ValueError(
                    f"Unknown '{source_str}' mode for estimation of inertial "
                    "properties."
                )

        @classmethod
        def default(cls) -> sdf_model_exporter.InertialEstimator:
            """
            Return the default mode for estimating inertial properties.
            """

            return cls.from_str(INERTIAL_ESTIMATION_MODE)

        @classmethod
        def _try_install_trimesh(cls):
            """
            Return the default mode for estimating inertial properties.
            """

            try:
                import trimesh
            except ImportError as err:
                import site
                import subprocess

                cls._print_bpy(
                    "Warn: Python module 'trimesh' could not found! This module is "
                    "necessary to estimate inertial properties of objects. Attempting "
                    "to install the module automatically via 'pip'...",
                    file=sys.stderr,
                )
                try:
                    subprocess.check_call([sys.executable, "-m", "ensurepip", "--user"])
                    subprocess.check_call(
                        [
                            sys.executable,
                            "-m",
                            "pip",
                            "install",
                            "--upgrade",
                            "trimesh",
                            "pycollada",
                            "--target",
                            str(site.getsitepackages()[0]),
                        ]
                    )
                    import trimesh
                except subprocess.CalledProcessError as err:
                    err_msg = (
                        "Python module 'trimesh' cannot be installed automatically! To "
                        "enable estimation of inertial properties, please install the "
                        "module manually (within Blender's Python environment) or use "
                        "the Python environment of your system (by passing Blender "
                        "option `--python-use-system-env`). Alternatively, you can "
                        "disable the estimation of inertial properties via script "
                        "argument `--inertial-estimation-mode none`."
                    )
                    cls._print_bpy(
                        f"Err: {err_msg}",
                        file=sys.stderr,
                    )
                    raise ImportError(err_msg) from err

    @classmethod
    def export(
        cls,
        output_dir: Optional[str] = OUTPUT_DIR,
        model_version: Optional[int] = MODEL_VERSION,
        model_name_prefix: str = "",
        model_name_suffix: str = "",
        filetype_visual: Union[ExportFormat, str] = ExportFormat.default_visual(),
        filetype_collision: Union[ExportFormat, str] = ExportFormat.default_collision(),
        detail_level_visual: int = DETAIL_LEVEL_VISUAL,
        detail_level_collision: int = DETAIL_LEVEL_COLLISION,
        ignore_objects_visual: List[str] = IGNORE_OBJECTS_VISUAL,
        ignore_objects_collision: List[str] = IGNORE_OBJECTS_COLLISION,
        symlink_external_textures: bool = SYMLINK_EXTERNAL_TEXTURES,
        texture_source_mode: Union[TextureSource, str] = TextureSource.default(),
        texture_source_value: Optional[str] = TEXTURE_SOURCE_VALUE,
        material_texture_diffuse: Optional[
            Tuple[float, float, float]
        ] = MATERIAL_TEXTURE_DIFFUSE,
        material_texture_specular: Optional[
            Tuple[float, float, float]
        ] = MATERIAL_TEXTURE_SPECULAR,
        static: bool = STATIC,
        inertial_estimation_mode: Union[
            InertialEstimator, str
        ] = InertialEstimator.default(),
        inertial_estimation_value: Optional[List[float]] = INERTIAL_ESTIMATION_VALUE,
        inertial_estimation_use_collision_mesh: bool = True,
        friction_coefficient: Optional[
            Union[float, Tuple[float, float]]
        ] = FRICTION_COEFFICIENT,
        generate_thumbnails: bool = True,
        **kwargs,
    ):
        """
        The primary function enables exporting of a single SDF model.
        """

        # Store the list of originally selected objects
        selected_objects_original = bpy.context.selected_objects
        # Create a list of objects to process
        if selected_objects_original:
            # Use currently selected objects if any is selected
            objects_to_process = selected_objects_original
        else:
            # Otherwise, use all objects if nothing is selected
            objects_to_process = bpy.context.selectable_objects

        # Separate different object types to process them differently
        # Note: Not all types are currently supported by this script
        # TODO[feature]: Support exporting of cameras and lights as SDF entities
        meshes_to_process = cameras_to_process = lights_to_process = []
        for obj in objects_to_process:
            if obj.type == "MESH":
                meshes_to_process.append(obj)
                cls._print_bpy(
                    f"Info: Processing mesh '{obj.name}'...",
                )
            # elif obj.type == "CAMERA":
            #     cameras_to_process.append(obj)
            #     cls._print_bpy(
            #         f"Info: Processing camera '{obj.name}'...",
            #     )
            # elif obj.type == "LIGHT":
            #     lights_to_process.append(obj)
            #     cls._print_bpy(
            #         f"Info: Processing light '{obj.name}'...",
            #     )
            else:
                cls._print_bpy(
                    f"Warn: Blender object '{obj.name}' with type '{obj.type}' will "
                    "not be processed.",
                    file=sys.stderr,
                )

        # Determine name of the exported model
        if len(meshes_to_process) == 0:
            raise EnvironmentError("Blender scene has no mesh models to export.")
        elif len(meshes_to_process) == 1:
            # Use object name as the model name if there is only one
            model_name = meshes_to_process[0].name
        else:
            # Use the project name as a joint model name if there are multiple meshes
            model_name = bpy.path.basename(bpy.data.filepath).split(".")[0]
        # Add prefix and suffix if desired
        model_name = f"{model_name_prefix}{model_name}{model_name_suffix}"

        # If output directory is not set (None), use the default Fuel model path
        if output_dir is None:
            output_dir = path.join(
                os.environ.get(
                    "GZ_FUEL_CACHE_PATH",
                    default=path.join(path.expanduser("~"), ".gazebo", "fuel"),
                ),
                "fuel.gazebosim.org",
                os.getlogin(),
                "models",
            )
            # Fuel requires model versioning, therefore, default to the next available
            # version if the model already exists (begins with version 1)
            if model_version is None:
                model_version = -1

        # Get the absolute path and create the directory if it does not already exist
        output_dir = path.abspath(output_dir)
        os.makedirs(name=output_dir, exist_ok=True)

        # Get a versioned model path (if desired)
        output_path = cls._try_get_versioned_model_path(
            output_dir=output_dir,
            model_name=model_name,
            model_version=model_version,
        )

        # Store all data necessary for SDF creation inside a dictionary
        sdf_data = {}

        # Process and export meshes
        exported_meshes = cls._process_meshes(
            meshes_to_process=meshes_to_process,
            output_path=output_path,
            model_name=model_name,
            filetype_visual=filetype_visual,
            filetype_collision=filetype_collision,
            detail_level_visual=detail_level_visual,
            detail_level_collision=detail_level_collision,
            ignore_objects_visual=ignore_objects_visual,
            ignore_objects_collision=ignore_objects_collision,
        )
        sdf_data.update(exported_meshes)

        # Sample textures
        texture_source = (
            texture_source_mode
            if isinstance(texture_source_mode, cls.TextureSource)
            else cls.TextureSource.from_str(texture_source_mode)
        )
        if texture_source.is_enabled():
            textures_dirpath = texture_source.get_path(texture_source_value)
            textures = cls._sample_textures(textures_dirpath=textures_dirpath)
            sdf_data.update(textures)

        # Estimate inertial properties (if enabled)
        if not static:
            inertial_estimator = (
                inertial_estimation_mode
                if isinstance(inertial_estimation_mode, cls.InertialEstimator)
                else cls.InertialEstimator.from_str(inertial_estimation_mode)
            )
            if inertial_estimator.is_enabled():
                analysis_mesh_filepath = path.join(
                    output_path,
                    exported_meshes["filepath_mesh_collision"]
                    if inertial_estimation_use_collision_mesh
                    else exported_meshes["filepath_mesh_visual"],
                )
                inertial_properties = inertial_estimator.estimate_inertial_properties(
                    analysed_mesh_filepath=analysis_mesh_filepath,
                    value=inertial_estimation_value,
                )
                sdf_data.update(inertial_properties)

        # Write data into an SDF file
        cls._generate_sdf_file(
            output_path=output_path,
            model_name=model_name,
            static=static,
            friction_coefficient=friction_coefficient,
            symlink_external_textures=symlink_external_textures,
            material_texture_diffuse=material_texture_diffuse,
            material_texture_specular=material_texture_specular,
            **sdf_data,
        )

        # Create a corresponding config file for the SDF model
        cls._generate_model_config_file(output_path=output_path, model_name=model_name)

        # Render few images to generate thumbnails
        if not bpy.app.background and generate_thumbnails:
            cls._generate_thumbnails(output_path=output_path)

        cls._print_bpy(
            f"Info: Model '{model_name}' exported to 'file://{output_path}'."
        )

        # Select the original objects again to keep the UI (almost) the same
        bpy.ops.object.select_all(action="DESELECT")
        # Select all desired meshes at the same time
        for obj in selected_objects_original:
            obj.select_set(True)

    def _print_bpy(msg: Any, file: Optional[TextIO] = sys.stdout, *args, **kwargs):
        """
        Helper print function that also provides output inside the Blender console,
        in addition to the system console.
        """

        print(msg, file=file, *args, **kwargs)
        for window in bpy.context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "CONSOLE":
                    with bpy.context.temp_override(
                        window=window, screen=window.screen, area=area
                    ):
                        bpy.ops.console.scrollback_append(
                            text=str(msg),
                            type="ERROR" if file == sys.stderr else "OUTPUT",
                        )

    @classmethod
    def _process_meshes(
        cls,
        meshes_to_process: List,
        output_path: str,
        model_name: str,
        filetype_visual: Union[ExportFormat, str],
        filetype_collision: Union[ExportFormat, str],
        detail_level_visual: int,
        detail_level_collision: int,
        ignore_objects_visual: List[str],
        ignore_objects_collision: List[str],
    ) -> Dict[str, str]:
        """
        Process and export meshes of the model.
        """

        # Convert to Enum if strings were passed
        if isinstance(filetype_visual, str):
            filetype_visual = cls.ExportFormat.from_str(filetype_visual)
        if isinstance(filetype_collision, str):
            filetype_collision = cls.ExportFormat.from_str(filetype_collision)

        # Keep only object names that need processing in the ignore list
        meshes_to_process_names = [mesh.name for mesh in meshes_to_process]
        ignore_objects_visual = [
            name for name in ignore_objects_visual if name in meshes_to_process_names
        ]
        ignore_objects_collision = [
            name for name in ignore_objects_collision if name in meshes_to_process_names
        ]

        # Deselect all objects
        bpy.ops.object.select_all(action="DESELECT")
        # Select all desired meshes at the same time
        for obj in meshes_to_process:
            obj.select_set(True)

        return cls._export_geometry(
            output_path=output_path,
            model_name=model_name,
            filetype_visual=filetype_visual,
            filetype_collision=filetype_collision,
            detail_level_visual=detail_level_visual,
            detail_level_collision=detail_level_collision,
            ignore_objects_visual=ignore_objects_visual,
            ignore_objects_collision=ignore_objects_collision,
        )

    @classmethod
    def _export_geometry(
        cls,
        output_path: str,
        model_name: str,
        filetype_visual: ExportFormat,
        filetype_collision: ExportFormat,
        detail_level_visual: int,
        detail_level_collision: int,
        ignore_objects_visual: List[str],
        ignore_objects_collision: List[str],
    ) -> Dict[str, str]:
        """
        Export both visual and collision mesh geometry.
        """

        filepath_collision = cls._export_geometry_collision(
            output_path=output_path,
            model_name=model_name,
            filetype=filetype_collision,
            detail_level=detail_level_collision,
            ignore_object_names=ignore_objects_collision,
        )
        filepath_visual = cls._export_geometry_visual(
            output_path=output_path,
            model_name=model_name,
            filetype=filetype_visual,
            detail_level=detail_level_visual,
            ignore_object_names=ignore_objects_visual,
        )

        return {
            "filepath_mesh_collision": path.relpath(
                path=filepath_collision, start=output_path
            ),
            "filepath_mesh_visual": path.relpath(
                path=filepath_visual, start=output_path
            ),
        }

    @classmethod
    def _export_geometry_collision(
        cls,
        output_path: str,
        model_name: str,
        filetype: ExportFormat,
        detail_level: int,
        ignore_object_names: List[str],
    ) -> str:
        """
        Export collision geometry of the model with the specified `filetype`.
        Method `_pre_export_geometry_collision()` is called before the export.
        Method `_post_export_geometry_collision()` is called after the export.
        """

        # Hook call before export of collision geometry
        cls._pre_export_geometry_collision(
            detail_level=detail_level, ignore_object_names=ignore_object_names
        )

        resulting_output_path = filetype.export(
            path.join(output_path, cls.DIRNAME_MESHES_COLLISION, model_name)
        )

        # Hook call after export of collision geometry
        cls._post_export_geometry_collision(ignore_object_names=ignore_object_names)

        return resulting_output_path

    @classmethod
    def _export_geometry_visual(
        cls,
        output_path: str,
        model_name: str,
        filetype: ExportFormat,
        detail_level: int,
        ignore_object_names: List[str],
    ) -> str:
        """
        Export visual geometry of the model with the specified `filetype`.
        Method `_pre_export_geometry_visual()` is called before the export.
        Method `_post_export_geometry_visual()` is called after the export.
        """

        # Hook call before export of visual geometry
        cls._pre_export_geometry_visual(
            detail_level=detail_level,
            ignore_object_names=ignore_object_names,
        )

        resulting_output_path = filetype.export(
            path.join(output_path, cls.DIRNAME_MESHES_VISUAL, model_name)
        )

        # Hook call after export of visual geometry
        cls._post_export_geometry_collision(ignore_object_names=ignore_object_names)

        return resulting_output_path

    @classmethod
    def _sample_textures(
        cls, textures_dirpath: Optional[str]
    ) -> Dict[str, Optional[str]]:
        """
        Get PBR textures for the exported model from `textures_dirpath`. If the
        directory contains multiple texture sets, it is selected at random.
        """

        # Do not process if the texture directory is None
        if not textures_dirpath:
            return {}

        # Get the content of the texture directory
        textures = os.listdir(textures_dirpath)

        # Determine whether the directory contains multiple sets of textures
        choose_sample_random: bool = False
        for texture in textures:
            if path.isdir(path.join(textures_dirpath, texture)):
                choose_sample_random = True
                break

        if choose_sample_random:
            # Select a random set of PBR textures
            texture_dirpath = path.join(textures_dirpath, random.choice(textures))
        else:
            # The directory already points to a set of PBR textures
            texture_dirpath = textures_dirpath

        # Get the appropriate texture files
        texture_albedo: Optional[str] = None
        texture_roughness: Optional[str] = None
        texture_metalness: Optional[str] = None
        texture_normal: Optional[str] = None
        for texture in os.listdir(texture_dirpath):
            texture_cmp = cls._unify_string(texture)
            if not texture_albedo and (
                "color" in texture_cmp or "albedo" in texture_cmp
            ):
                texture_albedo = path.join(texture_dirpath, texture)
            elif not texture_roughness and "roughness" in texture_cmp:
                texture_roughness = path.join(texture_dirpath, texture)
            elif not texture_metalness and (
                "specular" in texture_cmp or "metalness" in texture_cmp
            ):
                texture_metalness = path.join(texture_dirpath, texture)
            elif not texture_normal and "normal" in texture_cmp:
                texture_normal = path.join(texture_dirpath, texture)
            else:
                cls._print_bpy(
                    f"Warn: File 'file://{path.join(texture_dirpath, texture)}' is "
                    "not a recognized texture type or there are multiple textures "
                    "of the same type inside 'file://{texture_dirpath}'.",
                    file=sys.stderr,
                )

        return {
            "texture_albedo": texture_albedo,
            "texture_roughness": texture_roughness,
            "texture_metalness": texture_metalness,
            "texture_normal": texture_normal,
        }

    @classmethod
    def _generate_sdf_file(
        cls,
        output_path: str,
        model_name: str,
        filepath_mesh_visual: str,
        filepath_mesh_collision: str,
        texture_albedo: Optional[str] = None,
        texture_roughness: Optional[str] = None,
        texture_metalness: Optional[str] = None,
        texture_normal: Optional[str] = None,
        material_texture_diffuse: Optional[
            Tuple[float, float, float]
        ] = MATERIAL_TEXTURE_DIFFUSE,
        material_texture_specular: Optional[
            Tuple[float, float, float]
        ] = MATERIAL_TEXTURE_SPECULAR,
        symlink_external_textures: bool = SYMLINK_EXTERNAL_TEXTURES,
        static: bool = STATIC,
        mass: Optional[float] = None,
        inertia: Optional[
            Tuple[
                Tuple[float, float, float],
                Tuple[float, float, float],
                Tuple[float, float, float],
            ]
        ] = None,
        centre_of_mass: Optional[Tuple[float, float, float]] = None,
        friction_coefficient: Optional[
            Union[float, Tuple[float, float]]
        ] = FRICTION_COEFFICIENT,
    ):
        """
        Generate SDF file from passed arguments and export to `output_path`.
        """

        # Initialize SDF with a single model and a link
        sdf = ElementTree.Element("sdf", attrib={"version": cls.SDF_VERSION})
        model = ElementTree.SubElement(sdf, "model", attrib={"name": model_name})
        statit_xml = ElementTree.SubElement(model, "static")
        statit_xml.text = str(static)
        link = ElementTree.SubElement(
            model, "link", attrib={"name": f"{model_name}_link"}
        )

        # Visual geometry
        visual = ElementTree.SubElement(
            link, "visual", attrib={"name": f"{model_name}_visual"}
        )
        visual_geometry = ElementTree.SubElement(visual, "geometry")
        visual_mesh = ElementTree.SubElement(visual_geometry, "mesh")
        visual_mesh_uri = ElementTree.SubElement(visual_mesh, "uri")
        visual_mesh_uri.text = filepath_mesh_visual
        # Material
        textures = (
            texture_albedo,
            texture_roughness,
            texture_metalness,
            texture_normal,
        )
        if any(texture for texture in textures):
            material = ElementTree.SubElement(visual, "material")
            pbr = ElementTree.SubElement(material, "pbr")
            metal = ElementTree.SubElement(pbr, "metal")

            texture_albedo, texture_roughness, texture_metalness, texture_normal = (
                cls._preprocess_texture_path(
                    texture,
                    output_path=output_path,
                    symlink_external_textures=symlink_external_textures,
                )
                for texture in textures
            )

            if texture_albedo:
                diffuse = ElementTree.SubElement(material, "diffuse")
                if material_texture_diffuse:
                    diffuse.text = " ".join(map(str, material_texture_diffuse))
                specular = ElementTree.SubElement(material, "specular")
                if material_texture_specular:
                    specular.text = " ".join(map(str, material_texture_specular))
                albedo_map = ElementTree.SubElement(metal, "albedo_map")
                albedo_map.text = texture_albedo
            if texture_roughness:
                roughness_map = ElementTree.SubElement(metal, "roughness_map")
                roughness_map.text = texture_roughness
            if texture_metalness:
                metalness_map = ElementTree.SubElement(metal, "metalness_map")
                metalness_map.text = texture_metalness
            if texture_normal:
                normal_map = ElementTree.SubElement(metal, "normal_map")
                normal_map.text = texture_normal

        # Collision geometry
        collision = ElementTree.SubElement(
            link, "collision", attrib={"name": f"{model_name}_collision"}
        )
        collision_geometry = ElementTree.SubElement(collision, "geometry")
        collision_mesh = ElementTree.SubElement(collision_geometry, "mesh")
        collision_mesh_uri = ElementTree.SubElement(collision_mesh, "uri")
        collision_mesh_uri.text = filepath_mesh_collision
        # Surface friction
        surface = ElementTree.SubElement(collision, "surface")
        surface_friction = ElementTree.SubElement(surface, "friction")
        ode_friction = ElementTree.SubElement(surface_friction, "ode")
        ode_friction_mu = ElementTree.SubElement(ode_friction, "mu")
        ode_friction_mu2 = ElementTree.SubElement(ode_friction, "mu2")
        if isinstance(friction_coefficient, Iterable):
            if len(friction_coefficient) == 2:
                # Randomize friction coefficient if a range is passed
                friction_coefficient = random.uniform(
                    friction_coefficient[0], friction_coefficient[1]
                )
            else:
                friction_coefficient = friction_coefficient[0]

        ode_friction_mu.text = ode_friction_mu2.text = str(friction_coefficient)
        bullet_friction = ElementTree.SubElement(surface_friction, "bullet")
        bullet_friction_coef = ElementTree.SubElement(bullet_friction, "friction")
        bullet_friction_coef2 = ElementTree.SubElement(bullet_friction, "friction2")
        bullet_friction_coef.text = bullet_friction_coef2.text = str(
            friction_coefficient
        )

        # Inertial
        if not static and (
            centre_of_mass is not None and mass is not None and inertia is not None
        ):
            inertial = ElementTree.SubElement(link, "inertial")
            pose = ElementTree.SubElement(inertial, "pose")
            pose.text = f"{' '.join(map(str, centre_of_mass))} 0.0 0.0 0.0"
            mass_xml = ElementTree.SubElement(inertial, "mass")
            mass_xml.text = str(mass)
            inertia_xml = ElementTree.SubElement(inertial, "inertia")
            inertia_xx = ElementTree.SubElement(inertia_xml, "ixx")
            inertia_xx.text = str(inertia[0][0])
            inertia_xy = ElementTree.SubElement(inertia_xml, "ixy")
            inertia_xy.text = str(inertia[0][1])
            inertia_xz = ElementTree.SubElement(inertia_xml, "ixz")
            inertia_xz.text = str(inertia[0][2])
            inertia_yy = ElementTree.SubElement(inertia_xml, "iyy")
            inertia_yy.text = str(inertia[1][1])
            inertia_yz = ElementTree.SubElement(inertia_xml, "iyz")
            inertia_yz.text = str(inertia[1][2])
            inertia_zz = ElementTree.SubElement(inertia_xml, "izz")
            inertia_zz.text = str(inertia[2][2])

        # Convert SDF to xml string and write to file
        sdf_xml_string = minidom.parseString(
            ElementTree.tostring(sdf, encoding="unicode")
        ).toprettyxml(indent="  ")
        sdf_file = open(path.join(output_path, cls.BASENAME_SDF), "w")
        sdf_file.write(sdf_xml_string)
        sdf_file.close()

    @classmethod
    def _generate_model_config_file(
        cls,
        output_path: str,
        model_name: str,
    ):
        """
        Generate model config for the SDF model.
        """

        # Initialize model config with its corresponding name
        model_config = ElementTree.Element("model")
        name = ElementTree.SubElement(model_config, "name")
        name.text = model_name

        # Version of the model (try to match version from the exported path)
        maybe_version = path.basename(output_path)
        if maybe_version.isnumeric():
            version = ElementTree.SubElement(model_config, "version")
            version.text = maybe_version

        # Path to SDF
        sdf_tag = ElementTree.SubElement(
            model_config, "sdf", attrib={"version": cls.SDF_VERSION}
        )
        sdf_tag.text = cls.BASENAME_SDF

        # Author name is based on ${USER}
        author = ElementTree.SubElement(model_config, "author")
        name = ElementTree.SubElement(author, "name")
        name.text = os.getlogin()
        # Keep track of Blender version that produced the model
        producer = ElementTree.SubElement(author, "producer")
        producer.text = f"Blender {bpy.app.version_string}"

        # Describe how the model was generated
        description = ElementTree.SubElement(model_config, "description")
        description.text = (
            f"Model generated from '{bpy.path.basename(bpy.data.filepath)}' by "
            f"'{path.relpath(path=__file__, start=bpy.data.filepath)}' Python script"
        )

        # Convert config to xml string and write to file
        model_config_xml_string = minidom.parseString(
            ElementTree.tostring(model_config, encoding="unicode")
        ).toprettyxml(indent="  ")
        model_config_file = open(
            path.join(output_path, cls.BASENAME_SDF_MODEL_CONFIG), "w"
        )
        model_config_file.write(model_config_xml_string)
        model_config_file.close()

    @classmethod
    def _generate_thumbnails(cls, output_path: str):
        """
        Render thumbnails for the SDF model.
        Currently, only a single viewport render is created using OpenGL.
        """

        # Create thumbnails directory for the model
        thumbnails_dir = path.join(output_path, cls.DIRNAME_THUMBNAILS)
        os.makedirs(name=thumbnails_dir, exist_ok=True)

        # Specify path for the thumbnail
        bpy.context.scene.render.filepath = path.join(thumbnails_dir, "0")

        # Set render parameters
        bpy.context.scene.render.resolution_x = 256
        bpy.context.scene.render.resolution_y = 256
        bpy.context.scene.render.pixel_aspect_x = 1.0
        bpy.context.scene.render.pixel_aspect_x = 1.0

        # Render image through viewport
        bpy.ops.render.opengl(write_still=True)

    @classmethod
    def _pre_export_geometry_collision(
        cls, detail_level: int, ignore_object_names: List[str] = []
    ):
        """
        A hook that is called before exporting collision geometry. Always chain up the
        parent implementation.
        By default, this hook handles reselecting objects from `ignore_object_names`.
        """

        cls._select_objects(names=ignore_object_names, type_filter="MESH", select=False)

    @classmethod
    def _post_export_geometry_collision(cls, ignore_object_names: List[str] = []):
        """
        A hook that is called after exporting collision geometry. Always chain up the
        parent implementation.
        By default, this hook handles deselecting objects from `ignore_object_names`.
        """

        cls._select_objects(names=ignore_object_names, type_filter="MESH", select=True)

    @classmethod
    def _pre_export_geometry_visual(
        cls,
        detail_level: int,
        ignore_object_names: List[str] = [],
    ):
        """
        A hook that is called before exporting visual geometry. Always chain up the
        parent implementation.
        By default, this hook handles reselecting objects from `ignore_object_names`.
        """

        cls._select_objects(names=ignore_object_names, type_filter="MESH", select=False)

    @classmethod
    def _post_export_geometry_visual(cls, ignore_object_names: List[str] = []):
        """
        A hook that is called after exporting visual geometry. Always chain up the
        parent implementation.
        By default, this hook handles deselecting objects from `ignore_object_names`.
        """

        cls._select_objects(names=ignore_object_names, type_filter="MESH", select=True)

    def _select_objects(
        names=List[str], type_filter: Optional[str] = None, select: bool = True
    ):
        """
        (De)select list of objects based on their `names`. List can be filtered
        according to the object type via `type_filter`.
        """

        for obj in bpy.context.selectable_objects:
            if type_filter and obj.type != type_filter:
                continue
            if obj.name in names:
                obj.select_set(select)

    def _unify_string(
        string: str,
        chars_to_remove: Union[str, Iterable[str]] = (
            " ",
            "_",
            "-",
            "(",
            ")",
            "[",
            "]",
            "{",
            "}",
        ),
        lowercase: bool = True,
    ) -> str:
        """
        Helper function unifies string for a more robust matching. The strings is
        changed to lower-case and `chars_to_remove` are removed.
        """

        string = string.strip()
        for char in chars_to_remove:
            string = string.replace(char, "")
        if lowercase:
            return string.lower()
        else:
            return string.upper()

    @classmethod
    def _preprocess_texture_path(
        cls,
        texture_original_filepath: Optional[str],
        output_path: str,
        symlink_external_textures: bool,
    ):
        """
        Preprocess filepath of a texture such that it is in the local model directory
        path. If `symlink_external_textures` is enabled, symbolic links will be
        created. No copy or symlink will be made if the `texture_original_filepath` is
        already a subpath of `output_path`.
        This can fail due to the lack of filesystem permissions.
        """

        # Skip processing of unset textures
        if not texture_original_filepath:
            return None

        # Make sure the texture is valid
        if not path.exists(texture_original_filepath):
            raise ValueError(
                f"Texture 'file://{texture_original_filepath}' does not exist!"
            )

        # Copy over the texture inside the model directory, or create a symbolic link
        # Only do this if the texture is not already under the export directory
        if texture_original_filepath.startswith(output_path):
            texture_target_filepath = texture_original_filepath
        else:
            texture_dir = path.join(output_path, cls.DIRNAME_TEXTURES)
            texture_target_filepath = path.join(
                texture_dir, path.basename(texture_original_filepath)
            )
            os.makedirs(name=texture_dir, exist_ok=True)
            if symlink_external_textures:
                try:
                    os.symlink(
                        src=texture_original_filepath, dst=texture_target_filepath
                    )
                except OSError as err:
                    import errno

                    if err.errno == errno.EEXIST:
                        os.remove(texture_target_filepath)
                        os.symlink(
                            src=texture_original_filepath, dst=texture_target_filepath
                        )
                    else:
                        raise err
            else:
                shutil.copy(src=texture_original_filepath, dst=texture_target_filepath)

        return path.relpath(path=texture_target_filepath, start=output_path)

    @classmethod
    def _try_get_versioned_model_path(
        cls, output_dir: str, model_name: str, model_version: Optional[int]
    ) -> str:
        """
        Return versioned model directory path if `model_version` is specified. For
        negative `model_version` and an existing model directory, the next version will
        be used to avoid overwriting.
        """

        unversioned_model_path = path.join(output_dir, model_name)

        if model_version is None:
            return unversioned_model_path
        elif model_version < 0:
            return path.join(
                unversioned_model_path,
                str(cls._get_next_model_version(model_path=unversioned_model_path)),
            )
        else:
            return path.join(unversioned_model_path, model_version)

    def _get_next_model_version(model_path: str) -> int:
        """
        Return the next version if model already exists. Otherwise, return '1' as the
        initial (first) version.
        """

        if path.exists(model_path):
            last_version = max(
                [
                    int(path.basename(subdir))
                    for subdir in os.scandir(model_path)
                    if subdir.is_dir() and path.basename(subdir).isnumeric()
                ]
            )
            return last_version + 1
        else:
            return 1


class procedural_dataset_generator(sdf_model_exporter):
    """
    Generator of procedural datasets using Blender's Geometry Nodes.
    """

    # The following lookup phrases are used to find the corresponding input attributes
    # of the node system (exact match, insensitive to case, insensitive to '_'/'-'/...)
    LOOKUP_PHRASES_RANDOM_SEED: List[str] = [
        "rng",
        "seed",
        "randomseed",
        "pseodorandomseed",
    ]
    LOOKUP_PHRASES_DETAIL_LEVEL: List[str] = [
        "detail",
        "detaillevel",
        "detailobject",
        "levelofdetail",
        "subdivisionlevel",
        "subdivlevel",
    ]

    @classmethod
    def generate(
        cls,
        first_seed: int = FIRST_SEED,
        number_of_variants: int = NUMBER_OF_VARIANTS,
        redraw_viewport_during_processing: bool = True,
        **kwargs,
    ):
        """
        Generate `number_of_variants` different models by changing the random seed in
        the Geometry Nodes system of the individual meshes.
        """

        cls._print_bpy(
            f"Info: Generating {number_of_variants} model variants in the seed range "
            f"[{first_seed}:{first_seed + number_of_variants}]."
        )

        # Export models for the entire range of random seeds
        global current_seed
        for seed in range(first_seed, first_seed + number_of_variants):
            current_seed = seed
            random.seed(seed)
            name_suffix = str(seed)
            if "model_name_suffix" in kwargs:
                name_suffix = f'{kwargs.pop("model_name_suffix")}{name_suffix}'
            cls.export(model_name_suffix=name_suffix, **kwargs)

            # Update the viewport to keep track of progress (only if GUI is enabled)
            # Performance might be lowered because the scene needs to be re-rendered
            if not bpy.app.background and redraw_viewport_during_processing:
                bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)

    @classmethod
    def _pre_export_geometry_collision(
        cls, detail_level: int, ignore_object_names: List[str] = []
    ):
        """
        A hook that is called before exporting collision geometry. This implementation
        adjusts input attributes of the Geometry Nodes system for each mesh.
        """

        # Call parent impl
        sdf_model_exporter._pre_export_geometry_collision(
            detail_level=detail_level,
            ignore_object_names=ignore_object_names,
        )

        global current_seed
        selected_meshes = bpy.context.selected_objects
        for obj in selected_meshes:
            for nodes_modifier in cls._get_all_nodes_modifiers(obj):
                cls._try_set_nodes_input_attribute(
                    obj,
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_RANDOM_SEED,
                    current_seed,
                    print_warning=True,
                )
                cls._try_set_nodes_input_attribute(
                    obj,
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_DETAIL_LEVEL,
                    detail_level,
                )
            cls._trigger_modifier_update(obj)

    @classmethod
    def _pre_export_geometry_visual(
        cls,
        detail_level: int,
        ignore_object_names: List[str] = [],
    ):
        """
        A hook that is called before exporting visual geometry. This implementation
        adjusts input attributes of the Geometry Nodes system for each mesh.
        """

        # Call parent impl
        sdf_model_exporter._pre_export_geometry_visual(
            detail_level=detail_level,
            ignore_object_names=ignore_object_names,
        )

        global current_seed
        selected_meshes = bpy.context.selected_objects
        for obj in selected_meshes:
            for nodes_modifier in cls._get_all_nodes_modifiers(obj):
                cls._try_set_nodes_input_attribute(
                    obj,
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_RANDOM_SEED,
                    current_seed,
                    print_warning=True,
                )
                cls._try_set_nodes_input_attribute(
                    obj,
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_DETAIL_LEVEL,
                    detail_level,
                )

            cls._trigger_modifier_update(obj)

    def _get_all_nodes_modifiers(obj: bpy.types.Object) -> List[bpy.types.Modifier]:
        """
        Return all node-based modifiers of an object.
        """

        return [modifier for modifier in obj.modifiers if modifier.type == "NODES"]

    @classmethod
    def _try_set_nodes_input_attribute(
        cls,
        obj: bpy.types.Object,
        modifier: bpy.types.NodesModifier,
        lookup_phrases: Iterable[str],
        value: Any,
        print_warning: bool = False,
    ):
        """
        Try to set an input attribute of a nodes system to a `value`. The attribute
        looked is performed by using `lookup_phrases`.
        """

        # Try to find the corresponding ID of the input attribute
        input_id: Optional[str] = None
        for attribute in modifier.node_group.inputs:
            if cls._unify_string(attribute.name) in lookup_phrases:
                input_id = attribute.identifier
                break
        if input_id is None:
            if print_warning:
                cls._print_bpy(
                    "Warn: Unable to find a matching input attribute of the object's "
                    f"'{obj.name}' modifier '{modifier.name}' for any of the requested "
                    f"lookup phrases {lookup_phrases}. You can ignore this warning if "
                    "the modifier is not supposed to have the requested input.",
                    file=sys.stderr,
                )
            return

        # Set the attribute
        modifier[input_id] = value

    def _trigger_modifier_update(obj: bpy.types.Object):
        """
        Trigger an update of object's modifiers after changing its attributes.
        """

        # Not sure how else to trigger update of the modifiers, but setting the index
        # of any modifier does the trick (even if the index stays the same)
        # TODO[enhancement]: Improve updating of modifiers after programmatic changes
        bpy.context.view_layer.objects.active = obj
        if len(obj.modifiers.values()):
            bpy.ops.object.modifier_move_to_index(
                modifier=obj.modifiers.values()[0].name,
                index=0,
            )


def main(**kwargs):

    # Warn the user in case an untested version of Blender is used
    if bpy.app.version[0] != LAST_WORKING_VERSION[0]:
        sdf_model_exporter._print_bpy(
            f"Err: Untested major version of Blender ({bpy.app.version_string})! "
            "This script will likely fail. Please, use Blender version "
            f"[~{LAST_WORKING_VERSION[0]}.{LAST_WORKING_VERSION[1]}].",
            file=sys.stderr,
        )
    elif bpy.app.version[1] < LAST_WORKING_VERSION[1]:
        sdf_model_exporter._print_bpy(
            f"Warn: Untested minor version of Blender ({bpy.app.version_string})! "
            "This script might not work as intended. Please, consider using Blender "
            f"version [~{LAST_WORKING_VERSION[0]}.{LAST_WORKING_VERSION[1]}].",
            file=sys.stderr,
        )

    # Update default keyword arguments with parsed arguments
    export_kwargs = DEFAULT_KWARGS
    export_kwargs.update(kwargs)

    if export_kwargs.pop("export_dataset"):
        # Generate a dataset of procedural models
        procedural_dataset_generator.generate(**export_kwargs)
        sdf_model_exporter._print_bpy(
            f'Info: A dataset with {export_kwargs["number_of_variants"]} models was '
            "exported."
        )
    else:
        # Export a single SDF model
        sdf_model_exporter.export(**export_kwargs)
        sdf_model_exporter._print_bpy("Info: A single SDF models was exported.")

    # Inform user how to make the models discoverable by Gazebo
    if export_kwargs["output_dir"] is not None:
        output_dir = path.abspath(export_kwargs["output_dir"])
        sdf_model_exporter._print_bpy(
            "Info: Please, set GZ_SIM_RESOURCE_PATH environment variable to make the "
            "exported models discoverable."
            f'\n\texport GZ_SIM_RESOURCE_PATH="{output_dir}'
            '${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"',
        )


if __name__ == "__main__":

    # Setup argument parser
    parser = argparse.ArgumentParser(
        description="Generate a procedural dataset of SDF models using Blender.",
        usage=(
            f"\n\t{sys.argv[0]} [blender options] file.blend --python-text "
            "script.py -- [script options]"
            f"\n\t{sys.argv[0]} [blender options] file.blend --python "
            "external_script.py -- [script options]"
            "\nlist script options:"
            f"\n\t{sys.argv[0]} -b file.blend --python-text script.py -- -h"
            f"\n\t{sys.argv[0]} -b file.blend --python external_script.py -- -h"
            "\nlist blender options:"
            f"\n\t{sys.argv[0]} -h"
        ),
        epilog=f"Default keyword arguments of the script (model): {DEFAULT_KWARGS}"
        if DEFAULT_KWARGS
        else "",
        argument_default=argparse.SUPPRESS,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Append example to usage (with the specific internal/external script)
    if "--python-text" in sys.argv and bpy.data.filepath:
        blend_relpath = path.relpath(path=bpy.data.filepath, start=os.getcwd())
        py_relpath = path.relpath(path=__file__, start=bpy.data.filepath)
        parser.usage += (
            f"\nexample:\n\t{sys.argv[0]} {blend_relpath} --python-text {py_relpath} "
            f"-- -o {OUTPUT_DIR}"
        )
    else:
        py_relpath = path.relpath(path=__file__, start=os.getcwd())
        parser.usage += (
            f"\nexample:\n\t{sys.argv[0]} file.blend --python {py_relpath} "
            f"-- -o {OUTPUT_DIR}"
        )

    # Exit if opened without a Blender project
    if not bpy.data.filepath:
        print(
            "Err: Blender project was not opened before running the script.",
            file=sys.stderr,
        )
        parser.print_usage()

        sys.exit(22)

    ### Specify all CLI arguments
    ## Helper argparse types
    def str2bool(value: Union[str, bool]) -> bool:
        """
        Convert logical string to boolean. Can be used as argparse type.
        """

        if isinstance(value, bool):
            return value
        if value.lower() in ("yes", "true", "t", "y", "1"):
            return True
        elif value.lower() in ("no", "false", "f", "n", "0"):
            return False
        else:
            raise argparse.ArgumentTypeError("Boolean value expected.")

    def str_empty2none(value: Optional[str]) -> Optional[str]:
        """
        If string is empty, convert to None. Can be used as argparse type.
        """

        return str(value) if value else None

    ## Command mode of the script
    parser.add_argument(
        "--export-dataset",
        action="store_true",
        default=True,
        help="[default mode] Generate a procedural dataset.",
    )
    parser.add_argument(
        "--export-model",
        dest="export_dataset",
        action="store_false",
        help="[alternative mode] Export a single model.",
    )
    ## Parameters for SDF model exporter `sdf_model_exporter`
    parser.add_argument(
        "-o",
        "--output-dir",
        type=str_empty2none,
        default=OUTPUT_DIR,
        help="The output directory for all models (each model is located under its own "
        "subdirectory). When exporting a single SDF model, its output path is "
        "'OUTPUT_DIR/MODEL_NAME'. For datasets, the path of each SDF model name is "
        "appended by the random seed used during its generation as "
        "'OUTPUT_DIR/MODEL_NAME+VARIANT_SEED'. If set to empty, Fuel cache "
        "'${GZ_FUEL_CACHE_PATH}/fuel.gazebosim.org/${USER}/models' is used.",
    )
    parser.add_argument(
        "-v",
        "--model-version",
        type=int,
        default=MODEL_VERSION,
        help="The version of exported model path that is joined with the model output "
        "path 'OUTPUT_DIR/MODEL_NAME/MODEL_VERSION', or "
        "'OUTPUT_DIR/MODEL_NAME+VARIANT_SEED/MODEL_VERSION'. The output path is "
        "modified only if 'MODEL_VERSION' is set (not None). For negative values "
        "and an existing model directory, the next unique version is used to avoid "
        "overwriting. If 'OUTPUT_DIR' is empty (export to Fuel cache), a negative "
        "value is used to guarantee a unique version.",
    )
    parser.add_argument(
        "--model-name-prefix",
        type=str,
        help="Prefix of exported model as 'OUTPUT_DIR/MODEL_NAME_PREFIX+MODEL_NAME' "
        ", or 'OUTPUT_DIR/MODEL_NAME_PREFIX+MODEL_NAME+VARIANT_SEED' (default: '').",
    )
    parser.add_argument(
        "--model-name-suffix",
        type=str,
        help="Suffix of exported model as 'OUTPUT_DIR/MODEL_NAME+MODEL_NAME_SUFFIX' "
        ", or 'OUTPUT_DIR/MODEL_NAME+MODEL_NAME_SUFFIX+VARIANT_SEED' (default: '').",
    )
    ## Parameters for procedural dataset generator `procedural_dataset_generator`
    parser.add_argument(
        "-s",
        "--first-seed",
        type=int,
        default=FIRST_SEED,
        help="The random seed of the first model when generating a dataset.",
    )
    parser.add_argument(
        "-n",
        "--number-of-variants",
        type=int,
        default=NUMBER_OF_VARIANTS,
        help="Number of model variants to export when generating a dataset.",
    )
    ## Filetypes of exported geometry
    parser.add_argument(
        "--filetype-visual",
        type=str,
        default=FILETYPE_VISUAL,
        choices=["collada", "dae", "wavefront", "obj", "stl"],
        help="The format of exported visual geometry ['collada' == 'dae', "
        "'wavefront' == 'obj'].",
    )
    parser.add_argument(
        "--filetype-collision",
        type=str,
        default=FILETYPE_COLLISION,
        choices=["collada", "dae", "wavefront", "obj", "stl"],
        help="The format of exported collision geometry ['collada' == 'dae', "
        "'wavefront' == 'obj'].",
    )
    ## Level of detail for exported visual/collision geometry, e.g. subdivision level
    parser.add_argument(
        "--detail-level-visual",
        type=int,
        default=DETAIL_LEVEL_VISUAL,
        help="Level of detail for exported visual geometry, e.g. subdivision level.",
    )
    parser.add_argument(
        "--detail-level-collision",
        type=int,
        default=DETAIL_LEVEL_COLLISION,
        help="Level of detail for exported collision geometry, e.g. subdivision level.",
    )
    ## Objects to ignore while exporting visual/collision geometry, even if selected
    parser.add_argument(
        "--ignore-objects-visual",
        type=str,
        nargs="+",
        default=IGNORE_OBJECTS_VISUAL,
        help="List of objects to ignore when exporting visual geometry.",
    )
    parser.add_argument(
        "--ignore-objects-collision",
        type=str,
        nargs="+",
        default=IGNORE_OBJECTS_COLLISION,
        help="List of objects to ignore when exporting collision geometry.",
    )
    ## Source of textures for the model
    parser.add_argument(
        "--symlink-external-textures",
        type=str2bool,
        default=SYMLINK_EXTERNAL_TEXTURES,
        help="If true, symbolic links will be created for all textures instead of "
        "copies. No copy or symlink will be if the texture is already under the output "
        "path.",
    )
    parser.add_argument(
        "--texture-source-mode",
        type=str,
        choices=["none", "path", "online", "blender"],
        default=TEXTURE_SOURCE_MODE,
        help="The source from which to select/extract (PBR) textures for the model. "
        "Option 'none' disables texturing in SDF and relies on mesh exporters. "
        "Option 'path' should either point to a single set of textures or a number of "
        "texture sets, from which a set would be sampled at random. "
        "Options 'online' (textures from an online source) and 'blender' (baking of "
        "textures) are currently not implemented. "
        "The value must be specified using the 'TEXTURE_SOURCE_VALUE' option.",
    )
    parser.add_argument(
        "--texture-source-value",
        type=str_empty2none,
        default=TEXTURE_SOURCE_VALUE,
        help="Value for the texture source, with the context based on the selected "
        "'TEXTURE_SOURCE_MODE'. For example, this value expresses path to a directory "
        "with textures or a name of the environment.",
    )
    parser.add_argument(
        "--material-texture-diffuse",
        type=float,
        nargs="+",
        default=MATERIAL_TEXTURE_DIFFUSE,
        help="Diffuse intensity of the albedo texture map. "
        "Please, enter values for each channel as `--material-texture-diffuse R G B`.",
    )
    parser.add_argument(
        "--material-texture-specular",
        type=float,
        nargs="+",
        default=MATERIAL_TEXTURE_SPECULAR,
        help="Specular intensity of the albedo texture map. "
        "Please, enter values for each channel as `--material-texture-specular R G B`.",
    )
    ## Inertial and dynamic properties of exported models
    parser.add_argument(
        "--static",
        type=str2bool,
        default=STATIC,
        help="If true, the SDF model is exported as immovable and it won't be updated "
        "by the physics engine.",
    )
    parser.add_argument(
        "--inertial-estimation-mode",
        type=str,
        choices=["none", "density", "random_density", "mass", "random_mass"],
        default=INERTIAL_ESTIMATION_MODE,
        help="The mode used during the estimation of inertial properties. "
        "Option 'none' disables estimation of inertial properties. "
        "Option '[random_]density' assumes a uniform density of the model. "
        "Option '[random_]mass' determines a uniform density based on the target mass. "
        "Random options uniformly sample the target value from a specified range. "
        "The value must be specified using the 'INERTIAL_ESTIMATION_VALUE' option. "
        "Estimation of inertial properties is always disabled for 'STATIC' models.",
    )
    parser.add_argument(
        "--inertial-estimation-value",
        type=float,
        nargs="+",
        default=INERTIAL_ESTIMATION_VALUE,
        help="Value for the inertial estimation, with the context based on the "
        "selected 'INERTIAL_ESTIMATION_MODE'. "
        "For non-random modes, please use a single value as "
        "`--inertial-estimation-value TARGER_VALUE`. "
        "For a random range, please enter min and max values as "
        "`--inertial-estimation-value MIN MAX`.",
    )
    parser.add_argument(
        "--inertial-estimation-use-collision-mesh",
        type=str2bool,
        help="If true, collision geometry will be used for the estimation of inertial "
        "properties instead of the visual geometry of the model. (default: True)",
    )
    parser.add_argument(
        "--friction-coefficient",
        type=float,
        nargs="+",
        default=FRICTION_COEFFICIENT,
        help="Coefficient of the surface friction, equal in both directions. "
        "For a random range, please enter min and max values as "
        "`--friction-coefficient MIN MAX`.",
    )
    ## Miscellaneous
    parser.add_argument(
        "--generate-thumbnails",
        type=str2bool,
        help="If true, thumbnails will be generated for the exported models. Only "
        "applicable if Blender is run in the foreground without `-b`. (default: True)",
    )

    # Parse arguments
    if "--" in sys.argv:
        args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    else:
        args, unknown_args = parser.parse_known_args()
        sdf_model_exporter._print_bpy(
            f"Warn: The following arguments are not recognized by '{py_relpath}'!"
            f"\n\t{unknown_args}\nHint: Please, delimit your arguments for Python "
            "script with '--' (see usage).",
            file=sys.stderr,
        )

    # Pass as keyword arguments to the main function
    main(**vars(args))
