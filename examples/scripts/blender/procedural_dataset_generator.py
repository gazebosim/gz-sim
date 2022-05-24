"""
Blender exporter of SDF models with support for generating procedural datasets via node
based modifiers, e.g. Geometry Nodes.
"""
from __future__ import annotations

# Please, manually adjust the last tested (and working) Blender version for this script
LAST_TESTED_VERSION: Tuple(int, int, int) = (3, 1, 0)


### Default parameters for `sdf_model_exporter`
DIRNAME_EXPORT: str = "./blender_export"
MODEL_VERSION: Optional[int] = None
###

### Default parameters for `procedural_dataset_generator`
INITIAL_SEED: int = 0
NUMBER_OF_VARIANTS: int = 8
###

### Default model-specific parameters
STATIC: bool = False

SUBDIVISION_LEVEL_VISUAL: int = 4
SUBDIVISION_LEVEL_COLLISION: int = 2

IGNORE_OBJECT_NAMES_VISUAL: List[str] = []
IGNORE_OBJECT_NAMES_COLLISION: List[str] = []

SHADE_SMOOTH: bool = True
TEXTURE_SOURCE: str = "none"
TEXTURE_SOURCE_VALUE: Optional[Any] = None
MATERIAL_TEXTURE_DIFFUSE: Union[
    Tuple[float, float, float], Tuple[float, float, float, float]
] = (1.0, 1.0, 1.0, 1.0)
MATERIAL_TEXTURE_SPECULAR: Union[
    Tuple[float, float, float], Tuple[float, float, float, float]
] = (0.2, 0.2, 0.2, 1.0)

MODEL_TARGET_MASS: Optional[Union[float, Tuple[float, float]]] = None
MODEL_DENSITY: Union[float, Tuple[float, float]] = 1.0

FRICTION_COEFFICIENT: Optional[Union[float, Tuple[float, float]]] = 1.0
###

### All additional keyword arguments can go here, e.g. `{"filetype_visual": "OBJ"}`
DEFAULT_KWARGS: Any = {}
###


### Imports
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

# Raise an informative error in case the script is run outside Python env of Blender
try:
    import bpy
except ImportError as err:
    raise ImportError(
        "Python module of Blender 'bpy' not found! Please, execute this script inside "
        "a Blender environment (e.g. copy-paste into Scripting tab)."
    ) from err
###


class sdf_model_exporter(ModuleType):
    """
    Blender exporter of mesh objects as SDF models.

    Note: This class is designed as a "module class" to enable simple copy-pasting
    among Blender projects and environments, i.e. methods do not require an instance
    during their call while still supporting inheritance.
    """

    # TODO[desing decision]: Consider exporting individual meshes for each object
    #       Exporting objects individually and genering SDF with multiple <visual>
    #       and <colision> tags might be more robust
    # Benefit  - Support multiple materials with different textures and overlapping UVs
    # Drawback - Much more complex, especially for estimating inertial properties

    # Default args for export
    DEFAULT_DIRNAME_EXPORT: str = DIRNAME_EXPORT
    DEFAULT_MODEL_VERSION: Optional[int] = MODEL_VERSION

    # Default args for model being static
    DEFAULT_STATIC: bool = STATIC

    # Default args for which objects to ignore from mesh exporting
    DEFAULT_IGNORE_OBJECT_NAMES_VISUAL: List[str] = IGNORE_OBJECT_NAMES_VISUAL
    DEFAULT_IGNORE_OBJECT_NAMES_COLLISION: List[str] = IGNORE_OBJECT_NAMES_COLLISION

    # Default args for subdivision level
    DEFAULT_SUBDIVISION_LEVEL_VISUAL: int = SUBDIVISION_LEVEL_VISUAL
    DEFAULT_SUBDIVISION_LEVEL_COLLISION: int = SUBDIVISION_LEVEL_COLLISION

    # Default args for smooth shading (visual geometry)
    DEFAULT_SHADE_SMOOTH: bool = SHADE_SMOOTH

    # Default args for material
    DEFAULT_MATERIAL_TEXTURE_DIFFUSE: Union[
        Tuple[float, float, float], Tuple[float, float, float, float]
    ] = MATERIAL_TEXTURE_DIFFUSE
    DEFAULT_MATERIAL_TEXTURE_SPECULAR: Union[
        Tuple[float, float, float], Tuple[float, float, float, float]
    ] = MATERIAL_TEXTURE_SPECULAR

    # Default args for inertial properties
    DEFAULT_MODEL_TARGET_MASS: Optional[
        Union[float, Tuple[float, float]]
    ] = MODEL_TARGET_MASS
    DEFAULT_MODEL_DENSITY: Union[float, Tuple[float, float]] = MODEL_DENSITY

    # Default args for surface friction
    DEFAULT_FRICTION_COEFFICIENT: Optional[
        Union[float, Tuple[float, float]]
    ] = FRICTION_COEFFICIENT

    # Default directory and basenames compatible with SDF directory structure
    # All of these are expressed with respect to the model base directory
    BASENAME_SDF: str = "model.sdf"
    BASENAME_SDF_MODEL_CONFIG: str = "model.config"
    DIRNAME_MESHES: str = "meshes"
    DIRNAME_MESHES_VISUAL: str = path.join(DIRNAME_MESHES, "visual")
    DIRNAME_MESHES_COLLISION: str = path.join(DIRNAME_MESHES, "collision")
    DIRNAME_MATERIALS: str = "materials"
    DIRNAME_TEXTURES: str = path.join(DIRNAME_MATERIALS, "textures")
    DIRNAME_THUMBNAILS: str = "thumbnails"
    # Targetted version of SDF
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
            only the selected mesh object(s) will be exported.
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
                    use_selection=True,
                    use_mesh_modifiers=True,
                )
            elif self.WAVEFRONT == self:
                bpy.ops.wm.obj_export(
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
                    filter_collada=False,
                    filter_alembic=False,
                    filter_usd=False,
                    filter_obj=True,
                    filter_volume=False,
                    filter_folder=True,
                    filter_blenlib=False,
                    filemode=8,
                    display_type="DEFAULT",
                    sort_method="DEFAULT",
                    export_animation=False,
                    start_frame=-2147483648,
                    end_frame=2147483647,
                    forward_axis="Y_FORWARD",
                    up_axis="Z_UP",
                    scaling_factor=1,
                    apply_modifiers=True,
                    export_eval_mode="DAG_EVAL_VIEWPORT",
                    export_selected_objects=False,
                    export_uv=True,
                    export_normals=True,
                    export_materials=True,
                    export_triangulated_mesh=False,
                    export_curves_as_nurbs=False,
                    export_object_groups=False,
                    export_material_groups=False,
                    export_vertex_groups=False,
                    export_smooth_groups=False,
                    smooth_group_bitflags=False,
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

            return cls.COLLADA

        @classmethod
        def default_collision(cls) -> sdf_model_exporter.ExportFormat:
            """
            Return the default filetype for collision mesh geometry.
            """

            return cls.STL

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

    @classmethod
    def export(
        cls,
        export_dir: Optional[str] = DEFAULT_DIRNAME_EXPORT,
        model_version: Optional[int] = DEFAULT_MODEL_VERSION,
        filetype_visual: Union[ExportFormat, str] = ExportFormat.default_visual(),
        filetype_collision: Union[ExportFormat, str] = ExportFormat.default_collision(),
        symlink_external_textures: bool = True,
        estimate_inertial_properties: bool = True,
        estimate_inertial_properties_from_collision_geometry: bool = False,
        estimate_inertial_properties_target_mass: Optional[
            Union[float, Tuple[float, float]]
        ] = DEFAULT_MODEL_TARGET_MASS,
        estimate_inertial_properties_density: Union[
            float, Tuple[float, float]
        ] = DEFAULT_MODEL_DENSITY,
        subdivision_level_visual: int = DEFAULT_SUBDIVISION_LEVEL_VISUAL,
        subdivision_level_collision: int = DEFAULT_SUBDIVISION_LEVEL_COLLISION,
        shade_smooth: bool = DEFAULT_SHADE_SMOOTH,
        ignore_object_names_visual: List[str] = DEFAULT_IGNORE_OBJECT_NAMES_VISUAL,
        ignore_object_names_collision: List[
            str
        ] = DEFAULT_IGNORE_OBJECT_NAMES_COLLISION,
        generate_thumbnails: bool = True,
        model_name_prefix: str = "",
        model_name_suffix: str = "",
        **kwargs,
    ):
        """
        The primary function enables exporting of SDF models.
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
        # TODO[feature]: Support exporting cameras and lights? Might be useful for some
        #                assets...
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
            model_name = path.basename(bpy.data.filepath).split(".")[0]
        # Add prefix and suffix if desired
        model_name = f"{model_name_prefix}{model_name}{model_name_suffix}"

        # If export directory is not set (not even "" or "."), use the default Ignition
        # Fuel model path
        # TODO[design]: I originally hoped that models exported to LocalCache of Fuel
        #               would be automatically discovered. However, it is not the case.
        #               Using `ign fuel update` does not make the models discoverable
        #               either ('Failed to fetch model details for model' is returned).
        #               Therefore, there might be little reason to export directly to
        #               the LocalCache of Fuel. Consider removing this...
        if export_dir is None:
            export_dir = path.join(
                os.environ.get(
                    "IGN_FUEL_CACHE_PATH",
                    default=path.join(path.expanduser("~"), ".ignition", "fuel"),
                ),
                "fuel.ignitionrobotics.org",
                os.getlogin(),
                "models",
            )
            # Ignition Fuel requires model versioning, therefore, default to the next
            # available version if the model already exists (begins with version 1)
            if model_version is None:
                model_version = -1
        # Get the absolute path and create the directory if it does not already exist
        export_dir = path.abspath(export_dir)
        os.makedirs(name=export_dir, exist_ok=True)

        # Get the versioned model path (if desired)
        export_path = cls.__try_get_versioned_model_path(
            export_dir=export_dir,
            model_name=model_name,
            model_version=model_version,
        )

        # Store all data necessary for SDF creation inside a dictionary
        sdf_data = {}

        # Process and export meshes
        exported_meshes = cls._process_meshes(
            meshes_to_process=meshes_to_process,
            export_path=export_path,
            model_name=model_name,
            filetype_visual=filetype_visual,
            filetype_collision=filetype_collision,
            subdivision_level_visual=subdivision_level_visual,
            subdivision_level_collision=subdivision_level_collision,
            shade_smooth=shade_smooth,
            ignore_object_names_visual=ignore_object_names_visual,
            ignore_object_names_collision=ignore_object_names_collision,
        )
        sdf_data.update(exported_meshes)

        # Sample textures
        textures = cls._sample_textures()
        sdf_data.update(textures)

        # Estimate inertial properties from the mesh (either visual or collision)
        if estimate_inertial_properties:
            analysis_mesh_filepath = path.join(
                export_path,
                exported_meshes["filepath_mesh_collision"]
                if estimate_inertial_properties_from_collision_geometry
                else exported_meshes["filepath_mesh_visual"],
            )
            inertial_properties = cls._estimate_inertial_properties(
                analysed_mesh_filepath=analysis_mesh_filepath,
                target_mass=estimate_inertial_properties_target_mass,
                density=estimate_inertial_properties_density,
            )
            sdf_data.update(inertial_properties)

        # Enable overriding of kwargs from the external caller
        sdf_data.update(kwargs)

        # Write data into an SDF file
        cls._generate_sdf_file(
            export_path=export_path,
            model_name=model_name,
            symlink_external_textures=symlink_external_textures,
            **sdf_data,
        )

        # Create a corresponding config file for the SDF model
        cls._generate_model_config_file(export_path=export_path, model_name=model_name)

        # Render few images to generate thumbnails
        if generate_thumbnails:
            cls._generate_thumbnails(export_path=export_path)

        cls._print_bpy(
            f"Info: Model '{model_name}' exported to 'file://{export_path}'."
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
                    bpy.ops.console.scrollback_append(
                        {"window": window, "screen": window.screen, "area": area},
                        text=str(msg),
                        type="ERROR" if file == sys.stderr else "OUTPUT",
                    )

    @classmethod
    def _process_meshes(
        cls,
        meshes_to_process: List,
        export_path: str,
        model_name: str,
        filetype_visual: Union[ExportFormat, str],
        filetype_collision: Union[ExportFormat, str],
        subdivision_level_visual: int,
        subdivision_level_collision: int,
        shade_smooth: bool,
        ignore_object_names_visual: List[str],
        ignore_object_names_collision: List[str],
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
        ignore_object_names_visual = [
            name
            for name in ignore_object_names_visual
            if name in meshes_to_process_names
        ]
        ignore_object_names_collision = [
            name
            for name in ignore_object_names_collision
            if name in meshes_to_process_names
        ]

        # Deselect all objects
        bpy.ops.object.select_all(action="DESELECT")
        # Select all desired meshes at the same time
        for obj in meshes_to_process:
            obj.select_set(True)

        return cls._export_geometry(
            export_path=export_path,
            model_name=model_name,
            filetype_visual=filetype_visual,
            filetype_collision=filetype_collision,
            subdivision_level_visual=subdivision_level_visual,
            subdivision_level_collision=subdivision_level_collision,
            shade_smooth=shade_smooth,
            ignore_object_names_visual=ignore_object_names_visual,
            ignore_object_names_collision=ignore_object_names_collision,
        )

    @classmethod
    def _export_geometry(
        cls,
        export_path: str,
        model_name: str,
        filetype_visual: ExportFormat,
        filetype_collision: ExportFormat,
        subdivision_level_visual: int,
        subdivision_level_collision: int,
        shade_smooth: bool,
        ignore_object_names_visual: List[str],
        ignore_object_names_collision: List[str],
    ) -> Dict[str, str]:
        """
        Export both visual and collision mesh geometry.
        """

        filepath_collision = cls._export_geometry_collision(
            export_path=export_path,
            model_name=model_name,
            filetype=filetype_collision,
            subdivision_level=subdivision_level_collision,
            ignore_object_names=ignore_object_names_collision,
        )
        filepath_visual = cls._export_geometry_visual(
            export_path=export_path,
            model_name=model_name,
            filetype=filetype_visual,
            subdivision_level=subdivision_level_visual,
            shade_smooth=shade_smooth,
            ignore_object_names=ignore_object_names_visual,
        )

        return {
            "filepath_mesh_collision": path.relpath(
                path=filepath_collision, start=export_path
            ),
            "filepath_mesh_visual": path.relpath(
                path=filepath_visual, start=export_path
            ),
        }

    @classmethod
    def _export_geometry_collision(
        cls,
        export_path: str,
        model_name: str,
        filetype: ExportFormat,
        subdivision_level: int,
        ignore_object_names: List[str],
    ) -> str:
        """
        Export collision geometry of the model with the specified `filetype`.
        Method `_pre_export_geometry_collision()` is called before the export.
        Method `_post_export_geometry_collision()` is called after the export.
        """

        # Hook call before export of collision geometry
        cls._pre_export_geometry_collision(
            subdivision_level=subdivision_level, ignore_object_names=ignore_object_names
        )

        resulting_export_path = filetype.export(
            path.join(export_path, cls.DIRNAME_MESHES_COLLISION, model_name)
        )

        # Hook call after export of collision geometry
        cls._post_export_geometry_collision(ignore_object_names=ignore_object_names)

        return resulting_export_path

    @classmethod
    def _export_geometry_visual(
        cls,
        export_path: str,
        model_name: str,
        filetype: ExportFormat,
        subdivision_level: int,
        shade_smooth: bool,
        ignore_object_names: List[str],
    ) -> str:
        """
        Export visual geometry of the model with the specified `filetype`.
        Method `_pre_export_geometry_visual()` is called before the export.
        Method `_post_export_geometry_visual()` is called after the export.
        """

        # Hook call before export of visual geometry
        cls._pre_export_geometry_visual(
            subdivision_level=subdivision_level,
            shade_smooth=shade_smooth,
            ignore_object_names=ignore_object_names,
        )

        resulting_export_path = filetype.export(
            path.join(export_path, cls.DIRNAME_MESHES_VISUAL, model_name)
        )

        # Hook call after export of visual geometry
        cls._post_export_geometry_collision(ignore_object_names=ignore_object_names)

        return resulting_export_path

    @classmethod
    def _estimate_inertial_properties(
        cls,
        analysed_mesh_filepath: str,
        target_mass: Optional[Union[float, Tuple[float, float]]],
        density: Union[float, Tuple[float, float]],
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
        Estimate inertial properties of the mesh assuming uniform density. If
        `target_mass` is passed in, it is used to compute density based on object's
        volume. Otherwise, `density` is used directly. Both `target_mass` and `density`
        can also be specified as a range and sampled randomly (uniform distribution).
        """

        # TODO[feature]: Consider supporting other methods/libraries for estimating
        #                inertial properties. Maybe there is some good Blender addon?

        # Import trimesh or try to install it within the Blender's Python environment
        # if it is not detected
        # Note: Depending on the installation process of Blender, 'sudo' might be
        #       required to perform this step. With snap installation, it might not be
        #       possible at all
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
                    "module manually (within Blender environment or using the Python "
                    "environment of the system via `--python-use-system-env` flag when "
                    "running Blender)."
                )
                cls._print_bpy(
                    f"Err: {err_msg}",
                    file=sys.stderr,
                )
                raise ImportError(err_msg) from err

        mesh = trimesh.load(analysed_mesh_filepath, force="mesh", ignore_materials=True)

        # Set the density (either through target mass, randomized mass/density or
        # directly with the passed value)
        if target_mass is not None:
            if isinstance(target_mass, Iterable) and len(target_mass) == 2:
                # Randomize target mass if a range is passed
                target_mass = random.uniform(target_mass[0], target_mass[1])
            # Deduce density from the target mass
            mesh.density = target_mass / mesh.volume
        elif isinstance(density, Iterable):
            # TODO[design/discussion]: Maybe Gaussian distribution is more appropriate?
            # Randomize density if a range is passed
            mesh.density = random.uniform(density[0], density[1])
        else:
            # Otherwise just used the passed density
            mesh.density = density

        return {
            "mass": mesh.mass,
            "inertia": mesh.moment_inertia,
            "centre_of_mass": mesh.center_mass,
        }

    @classmethod
    def _sample_textures(
        cls,
    ) -> Dict[str, Optional[str]]:
        """
        Abstract method for getting PBR textures for the exported model. These can have
        different origins, e.g. baked from Blender or imported from external sources.
        No texture is returned by default.
        """

        # TODO[feature]: Default to baked model textures (once implemented)
        #                (Taking the base color texture would not support procedural
        #                textures)
        return {}  # abstractclassmethod

    def _generate_baked_textures(
        cls,
    ) -> str:
        """
        Bake PBR textures for the model and return a path to a directory that contains
        them.
        """

        # TODO[feature]: Find a way of including baked textures
        return NotImplementedError("Baking of textures is not yet implemented!")

    @classmethod
    def _generate_sdf_file(
        cls,
        export_path: str,
        model_name: str,
        filepath_mesh_visual: str,
        filepath_mesh_collision: str,
        static: bool = DEFAULT_STATIC,
        material_texture_diffuse: Union[
            Tuple[float, float, float], Tuple[float, float, float, float]
        ] = DEFAULT_MATERIAL_TEXTURE_DIFFUSE,
        material_texture_specular: Union[
            Tuple[float, float, float], Tuple[float, float, float, float]
        ] = DEFAULT_MATERIAL_TEXTURE_SPECULAR,
        symlink_external_textures: bool = True,
        texture_albedo: Optional[str] = None,
        texture_roughness: Optional[str] = None,
        texture_metalness: Optional[str] = None,
        texture_normal: Optional[str] = None,
        friction_coefficient: Optional[
            Union[float, Tuple[float, float]]
        ] = DEFAULT_FRICTION_COEFFICIENT,
        mass: Optional[float] = None,
        inertia: Optional[
            Tuple[
                Tuple[float, float, float],
                Tuple[float, float, float],
                Tuple[float, float, float],
            ]
        ] = None,
        centre_of_mass: Optional[Tuple[float, float, float]] = None,
    ):
        """
        Generate SDF file from passed arguments and export to `export_path`.
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
        # TODO[feature]: Add better mapping of materials (in addition to COLLADA
        #                exporter)
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
                cls.__preprocess_texture_path(
                    texture,
                    export_path=export_path,
                    symlink_external_textures=symlink_external_textures,
                )
                for texture in textures
            )

            if texture_albedo:
                diffuse = ElementTree.SubElement(material, "diffuse")
                diffuse.text = " ".join(map(str, material_texture_diffuse))
                specular = ElementTree.SubElement(material, "specular")
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
        if (
            isinstance(friction_coefficient, Iterable)
            and len(friction_coefficient) == 2
        ):
            # Randomize friction coefficient if a range is passed
            friction_coefficient = random.uniform(
                friction_coefficient[0], friction_coefficient[1]
            )
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
        sdf_file = open(path.join(export_path, cls.BASENAME_SDF), "w")
        sdf_file.write(sdf_xml_string)
        sdf_file.close()

    @classmethod
    def _generate_model_config_file(
        cls,
        export_path: str,
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
        version = ElementTree.SubElement(model_config, "version")
        maybe_version = path.basename(export_path)
        version.text = maybe_version if maybe_version.isnumeric() else "1"

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
        script_relpath = path.join(
            path.basename(bpy.data.filepath),
            path.relpath(path=__file__, start=bpy.data.filepath),
        )
        description = ElementTree.SubElement(model_config, "description")
        description.text = (
            f"Model generated from '{path.basename(bpy.data.filepath)}' by "
            f"'{script_relpath}' Python script"
        )

        # Convert config to xml string and write to file
        model_config_xml_string = minidom.parseString(
            ElementTree.tostring(model_config, encoding="unicode")
        ).toprettyxml(indent="  ")
        model_config_file = open(
            path.join(export_path, cls.BASENAME_SDF_MODEL_CONFIG), "w"
        )
        model_config_file.write(model_config_xml_string)
        model_config_file.close()

    @classmethod
    def _generate_thumbnails(cls, export_path: str):
        """
        Render thumbnails for the SDF model.
        Currently, only a single viewport render is created using OpenGL.
        """

        # Create thumbnails directory for the model
        thumbnails_dir = path.join(export_path, cls.DIRNAME_THUMBNAILS)
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
        cls, subdivision_level: int, ignore_object_names: List[str] = []
    ):
        """
        A hook that is called before exporting collision geometry. Always chain up the
        parent implementation.
        By default, this hook handles reselecting objects from `ignore_object_names`.
        """

        cls.__select_objects(
            names=ignore_object_names, type_filter="MESH", select=False
        )

    @classmethod
    def _post_export_geometry_collision(cls, ignore_object_names: List[str] = []):
        """
        A hook that is called after exporting collision geometry. Always chain up the
        parent implementation.
        By default, this hook handles deselecting objects from `ignore_object_names`.
        """

        cls.__select_objects(names=ignore_object_names, type_filter="MESH", select=True)

    @classmethod
    def _pre_export_geometry_visual(
        cls,
        subdivision_level: int,
        shade_smooth: bool,
        ignore_object_names: List[str] = [],
    ):
        """
        A hook that is called before exporting visual geometry. Always chain up the
        parent implementation.
        By default, this hook handles reselecting objects from `ignore_object_names`.
        """

        cls.__select_objects(
            names=ignore_object_names, type_filter="MESH", select=False
        )

    @classmethod
    def _post_export_geometry_visual(cls, ignore_object_names: List[str] = []):
        """
        A hook that is called after exporting visual geometry. Always chain up the
        parent implementation.
        By default, this hook handles deselecting objects from `ignore_object_names`.
        """

        cls.__select_objects(names=ignore_object_names, type_filter="MESH", select=True)

    def __select_objects(
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

    @classmethod
    def __preprocess_texture_path(
        cls,
        texture_original_filepath: Optional[str],
        export_path: str,
        symlink_external_textures: bool,
    ):
        """
        Preprocess filepath of a texture such that it is in the local model directory
        path. If `symlink_external_textures` is enabled, a symbolic link will be
        created. No copy/symlink will be made is the `texture_original_filepath` is
        already a subpath of `export_path`.
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
        if texture_original_filepath.startswith(export_path):
            texture_target_filepath = texture_original_filepath
        else:
            texture_dir = path.join(export_path, cls.DIRNAME_TEXTURES)
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

        return path.relpath(path=texture_target_filepath, start=export_path)

    @classmethod
    def __try_get_versioned_model_path(
        cls, export_dir: str, model_name: str, model_version: Optional[int]
    ) -> str:
        """
        Return versioned model directory path if `model_version` is specified. For
        negative `model_version` and existing model directory, the next version will
        be used to avoid overwriting.
        """

        unversioned_model_path = path.join(export_dir, model_name)

        if model_version is None:
            return unversioned_model_path
        elif model_version < 0:
            return path.join(
                unversioned_model_path,
                str(cls.__get_next_model_version(model_path=unversioned_model_path)),
            )
        else:
            return path.join(unversioned_model_path, model_version)

    def __get_next_model_version(model_path: str) -> int:
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

    # Default export args
    DEFAULT_INITIAL_SEED: int = INITIAL_SEED
    DEFAULT_NUMBER_OF_VARIANTS: int = NUMBER_OF_VARIANTS

    # Default texture source
    DEFAULT_TEXTURE_SOURCE: str = TEXTURE_SOURCE
    DEFAULT_TEXTURE_SOURCE_VALUE: Optional[Any] = TEXTURE_SOURCE_VALUE

    # The following lookup phrases are used to find the corresponding input attributes
    # of the node system (exact match, insensitive to case, insensitive to '_'/'-'/...)
    LOOKUP_PHRASES_RANDOM_SEED: List[str] = [
        "rng",
        "seed",
        "randomseed",
        "pseodorandomseed",
    ]
    LOOKUP_PHRASES_SUBDIVISION_LEVEL: List[str] = [
        "detail",
        "detailobject",
        "levelofdetail",
        "subdivionlevel",
        "subdivlevel",
    ]
    LOOKUP_PHRASES_SHADE_SMOOTH: List[str] = [
        "shadesmooth",
        "smoothshade",
        "smoothshading",
    ]

    @enum.unique
    class TextureSource(enum.Enum):
        """
        Helper enum of possible sources for textures. Each source should provide a path
        to searchable directory with the following structure (each texture type is
        optional and image format must be supported by Ignition Gazebo):
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
        ENV_VARIABLE = enum.auto()
        ONLINE = enum.auto()

        def get_path(self, value: Optional[Any] = None) -> Optional[str]:
            """
            Return a path to a directory with PBR textures.
            """

            if self.NONE == self:
                return None
            elif self.BLENDER_MODEL == self:
                return path.abspath(sdf_model_exporter._generate_baked_textures())
            elif self.FILEPATH == self:
                return path.abspath(value)
            elif self.ENV_VARIABLE == self:
                return os.environ.get(str(value), default=None)
            elif self.ONLINE == self:
                # TODO(feature): Implement option for pulling PBR textures from an
                #                online source
                return NotImplementedError(
                    "Getting textures from an online source is not yet implemented!"
                )
            else:
                raise ValueError(f"Texture source '{self}' is not supported.")

        @classmethod
        def from_str(
            cls, source_str: str
        ) -> procedural_dataset_generator.TextureSource:
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
            elif "ENV" in source_str and "VAR" in source_str:
                return cls.ENV_VARIABLE
            elif "ONLINE" in source_str:
                return cls.ONLINE
            else:
                raise ValueError(f"Unknown '{source_str}' texture source.")

        @classmethod
        def default(cls) -> procedural_dataset_generator.TextureSource:
            """
            Return the default texture source.
            """

            return cls.NONE

    @classmethod
    def generate(
        cls,
        initial_seed: int = DEFAULT_INITIAL_SEED,
        number_of_variants: int = DEFAULT_NUMBER_OF_VARIANTS,
        texture_source: Union[TextureSource, str] = DEFAULT_TEXTURE_SOURCE,
        texture_source_value: Optional[Any] = DEFAULT_TEXTURE_SOURCE_VALUE,
        redraw_viewport_during_processing: bool = True,
        **kwargs,
    ):
        """
        Generate `number_of_variants` different models by changing the random seed in
        the Geometry Nodes system of the individual meshes.
        """

        cls._print_bpy(
            f"Generating {number_of_variants} model variants in the seed range "
            f"[{initial_seed}, {initial_seed + number_of_variants}]."
        )

        # Get path to textures (if enabled)
        global textures_dirpath
        if not isinstance(texture_source, cls.TextureSource):
            texture_source = cls.TextureSource.from_str(texture_source)
        textures_dirpath = texture_source.get_path(texture_source_value)
        if not textures_dirpath:
            cls._print_bpy(
                "Warn: Models will be exported without any textures.",
                file=sys.stderr,
            )
        elif not path.isdir(textures_dirpath):
            raise ValueError(
                f"Texture directory '{textures_dirpath}' is not valid!",
            )

        # Export models for the entire range of random seeds
        global current_seed
        for seed in range(initial_seed, initial_seed + number_of_variants):
            current_seed = seed
            random.seed(seed)
            cls.export(model_name_suffix=str(seed), **kwargs)

            # Update the viewport to keep track of progress and look *fancy*
            # Performance might be lowered because the scene needs to be re-rendered
            if redraw_viewport_during_processing:
                bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)

    @classmethod
    def _pre_export_geometry_collision(
        cls, subdivision_level: int, ignore_object_names: List[str] = []
    ):
        """
        A hook that is called before exporting collision geometry. This implementation
        adjusts input attributes of the Geometry Nodes system for each mesh.
        """

        # Call parent impl
        sdf_model_exporter._pre_export_geometry_collision(
            subdivision_level=subdivision_level,
            ignore_object_names=ignore_object_names,
        )

        global current_seed
        selected_meshes = bpy.context.selected_objects
        for obj in selected_meshes:
            for nodes_modifier in cls.__get_all_nodes_modifiers(obj):
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier, cls.LOOKUP_PHRASES_RANDOM_SEED, current_seed
                )
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_SUBDIVISION_LEVEL,
                    subdivision_level,
                )
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier, cls.LOOKUP_PHRASES_SHADE_SMOOTH, int(False)
                )
            cls.__trigger_modifier_update(obj)

    @classmethod
    def _pre_export_geometry_visual(
        cls,
        subdivision_level: int,
        shade_smooth: bool,
        ignore_object_names: List[str] = [],
    ):
        """
        A hook that is called before exporting visual geometry. This implementation
        adjusts input attributes of the Geometry Nodes system for each mesh.
        """

        # Call parent impl
        sdf_model_exporter._pre_export_geometry_visual(
            subdivision_level=subdivision_level,
            shade_smooth=shade_smooth,
            ignore_object_names=ignore_object_names,
        )

        global current_seed
        selected_meshes = bpy.context.selected_objects
        for obj in selected_meshes:
            for nodes_modifier in cls.__get_all_nodes_modifiers(obj):
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier, cls.LOOKUP_PHRASES_RANDOM_SEED, current_seed
                )
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_SUBDIVISION_LEVEL,
                    subdivision_level,
                )
                cls.__try_set_nodes_input_attribute(
                    nodes_modifier,
                    cls.LOOKUP_PHRASES_SHADE_SMOOTH,
                    int(shade_smooth),
                )

            cls.__trigger_modifier_update(obj)

    @classmethod
    def _sample_textures(
        cls,
    ) -> Dict[str, Optional[str]]:
        """
        Abstract method for getting PBR textures for the exported model. These can have
        different origins, e.g. baked from Blender or imported from external sources.
        This implementation returns a set of textures depending on global
        `textures_dirpath`.
        """

        texture_albedo: Optional[str] = None
        texture_roughness: Optional[str] = None
        texture_metalness: Optional[str] = None
        texture_normal: Optional[str] = None
        global textures_dirpath
        if textures_dirpath:
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
            for texture in os.listdir(texture_dirpath):
                texture_cmp = cls.__unify_string(texture)
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

    def __get_all_nodes_modifiers(obj: bpy.types.Object):
        """
        Return all node-based modifiers of an object.
        """

        return [modifier for modifier in obj.modifiers if modifier.type == "NODES"]

    @classmethod
    def __try_set_nodes_input_attribute(
        cls,
        modifier: bpy.types.NodesModifier,
        lookup_phrases: Iterable[str],
        value: Any,
    ):
        """
        Try to set an input attribute of a nodes system to a `value`. The attribute
        looked is performed by using `lookup_phrases`.
        """

        # Try to find the corresponding ID of the input attribute
        input_id: Optional[str] = None
        for attribute in modifier.node_group.inputs:
            if cls.__unify_string(attribute.name) in lookup_phrases:
                input_id = attribute.identifier
                break
        if input_id is None:
            cls._print_bpy(
                f"Warn: Cannot match an input attribute of '{modifier.name}' for any "
                f"of the requested lookup phrases {lookup_phrases}.",
                file=sys.stderr,
            )
            return

        # Set the attribute
        modifier[input_id] = value

    def __trigger_modifier_update(obj: bpy.types.Object):
        """
        Trigger an update of object's modifiers after changing its attributes.
        """

        # Not sure how else to trigger update of the modifiers, but setting the index
        # of any modifier does the trick (even if the index stays the same)
        # TODO[enhancement]: Figure out a better way of updating modifiers after
        #                    programatic changes
        bpy.context.view_layer.objects.active = obj
        if len(obj.modifiers.values()):
            bpy.ops.object.modifier_move_to_index(
                modifier=obj.modifiers.values()[0].name,
                index=0,
            )

    def __unify_string(
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


def main(**kwargs):

    # Warn the user in case an untested version of Blender is used
    if (
        bpy.app.version[0] != LAST_TESTED_VERSION[0]
        or bpy.app.version[1] < LAST_TESTED_VERSION[1]
    ):
        sdf_model_exporter._print_bpy(
            f"Warn: Untested version of Blender detected ({bpy.app.version_string})! "
            "This script might not work as originally intended. Please consider using "
            f"version [>={LAST_TESTED_VERSION[0]}.{LAST_TESTED_VERSION[1]}].",
            file=sys.stderr,
        )

    # Update default keyword arguments with parsed arguments
    export_kwargs = DEFAULT_KWARGS
    export_kwargs.update(kwargs)

    ### Generate a single SDF model
    # sdf_model_exporter.export(**export_kwargs)

    ### Generate a dataset of procedural models
    procedural_dataset_generator.generate(**export_kwargs)


if __name__ == "__main__":
    kwargs = {
        arg.split("=")[0]: arg.split("=")[1] for arg in sys.argv[1:] if "=" in arg
    }
    main(**kwargs)
