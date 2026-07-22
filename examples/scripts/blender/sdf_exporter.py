import xml.etree.ElementTree as ET
from os import path
from xml.dom import minidom

import bpy
from bpy.props import StringProperty
from bpy.types import Operator
from bpy_extras.io_utils import ImportHelper

# Tested Blender version: 4.2/4.3

########################################################################################################################
### Exports model.dae of the scene with textures, its corresponding model.sdf file, and a default model.config file ####
########################################################################################################################
def export_sdf(prefix_path):

    dae_filename = "model.dae"
    sdf_filename = "model.sdf"
    model_config_filename = "model.config"
    lightmap_filename = "LightmapBaked.png"
    model_name = "my_model"
    meshes_folder_prefix = "meshes/"

    # Exports the dae file and its associated textures
    bpy.ops.wm.collada_export(
        filepath=path.join(prefix_path, meshes_folder_prefix, dae_filename),
        check_existing=False,
        filter_blender=False,
        filter_image=False,
        filter_movie=False,
        filter_python=False,
        filter_font=False,
        filter_sound=False,
        filter_text=False,
        filter_btx=False,
        filter_collada=True,
        filter_folder=True,
        filemode=8,
    )

    # objects = bpy.context.selected_objects
    objects = bpy.context.selectable_objects
    mesh_objects = [o for o in objects if o.type == "MESH"]
    light_objects = [o for o in objects if o.type == "LIGHT"]

    #############################################
    #### export sdf xml based off the scene #####
    #############################################
    sdf = ET.Element("sdf", attrib={"version": "1.8"})

    # 1 model and 1 link
    model = ET.SubElement(sdf, "model", attrib={"name": "test"})
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", attrib={"name": "testlink"})
    # for each geometry in geometry library add a <visual> tag
    for o in mesh_objects:
        visual = ET.SubElement(link, "visual", attrib={"name": o.name})

        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        uri = ET.SubElement(mesh, "uri")
        uri.text = path.join(meshes_folder_prefix, dae_filename)
        submesh = ET.SubElement(mesh, "submesh")
        submesh_name = ET.SubElement(submesh, "name")
        submesh_name.text = o.name

        # grab diffuse/albedo map
        diffuse_map = ""
        if o.active_material is not None:
            nodes = o.active_material.node_tree.nodes
            principled = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
            if principled is not None:
                base_color = principled.inputs["Base Color"]
                if len(base_color.links):
                    link_node = base_color.links[0].from_node
                    diffuse_map = link_node.image.name

        # setup diffuse/specular color
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = "1.0 1.0 1.0 1.0"
        specular = ET.SubElement(material, "specular")
        specular.text = "0.0 0.0 0.0 1.0"
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        if diffuse_map != "":
            albedo_map = ET.SubElement(metal, "albedo_map")
            albedo_map.text = path.join(meshes_folder_prefix, diffuse_map)

        # for lightmapping, add the UV and turn off casting of shadows
        if path.isfile(lightmap_filename):
            light_map = ET.SubElement(metal, "light_map", attrib={"uv_set": "1"})
            light_map.text = path.join(meshes_folder_prefix, lightmap_filename)

            cast_shadows = ET.SubElement(visual, "cast_shadows")
            cast_shadows.text = "0"

    def add_attenuation_tags(light_tag, blender_light):
        attenuation = ET.SubElement(light_tag, "attenuation")
        range = ET.SubElement(attenuation, "range")
        range.text = str(blender_light.cutoff_distance)
        linear = ET.SubElement(attenuation, "linear")
        linear.text = "1.0"
        quadratic = ET.SubElement(attenuation, "quadratic")
        quadratic.text = "0.0"
        constant = ET.SubElement(attenuation, "constant")
        constant.text = "1.0"


    # export lights
    for l in light_objects:
        blender_light = l.data

        if blender_light.type == "POINT":
            light = ET.SubElement(link, "light", attrib={"name": l.name, "type": "point"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"
            add_attenuation_tags(light, blender_light)

        if blender_light.type == "SPOT":
            light = ET.SubElement(link, "light", attrib={"name": l.name, "type": "spot"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"
            add_attenuation_tags(light, blender_light)

            # note: unsupported <spot> tags in blender
            spot = ET.SubElement(light, "spot")
            inner_angle = ET.SubElement(spot, "inner_angle")
            inner_angle.text = str(blender_light.spot_size * 0.5)
            outer_angle = ET.SubElement(spot, "outer_angle")
            outer_angle.text = str(blender_light.spot_size)
            falloff = ET.SubElement(spot, "falloff")
            falloff.text = str(blender_light.spot_blend * 10)

        if blender_light.type == "SUN":
            light = ET.SubElement(link, "light", attrib={"name": l.name, "type": "directional"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"

        if blender_light.type in ["SUN", "SPOT"]:
            direction = ET.SubElement(light, "direction")
            direction.text = f"{l.matrix_world[0][2]} {l.matrix_world[1][2]} {l.matrix_world[2][2]}"

        # unsupported: AREA lights

        cast_shadows = ET.SubElement(light, "cast_shadows")
        cast_shadows.text = "true"

        # todo : bpy.types.light script api lacks an intensity value, possible candidate is energy/power(Watts)?
        intensity = ET.SubElement(light, "intensity")
        intensity.text = str(blender_light.energy)

    ## sdf collision tags
    collision = ET.SubElement(link, "collision", attrib={"name": "collision"})

    geometry = ET.SubElement(collision, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    uri = ET.SubElement(mesh, "uri")
    uri.text = path.join(meshes_folder_prefix, dae_filename)

    surface = ET.SubElement(collision, "surface")
    contact = ET.SubElement(surface, "contact")
    collide_bitmask = ET.SubElement(contact, "collide_bitmask")
    collide_bitmask.text = "0x01"

    ## sdf write to file
    xml_string = ET.tostring(sdf, encoding="unicode")
    reparsed = minidom.parseString(xml_string)
    with open(path.join(prefix_path, sdf_filename), "w") as sdf_file:
        sdf_file.write(reparsed.toprettyxml(indent="  "))

    ##############################
    ### generate model.config ####
    ##############################
    model = ET.Element("model")
    name = ET.SubElement(model, "name")
    name.text = model_name
    version = ET.SubElement(model, "version")
    version.text = "1.0"
    sdf_tag = ET.SubElement(model, "sdf", attrib={"version": "1.8"})
    sdf_tag.text = sdf_filename

    author = ET.SubElement(model, "author")
    name = ET.SubElement(author, "name")
    name.text = "Generated by blender SDF tools"

    xml_string = ET.tostring(model, encoding="unicode")
    reparsed = minidom.parseString(xml_string)
    with open(path.join(prefix_path, model_config_filename), "w") as config_file:
        config_file.write(reparsed.toprettyxml(indent="  "))

#### UI Handling ####
class OT_TestOpenFilebrowser(Operator, ImportHelper):
    bl_idname = "test.open_filebrowser"
    bl_label = "Save"

    directory: StringProperty(name="Outdir Path")

    def execute(self, context):
        """Do the export with the selected file."""
        if not path.isdir(self.directory):
            print(f"{self.directory} is not a directory!")
        else:
            print(f"exporting to directory: {self.directory}")
            export_sdf(self.directory)
        return {"FINISHED"}

def register():
    bpy.utils.register_class(OT_TestOpenFilebrowser)

def unregister():
    bpy.utils.unregister_class(OT_TestOpenFilebrowser)

if __name__ == "__main__":
    register()
    bpy.ops.test.open_filebrowser("INVOKE_DEFAULT")

# alternatively comment the main code block and do a function call without going through all the ui
# prefix_path = '/home/ddeng/blender_lightmap/final_office/office/'
# export_sdf(prefix_path)
