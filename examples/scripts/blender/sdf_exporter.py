import bpy
import os.path
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty
from bpy.types import Operator

import xml.etree.ElementTree as ET
from xml.dom import minidom

def export_sdf(prefix_path):
    dae_filename = 'model.dae'
    sdf_filename = 'model.sdf'
    model_config_filename = 'model.config'
    lightmap_filename = 'LightmapBaked.png'
    model_name = 'my_model'
    meshes_folder_prefix = 'meshes/'
    
    # Exports the dae file and its associated textures
    bpy.ops.wm.collada_export(filepath=prefix_path+meshes_folder_prefix+dae_filename, 
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
                             filemode=8)

    objects = bpy.context.selectable_objects
    mesh_objects = [o for o in objects if o.type == 'MESH']
    light_objects = [o for o in objects if o.type == 'LIGHT']

    sdf = ET.Element('sdf', attrib={'version':'1.8'})
    model = ET.SubElement(sdf, "model", attrib={"name":"test"})
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", attrib={"name":"testlink"})

    def get_diffuse_map(material):
        """Helper function to safely get diffuse map from material"""
        if not material or not material.use_nodes or not material.node_tree:
            return ""
        
        nodes = material.node_tree.nodes
        principled = next((n for n in nodes if n.type == 'BSDF_PRINCIPLED'), None)
        
        if not principled:
            return ""
            
        base_color = principled.inputs.get('Base Color') or principled.inputs[0]
        if not base_color.links:
            return ""
            
        link_node = base_color.links[0].from_node
        if not hasattr(link_node, 'image') or not link_node.image:
            return ""
            
        return link_node.image.name

    # for each geometry in geometry library add a <visual> tag
    for o in mesh_objects:
        visual = ET.SubElement(link, "visual", attrib={"name":o.name})
        
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        uri = ET.SubElement(mesh, "uri")
        uri.text = dae_filename
        submesh = ET.SubElement(mesh, "submesh")
        submesh_name = ET.SubElement(submesh, "name")
        submesh_name.text = o.name
        
        # Material handling
        material = ET.SubElement(visual, "material")
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = "1.0 1.0 1.0 1.0"
        specular = ET.SubElement(material, "specular")
        specular.text = "0.0 0.0 0.0 1.0"
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        
        # Get diffuse map if material exists
        if o.active_material:
            diffuse_map = get_diffuse_map(o.active_material)
            if diffuse_map:
                albedo_map = ET.SubElement(metal, "albedo_map")
                albedo_map.text = meshes_folder_prefix + diffuse_map
        
        # Lightmap handling
        if os.path.isfile(lightmap_filename):
            light_map = ET.SubElement(metal, "light_map", attrib={"uv_set":"1"})
            light_map.text = meshes_folder_prefix + lightmap_filename
            cast_shadows = ET.SubElement(visual, "cast_shadows")
            cast_shadows.text = "0"

    def add_attenuation_tags(light_tag, blender_light):
        """Helper function to add attenuation tags based on light type"""
        attenuation = ET.SubElement(light_tag, "attenuation")
        
        # Range (cutoff distance)
        range_tag = ET.SubElement(attenuation, "range")
        range_tag.text = str(blender_light.cutoff_distance)
        
        # Linear attenuation factor
        linear = ET.SubElement(attenuation, "linear")
        linear.text = "1.0"  # Default linear attenuation
        
        # Quadratic attenuation factor
        quadratic = ET.SubElement(attenuation, "quadratic")
        quadratic.text = "0.0"  # Default quadratic attenuation
        
        # Constant attenuation factor
        constant = ET.SubElement(attenuation, "constant")
        constant.text = "1.0"  # Default constant attenuation
    
    # Export lights
    for l in light_objects:
        blender_light = l.data
        
        if blender_light.type == "POINT":
            light = ET.SubElement(link, "light", attrib={"name":l.name, "type":"point"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"
            add_attenuation_tags(light, blender_light)
            
        elif blender_light.type == "SPOT":
            light = ET.SubElement(link, "light", attrib={"name":l.name, "type":"spot"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"
            add_attenuation_tags(light, blender_light)
            
            # Add spot light specific parameters
            spot = ET.SubElement(light, "spot")
            inner_angle = ET.SubElement(spot, "inner_angle")
            inner_angle.text = str(blender_light.spot_size * 0.5)  # Convert to inner angle
            outer_angle = ET.SubElement(spot, "outer_angle")
            outer_angle.text = str(blender_light.spot_size)  # Outer angle
            falloff = ET.SubElement(spot, "falloff")
            falloff.text = str(blender_light.spot_blend * 10)  # Approximate falloff from blend
            
        elif blender_light.type == "SUN":
            light = ET.SubElement(link, "light", attrib={"name":l.name, "type":"directional"})
            diffuse = ET.SubElement(light, "diffuse")
            diffuse.text = f"{blender_light.color.r} {blender_light.color.g} {blender_light.color.b} 1.0"
            
        if blender_light.type in ["SUN", "SPOT"]:
            direction = ET.SubElement(light, "direction")
            direction.text = f"{l.matrix_world[0][2]} {l.matrix_world[1][2]} {l.matrix_world[2][2]}"

        # Common light properties
        cast_shadows = ET.SubElement(light, "cast_shadows")
        cast_shadows.text = "true"
        
        intensity = ET.SubElement(light, "intensity")
        intensity.text = str(blender_light.energy)  # Use the light's energy as intensity
    
    # Collision tags
    collision = ET.SubElement(link, "collision", attrib={"name":"collision"})
    geometry = ET.SubElement(collision, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    uri = ET.SubElement(mesh, "uri")
    uri.text = dae_filename
    surface = ET.SubElement(collision, "surface")
    contact = ET.SubElement(collision, "contact")
    collide_bitmask = ET.SubElement(collision, "collide_bitmask")
    collide_bitmask.text = "0x01"

    # Write SDF file
    xml_string = ET.tostring(sdf, encoding='unicode')
    reparsed = minidom.parseString(xml_string)
    with open(prefix_path+sdf_filename, "w") as sdf_file:
        sdf_file.write(reparsed.toprettyxml(indent="  "))

    # Generate model.config
    model = ET.Element('model')
    name = ET.SubElement(model, 'name')
    name.text = model_name
    version = ET.SubElement(model, 'version')
    version.text = "1.0"
    sdf_tag = ET.SubElement(model, "sdf", attrib={"version":"1.8"})
    sdf_tag.text = sdf_filename

    author = ET.SubElement(model, 'author')
    name = ET.SubElement(author, 'name')
    name.text = "Generated by blender SDF tools"

    xml_string = ET.tostring(model, encoding='unicode')
    reparsed = minidom.parseString(xml_string)
    with open(prefix_path+model_config_filename, "w") as config_file:
        config_file.write(reparsed.toprettyxml(indent="  "))

class OT_TestOpenFilebrowser(Operator, ImportHelper):
    bl_idname = "test.open_filebrowser"
    bl_label = "Save"
    
    directory: bpy.props.StringProperty(name="Outdir Path")
    
    def execute(self, context):
        if not os.path.isdir(self.directory):
            print(self.directory + " is not a directory!")
        else:
            print("exporting to directory: " + self.directory)
            export_sdf(self.directory)
        return {'FINISHED'}

def register(): 
    bpy.utils.register_class(OT_TestOpenFilebrowser) 

def unregister(): 
    bpy.utils.unregister_class(OT_TestOpenFilebrowser)
    
if __name__ == "__main__":
    register() 
    bpy.ops.test.open_filebrowser('INVOKE_DEFAULT')