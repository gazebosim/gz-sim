\page meshtofuel Importing a Mesh to Fuel

This tutorial will explain how to import a mesh to the [Gazebo Fuel](https://app.gazebosim.org) web application.
Adding models and/or worlds to Fuel will make your content readily available to the open source robotics simulation community, and easier to use with the Gazebo GUI.

## Prerequisites

To import meshes to Fuel, you need to have a user account.
Go to [app.gazebosim.org](https://app.gazebosim.org) and click Login in the top right corner of the screen, then click Sign Up.
Once you verify your email address, your account will be ready.

You'll need a mesh ready before trying to import to Fuel.
There are several ways to acquire a mesh. <!--point cloud to mesh tutorial, cad to mesh tutorial-->
To save time, we'll use this [Electrical Box model](https://app.gazebosim.org/openrobotics/fuel/models/Electrical%20Box) that you can download from Fuel.

## Model Directory Structure

Fuel expects meshes to be contained inside model directories that follow a specific structure and format.
Download and unzip the Electrical Box model mentioned above, and you should see the structure of its contents as follows:

```
Electrical Box
├── materials               Directory for textures
    └── textures
├── meshes                  Directory for COLLADA, STL, and Wavefront OBJ files
├── thumbnails              Directory for preview images
├── model.config            Meta data about the model
└── model.sdf               SDF description of the model
```

A model directory needs to adhere to this file structure to successfully import to Fuel.
The contents of each directory and file are equally as important; each part is covered below.

### Materials

The `materials` directory is for textures.
Textures are optional; they provide color or detail to a mesh.
If you download a mesh online, the texture files should come with it.
Otherwise, you would have to use a 3D modeling tool to correctly design a texture.

The Electrical Box's texture is under `materials` > `textures` > `Electrical Box.png`.

### Meshes

The `meshes` directory is where the actual mesh files go.
The mesh could be one of several 3D model file types: `.dae`, `.stl`, `.obj`, etc.
This file is produced with a 3D modeling tool.

The Electrical Box's mesh is under `meshes` > `Electrical Box.dae`

### Thumbnails

Thumbnails are `.png` files named `1` through `5`.
Each contains a view of the model from a different angle to be displayed on the image carousel on its "Fuel Model Info" page.

### model.config

The `model.config` file is required in the root of the model directory.
It contains meta data about the model in XML format.
You have to create this file and add it to the model directory.

Here is the Electrical Box's `model.config`:

```xml
<?xml version='1.0'?>
<model>
  <name>Electrical Box</name>
  <version>1.0</version>
  <sdf version='1.6'>model.sdf</sdf>

  <description>
    Generated with mesh_to_sdf
    https://bitbucket.org/osrf/mesh_to_sdf
  </description>
</model>
```

Customize the `name` and `description` tags to fit your model.
The `sdf version` should ideally be the latest version.
Inside the `sdf` tags is the name of the SDF file describing your model.

### model.sdf

The Electrical Box's `model.sdf` file is as follows:

```xml
<?xml version='1.0' ?>
<sdf version='1.6'>
  <model name='Electrical Box'>
    <static>true</static>
    <link name='link'>
      <pose>0 0 0 1.57 3.14159 0</pose>
      <collision name='collision'>
        <geometry>
          <mesh>
            <uri>model://Electrical Box/meshes/Electrical Box.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>model://Electrical Box/meshes/Electrical Box.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

You can learn more about SDF [here](http://sdformat.org/).
For this tutorial, it's mostly only important to take note of the `<uri>` tags.
Following `model://` is the path within the model directory to the mesh.

## Upload to Fuel

Once you have all your content accumulated in a model directory, you're ready to upload to Fuel.
Click on the `+` button at the top right corner of the page, next to your account button, and then select `Model`.

Typically, you would try to ensure that the `Model name` field matches the `name` tags in both `model.config` and `model.sdf`, and the `Description` field matches its corresponding tag in `model.config`.

For the sake of this tutorial, however, we're going to enter your model name as `Electrical Box Test` to differentiate it from the original. In the `Description` field, write something like `Test model for Importing a Mesh to Fuel tutorial`.

Click the `Add folders` button, or drag and drop the `Electrical Box` folder you unzipped earlier into the labeled area.
All the files in your model description will be listed there.
Press `Upload`, and the "Fuel Model Info" page for your model will open.

![Electrical Box Test](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mesh_to_fuel/model_info2.png)

You can always delete a model by clicking the "Edit model" button and then selecting "Delete model" at the bottom of the page

![Delete model](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mesh_to_fuel/delete2.png)

## Include the Model in a World

With your mesh successfully uploaded to Fuel, you can now easily include it in a world SDF file.

Copy [this example world code](https://github.com/gazebosim/gz-sim/raw/main/examples/worlds/import_mesh.sdf) into a text editor and save it as `import_mesh.sdf`.
This is a simple world SDF file, which you can learn more about on the [SDF website](http://sdformat.org/).

Scroll all the way to the bottom of the file until you see the `include` tag section following the `<!-- mesh -->` comment line.

```xml
<?xml version="1.0" ?>
<!--
  Demo world to show how to include a model from Gazebo Fuel.
-->
<sdf version="1.6">
  <world name="fuel">

    ....

    <!-- mesh -->
    <include>
      <static>true</static>
      <name>Electrical Box</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>

  </world>
</sdf>
```

The address between the `uri` tags is how you include your Fuel model in the world.
To include *your* model, not the original, change `openrobotics` to your Fuel username.
You also need to customize the last section of the URI to match your model's name on Fuel.
Change `Electrical Box` to `Electrical Box Test`.

The syntax for including any model from Fuel is:

```xml
<uri>https://fuel.gazebosim.org/1.0/<user account name>/models/<model name></uri>
```

### Launch World

To launch the world and see your mesh, run Gazebo from inside the directory where you saved `import_mesh.sdf`:

```bash
gz sim import_mesh.sdf
```

![Launch sample world with mesh](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mesh_to_fuel/launch_world2.png)
