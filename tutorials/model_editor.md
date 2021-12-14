\page model_editor Model Editor

## Overview

Simulation worlds and models are defined using [SDF](https://sdformat.org),
which is an XML based file format. Your favorite text editor can be used to
construct worlds and models, and then run using Ignition Gazebo. While XML
files are human readable, they are not easy to manually parse or create.

Graphical tools for creating world and models was introduced in the Fortress
distribution of Ignition. The graphical tools, provided as plugins to
Ignition Gazebo's GUI, support adding models, links, joints, and sensors.
Entity properties can also be visualized and modified.

## Component inspector editor

Graphical editing capabilities currently (as of Fortress) are available in the
`Component inspector editor` GUI plugin, rather the more commonly used
`Component inspector`. This is done while edit features are under development, and not yet polished for prime-time use.

Make sure to load the `Component inspector editor` plugin in order to
access the capabilities discussed in the remainder of this tutorial.

## Simulation state

The ability to add and edit certain items via the GUI depends on whether
simulation is running or paused. This distinction is important due to the
interplay of physics with edits made via the GUI. For example, adding or
removing a link to/from model would require a major change to physics that
typically can't happen while simulation is running.

Creating new models and changing the pose of a model can be accomplished
while simulation is running. However, most other operations are not
permitted while in the running state. Graphical elements that are not
available while running will be disabled.

## Adding a model or light

A model or light can be inserted into a world using the `+` button on the
`Entity Tree`. The following images highlights the plus button with a red
square.

![Entity Tree Plus](https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/tutorials/files/model_editor/entity_tree_plus.png)

A menu listing different types of models and lights opens when the `+`
button is selected. Selecting an item from the menu will insert the
selection. Move your mouse cursor around in the 3D scene to place the new
object. Left click once when you are satisfied with the new object's
location.

Your new object will appear in the `Entity Tree`. At this point you can
start simulation (if it's not already running), or your can continue
creating and modifying simulation content.

## Adding a link, joint, or light to a model

Make sure the `Component inspector editor` GUI plugin is loaded. Simulation
must also be paused in order to add entities to a model.

Select a model in the `Entity Tree`, then select the `+` button on the
`Component inspector editor` plugin. The following image highlights the
component inspector plus button with a red square. In this example, we would
be adding an entity to the selected `box` entity.

![Inspector Plus](https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/main/tutorials/files/model_editor/inspector_plus.png)

A menu listing different types of links, joints, and lights will appear.
Select the entity type you would like to add to the model. The menu will close
and you'll see the new entity as a child of the selected model in the `Entity
Tree`. The 3D scene will also display the new entity.

At this point, the GUI knows about the existence of the entity but the server
does not. Once play or step is pressed, then the GUI will transmit
information about this new link to the server

You can also edit properties of the new entity before or after playing/stepping
simulation. Just select the newly added entity in the Entity Tree and use
the `Component inspector editor` to make changes.

## Adding a sensor

A sensor can be added to a link in a manner similar to adding a link, joint,
or light to model. Make sure the `Component inspector editor` GUI plugin
is loaded, and simulation is paused.

Select a link in the `Entity Tree`, then select the `+` button on the
`Component inspector editor` plugin.

A menu listing different types of sensors will appear.  Select the sensor
type you would like to add to the link. The menu will close and you'll see
the new sensor as a child of the selected link in the `Entity Tree`. 

Change sensor properties through the `Component inspector editor` by
selecting the new new sensor in the `Entity Tree`. Step simulation to inform
the server about the new sensor when ready.
