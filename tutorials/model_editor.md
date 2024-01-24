\page model_editor Model Editor

## Overview

Simulation worlds and models in Gazebo are defined using [SDF](https://sdformat.org), an XML-based file format. While these files are
human-readable, they can be challenging to manually parse or create. Gazebo
has introduced graphical tools to ease the process, allowing users to
construct worlds and models with greater ease.

## Component inspector editor

Graphical model editing capabilities are available through the `Component
Inspector Editor` GUI plugin, rather than the more commonly used `Component
Inspector`. This distinction is relevant during the development of editing
features. To access the features discussed in this tutorial, ensure you load
the `Component Inspector Editor` plugin.

## Simulation state

The ability to add and edit certain items via the GUI depends on whether the
simulation is running or paused. This is crucial due to the interplay of
physics with edits made through the GUI. For instance, major changes to
physics, like adding or removing a link from a model, generally cannot occur
while the simulation is running.

While simulation is running, you can create new models and change the pose
of a model. However, most other operations are not permitted in the running
state. Graphical elements that are not available during runtime will be
disabled.

## Adding a model or light

To insert a model or light into a world, use the `+` button on the `Entity Tree`.

![Entity Tree Plus](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim7/tutorials/files/model_editor/entity_tree_plus.png)

Selecting the `+` button opens a menu listing different types of models and
lights. Choose an item from the menu, move your mouse cursor in the 3D scene
to place the new object, and left-click to confirm. The new object will
appear in the `Entity Tree`, and you can start or continue modifying the
simulation content.

## Adding a link, joint, or light to a model

To add entities to a model, simulation must be paused. Select a model in
the Entity Tree, then click the + button on the Component Inspector Editor
plugin. In the example below, we are adding an entity to the selected box
entity.

@image html files/model_editor/add_link.png "Inspector Plus" width=60%

A menu displaying different types of links, joints, and lights will appear.
Select the desired entity type to add to the model. The new entity will
appear as a child of the selected model in the `Entity Tree`, and the 3D scene
will display the new entity. The GUI is aware of the entity's existence at
this point, but the server is not. Upon pressing play or step, the GUI
transmits information about the new link to the server.

You can edit properties of the new entity before or after playing/stepping
simulation by selecting the entity in the `Entity Tree` and using the
`Component Inspector Editor` to make changes.

## Adding a sensor

Adding a sensor is similar to adding a link, joint, or light to a model.
Ensure the `Component Inspector Editor` GUI plugin is loaded, and the
simulation is paused. Select a link in the `Entity Tree`, then click the
`+` button on the `Component Inspector Editor` plugin.

A menu listing different types of sensors will appear. Select the desired
sensor type to add to the link. The new sensor will appear as a child of the
selected link in the `Entity Tree`. Change sensor properties through the
`Component Inspector Editor` by selecting the new sensor in the `Entity Tree`.
Step the simulation to inform the server about the new sensor when ready.
