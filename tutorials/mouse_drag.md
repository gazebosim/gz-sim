\page mouse_drag Mouse Drag

The Mouse Drag plugin allows the user to exert forces and/or torques by dragging
objects in the scene with the mouse cursor. It has two modes: rotation and
translation.

To use it, open any world (such as `shapes.sdf`) and select `Mouse Drag` from
the plugin dropdown to load the plugin.

![Interface](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mouse_drag/Interface.png)

## Rotation mode

The rotation mode is activated by Ctrl+Left-clicking and holding a link in the
scene. Dragging the mouse causes a pure torque to be applied, contained in a
plane parallel to the camera. The transparent red bounding box displays the
desired orientation of the link corresponding to the current mouse position.

The magnitude of the torque is calculated by a spring-damper system with a
constant stiffness and critical damping. It is also proportional to the link's
inertia, so that the same stiffness causes similar effects on different links.
The rotational stiffness can be modified through the interface.

![Rotation mode](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mouse_drag/Rotation.png)

## Translation mode

The translation mode is activated by Ctrl+Right-clicking and holding a link in
the scene. Dragging the mouse will then move the link towards the mouse
position. On the interface, you may select whether the force should be applied
to the link's center of mass or to the point where the mouse click occured.

If center of mass is selected, only a force is applied, with a magnitude given
by a constant stiffness and critical damping, scaled by the mass of the link.
The force is always contained in a plane parallel to the camera and passing
through the application point, represented by a transparent gray plane. The
visualization also shows an arrow from the application point to the target
position under the mouse cursor.

If center of mass is not selected, an additional torque is applied to account
for the offset in the force application point. In this case, the rotation of
the object is also slightly damped (according to the rotational stiffness).
The position stiffness can be modified through the interface.

![Translation mode](https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/mouse_drag/Translation.png)

## SDF configuration

The rotation and position stiffness may also be configured in the SDF file.
To do this, add the `<rotation_stiffness>` and/or `<position_stiffness>` to the
plugin element under `<gui>`.

```xml
<sdf version="1.6">
  <world name="mouse_drag">
    <gui>
      <plugin filename="MouseDrag">
        <rotation_stiffness>100.0</rotation_stiffness>
        <position_stiffness>100.0</position_stiffness>
      </plugin>
      ...
    </gui>
    ...
  </world>
</sdf>
```
