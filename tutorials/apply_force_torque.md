\page apply_force_torque Apply Force and Torque

The Apply Force Torque plugin allows users to apply forces and/or torques to
links in the simulation through the graphical user interface.

## Examples

Let's go through an example of applying force and torque to simple models. Open
the `shapes.sdf` world with

```bash
gz sim shapes.sdf
```

From the plugin dropdown, select the `Apply Force Torque` plugin. Make sure the
simulation isn't paused.

\image html files/apply_force_torque/Interface.png width=60%

### Apply force to a link

We want to apply force to the `cylinder` model. Select the model, either by
clicking on it in the scene or through the Entity Tree. If the model had
multiple links, we could select through the interface which one to apply the
force to (in this case, `cylinder_link`).

On the dialog, write `10000` on the `X` field under `Force` and click on
`Apply Force`. The cylinder will be pushed on the X direction. The force was
applied in the link's `X` direction for a single time-step, which is in the
order of milliseconds, thus the need for such a large force.

\image html files/apply_force_torque/Force.png width=60%

### Apply torque to a link

On the dialog, write `2000` on the `X` field under `Torque` and click on
`Apply Torque` to see the cylinder rotate slightly.

\image html files/apply_force_torque/Torque.png width=60%

### Apply force with an offset

By default, the force is applied to the link's center of mass, but this can be
modified through the `Offset` fields. On the dialog, write `1000` on the `X`
field under `Force` and `1` under the `Z` field under `Offset`. Press
`Apply Force` to see the model move slightly in the `X` direction while also
rotating around the `Y` direction.

\image html files/apply_force_torque/ForceOffset.png width=60%

### Rotation tool

On the dialog, write `10000` on the `X` field under `Force`. Click on the force
vector to make the rotation tool appear. Drag the blue circle to rotate the
force around the `Z` axis so that it is aligned with the `Y` direction. Notice
how the XYZ fields changed, but not the magnitude. Press `Apply Force` to see
the model move in the `Y` direction.

\image html files/apply_force_torque/RotationTool.png width=60%

## The interface explained

> **Note**: If you apply force and/or torque while the simulation is paused,
they will accumulate and be applied all at once when the simulation is
unpaused.

### Force

- **Force X, Y, Z**: Each field specifies how much force will be applied on that
direction, in Newtons (N). The frame is fixed to the link.

- **Mag**: The total magnitude of the force which will be applied, which is the
Euclidean norm of the 3 forces above. Changing the magnitude changes the XYZ
fields proportionally, maintaining the force direction.

- **Offset X, Y, Z**: By default, force is applied to the link's center of mass,
in meters. Here you can edit the X, Y and Z fields to give the force an offset
with respect to the center of mass expressed in the link's frame.

  - **Tip**: Right-click the model and choose `View` -> `Center of Mass` to see
    its position. You will want to also make the model transparent to see the
    center of mass visual (`View` -> `Transparent`).

- **Apply Force**: Click this to apply only force for one time step. Keep in
mind that time steps are typically in the order of milliseconds, so relatively large
forces are needed in order to apply a significant impulse.

### Torque

- **X, Y, Z**: Each field specifies how much torque will be applied about that
axis, in Newton-meters (N.m). The frame is fixed to the link.

- **Mag**: The total magnitude of the torque which will be applied, which is the
Euclidean norm of the 3 torques above. Changing the magnitude changes the XYZ
fields proportionally, maintaining the torque direction.

- **Apply Torque**: Click this to apply only torque for one time step. Keep in
mind that time steps are typically in the order of milliseconds, so relatively large
torques are needed in order to apply a significant angular impulse.

  - **Note**: Torque is always applied about the center of mass.

### Apply All

Force and torque are applied at the same time, i.e. apply a wrench.

### Rotation Tool

The vector (force or torque) directions will always match the directions
specified in the dialog. From the dialog, the direction can be changed by
editing the numbers on the XYZ fields.

From the scene, select a vector to enable the rotation tool, then drag the
handles. This changes the direction of the vector, updating the XYZ fields
accordingly without modifying its magnitude. You may click again on the vector
to unselect the rotation tool.
