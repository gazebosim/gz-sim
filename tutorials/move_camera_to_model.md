\page move_camera_to_model Move Camera to model

This tutorial gives an introduction to Gazebo's service `/gui/move_to/model`. This service will allow to move the camera to a specific model taking into account distance to the model and camera angle.

## How to move the camera to a model

1. Load the **View Angle** plugin. This service is only available when the **View Angle** plugin is loaded.
2. Call the service using the request message type `gz.msgs.GUICamera` and the response message type `gz.msgs.Boolean`. The distance to the object is defined as the z coordinate, and the direction of the camera with a quaternion. It's possible to select the projection type.

For example, Let's move the camera to the `box` model looking down from 5 meters away.

```bash
gz service  -s /gui/move_to/model --reqtype gz.msgs.GUICamera  --reptype gz.msgs.Boolean -r 'name: "box", pose: {position: {z:5}, orientation: {x:0, y:0, z: -1, w:0}}, projection_type: "orbit"' --timeout 5000
```

@image html files/move_camera_to_model/box_5.gif

The camera can also be placed far away, for example 20 meters:

```bash
gz service  -s /gui/move_to/model --reqtype gz.msgs.GUICamera  --reptype gz.msgs.Boolean -r 'name: "box", pose: {position: {z:20}, orientation: {x:0, y:0, z: -1, w:0}}, projection_type: "orbit"' --timeout 5000
```

@image html files/move_camera_to_model/box_20.gif

In the following gif you can see the workflow of this service:

@image html files/move_camera_to_model/move_to_model.gif
