\page dynamicdetachablejoints Dynamic Detachable Joints

The `DynamicDetachableJoint` system creates and removes a fixed joint between
a configured parent link and a child link chosen at runtime. It is useful when
the parent model is known in the SDF, but the child model to attach is selected
during simulation.

Use \ref detachablejoints "DetachableJoint" when the child model and link are
known in the SDF. Use `DynamicDetachableJoint` when the child model and link are
chosen at runtime through a service request.

The system has the following parameters:

* `<parent_link>` (optional): Default name of the link in the model containing
this system that will be used as the parent link in the detachable joint when
`parent_link_name` is omitted from the service request.

* `<attach_distance>` (optional): Maximum distance in meters between the parent
link and requested child link. Attach requests are rejected during simulation
update when the links are farther apart. The default is 0.1 m.

* `<service_name>` (optional): Service name for attach and detach requests. If
empty, the default service name is
`/model/<model_name>/dynamic_detachable_joint/attach_detach`. A blocking
variant is also advertised at `<service_name>/blocking`.

* `<output_topic>` (optional): Topic name used to publish the attachment state.
If empty, the default topic name is
`/model/<model_name>/dynamic_detachable_joint/state`.

Here is a minimal plugin configuration:

```xml
<plugin filename="gz-sim-dynamic-detachable-joint-system"
        name="gz::sim::systems::DynamicDetachableJoint">
  <parent_link>parent_link</parent_link>
  <service_name>/payload/attach_detach</service_name>
  <output_topic>/payload/child_state</output_topic>
  <attach_distance>0.25</attach_distance>
</plugin>
```

Attach a child link with:

```bash
gz service -s /payload/attach_detach \
  --reqtype gz.msgs.AttachDetachRequest \
  --reptype gz.msgs.Result \
  --timeout 3000 \
  --req 'child_model_name:"red_cube" child_link_name:"link" command:ATTACH'
```

Detach the same child link with:

```bash
gz service -s /payload/attach_detach \
  --reqtype gz.msgs.AttachDetachRequest \
  --reptype gz.msgs.Result \
  --timeout 3000 \
  --req 'child_model_name:"red_cube" child_link_name:"link" command:DETACH'
```

The service response is `gz.msgs.Result`. On `<service_name>`, an `error_code`
of 0 means the request was accepted and queued for the next `PreUpdate` step.
To wait for `PreUpdate` to execute the joint creation or removal and return any
runtime error (such as missing entities or exceeding `<attach_distance>`), call
`<service_name>/blocking` instead. You can also pass `parent_link_name` in
`gz.msgs.AttachDetachRequest` to attach or detach using a specific parent link
on the model, allowing a single plugin instance to manage multiple parent/child
pairs simultaneously.

The state topic publishes `gz.msgs.Entity`. When a child link is attached, the
message contains the attached link entity ID with type `LINK`. When detached,
the message contains `kNullEntity` with type `NONE`.

The ROS 2 bridge can call the same Gazebo service using
`ros_gz_interfaces/srv/AttachDetach`:

```bash
ros2 service call /payload/attach_detach \
  ros_gz_interfaces/srv/AttachDetach \
  "{child_model_name: 'red_cube', child_link_name: 'link', command: 1}"
```

where `command: 1` is `ATTACH` and `command: 2` is `DETACH`.

An example world is available at `examples/worlds/dynamic_detachable_joint.sdf`.
