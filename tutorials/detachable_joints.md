\page detachablejoints Detachable Joints

The `DetachableJoint` system allows two models to start off rigidly attached
and then detach during simulation by publishing to a topic. The system
internally uses a fixed joint between two links, each belonging to two separate
models. Because the system uses joints to connect models, the resulting
kinematic topology has to be a tree, i.e., kinematic loops are not currently
supported. This affects the choice of the parent link, and therefore, the
parent model, which is the model that contains the `DetachableJoint` system.
Once detached, the joint can be re-attached by publishing to a topic.
When reattaching, the child model will be attached to the parent model at its
current pose/configuration. To achieve reattachment at a specific pose, the
child model can be positioned accordingly through a set_pose service call prior
to reattaching the joint.

For example, [detachable_joint.sdf](https://github.com/gazebosim/gz-sim/blob/ign-gazebo2/examples/worlds/detachable_joint.sdf)
demonstrates a four wheel vehicle that holds three objects that are later
detached from the vehicle. As seen in this example, the parent model is the
vehicle. The kinematic topology is the following.

```
world---vehicle_blue---B1
                    \
                      ---B2
                      \
                        ---B3
```
If the objects were each parent models, instead, there would be multiple
kinematic loops, as shown below.

```
world---B1---vehicle_blue
     \      /   /
       ---B2   /
       \      /
         ---B3
```

Due to a limitation in the implementation of this system, if detached models
need to collide with a parent model or other detached models that have the same
parent, the parent model needs to have `<self_collide>` set to true. However,
due to an issue in DART, the default physics engine, it is important that none of the parent
or child models be in collision in their initial (attached) state.
Furthermore, it is important to note that reattaching a child model is not
currently supported while the child model and parent model are in contact.
Therefore, it is imperative to ensure that there is no collision between the
child and parent model when attempting to perform the reattachment process.

The system has the following parameters:

* `<parent_link>`: Name of the link in the model containing this system that will be
used as the parent link in the detachable joint.

* `<child_model>`: The name of the model containing the child link in the detachable
joint.

* `<child_model_link>`:  Name of the link in the `<child_model>` that will be used
as the child link in the detachable joint.

* `topic` (optional): Topic name to be used for detaching connections. Using
<detach_topic> is preferred. If empty, a default topic will be created with a
pattern `/model/<model_name>/detachable_joint/detach`.

* `detach_topic` (optional): Topic name to be used for detaching connections.
 If empty, a default topic will be created with a pattern
`/model/<model_name>/detachable_joint/detach`. If multiple detachable plugin is
used in one model, `detach_topic` is REQUIRED to detach child models individually.

* `attach_topic` (optional): Topic name to be used for re-attaching connections.
 If empty, a default topic will be created with a pattern
`/model/<model_name>/detachable_joint/attach`. If multiple detachable plugin is
used in one model, `attach_topic` is REQUIRED to attach child models individually.

* `output_topic` (optional): Topic name to be used for publishing the state of
the detachment. If empty, a default topic will be created with a pattern
`/model/<child_model_name>/detachable_joint/state`. If multiple detachable plugin is
used in one model, `output_topic` is REQUIRED to publish child models state
individually.
